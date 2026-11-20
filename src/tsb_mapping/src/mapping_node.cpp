// mapping_node.cpp
//
// Nó ROS 2 que constrói um mapa de ocupação 2D a partir de:
//   /odom : posição e orientação do barco no referencial global "map"
//   /scan : distâncias medidas pelo LiDAR no referencial "base_link"
//
// Publica:
//   /map  : nav_msgs/msg/OccupancyGrid, com células desconhecidas (-1)
//           e probabilidades de ocupação (0 a 100)
//
// Este ficheiro trata da comunicação ROS 2 e da sincronização dos dados.
// Os cálculos de Bresenham e das probabilidades estão em grid_core.hpp.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tsb_mapping/grid_core.hpp"

class MappingNode : public rclcpp::Node
{
public:
  MappingNode() : Node("mapping_node")
  {
    // 1. Parâmetros da grelha, configuráveis em config/mapping.yaml.
    // A área é 28 x 28 m, dividida em células de 10 cm.
    // É maior do que o mundo simulado (-12 a +12 m) para incluir as paredes.
    const int width = declare_parameter<int>("width", 280);
    const int height = declare_parameter<int>("height", 280);
    const double resolution = declare_parameter<double>("resolution", 0.1);
    const double origin_x = declare_parameter<double>("origin_x", -14.0);
    const double origin_y = declare_parameter<double>("origin_y", -14.0);

    // 2. Modelo probabilístico do LiDAR.
    // Cada célula atravessada por um raio recebe evidência de espaço livre.
    // A célula onde o raio termina por colisão recebe evidência de ocupação.
    // O GridCore acumula estas evidências em log-odds, com limites definidos.
    const double p_free = declare_parameter<double>("p_free", 0.35);
    const double p_occupied = declare_parameter<double>("p_occupied", 0.75);
    const double log_min = declare_parameter<double>("log_odds_min", -5.0);
    const double log_max = declare_parameter<double>("log_odds_max", 5.0);

    // A atualização da grelha acontece quando chega um scan emparelhado.
    // Este parâmetro controla apenas a frequência de publicação em /map.
    const double publish_hz = declare_parameter<double>("publish_rate_hz", 2.0);
    max_pending_ = declare_parameter<int>("max_pending_scans", 30);
    if (publish_hz <= 0.0 || max_pending_ <= 0) {
      throw std::invalid_argument("publish_rate_hz e max_pending_scans devem ser positivos");
    }

    map_load_time_ = now();
    grid_ = std::make_unique<tsb_mapping::GridCore>(
      width, height, resolution, origin_x, origin_y,
      p_free, p_occupied, log_min, log_max);

    // 3. Criar os tópicos ROS 2.
    // transient_local permite a um visualizador receber o último mapa
    // mesmo que se ligue depois de a publicação ter começado.
    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);

    // Subscrevemos separadamente a odometria e o scan.
    // Os callbacks guardam as mensagens até ser possível emparelhá-las.
    odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&MappingNode::on_odometry, this, std::placeholders::_1));
    scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MappingNode::on_scan, this, std::placeholders::_1));

    // Publicar o estado atual da grelha periodicamente (por defeito, 2 Hz).
    const auto period = std::chrono::duration<double>(1.0 / publish_hz);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MappingNode::publish_map, this));

    RCLCPP_INFO(get_logger(),
      "Mapper iniciado: /odom + /scan -> /map (%dx%d, %.2f m/celula)",
      width, height, resolution);
  }

private:
  // O mapping é 2D: apenas precisamos de x, y e orientação (yaw).
  struct Pose2D
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  // O simulador atribui o MESMO timestamp à odometria e ao LiDAR de cada tick.
  // Convertemos esse instante para nanosegundos para procurar pares exatos.
  // Isto evita aplicar um scan à posição do barco de outro instante.
  static int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // A solução pressupõe os frames usados pelo NOSSO simulador.
    // Não é uma implementação genérica para odometria de qualquer robô.
    if (msg->header.frame_id != "map" || msg->child_frame_id != "base_link") {
      RCLCPP_WARN_ONCE(get_logger(), "Esperava odometria map -> base_link; verificar frames");
      return;
    }

    const auto & position = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;

    // ROS 2 representa orientações por quaternions (x, y, z, w).
    // Transformamos o quaternion no ângulo yaw (rotação em torno de Z),
    // que é o único ângulo necessário para o movimento num plano 2D.
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // Guardar a pose pelo timestamp. Limitamos a memória a um número
    // pequeno de poses recentes, pois não precisamos de todo o histórico.
    poses_[stamp_ns(msg->header.stamp)] = Pose2D{position.x, position.y, yaw};
    while (static_cast<int>(poses_.size()) > max_pending_) {
      poses_.erase(poses_.begin());
    }

    // Pode já existir um scan com o mesmo timestamp à espera desta pose.
    process_available_scans();
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // O LiDAR do simulador está no centro do barco (base_link).
    // Por isso não precisamos de uma transformação adicional do sensor.
    if (msg->header.frame_id != "base_link") {
      RCLCPP_WARN_ONCE(get_logger(), "Esperava LaserScan no frame base_link");
      return;
    }

    // O scan pode chegar antes da odometria correspondente.
    // Guardamo-lo temporariamente, em vez de usar a pose mais recente.
    pending_scans_.push_back(msg);
    while (static_cast<int>(pending_scans_.size()) > max_pending_) {
      pending_scans_.pop_front();
      RCLCPP_WARN_ONCE(get_logger(), "Scans sem pose correspondente foram descartados");
    }

    process_available_scans();
  }

  void process_available_scans()
  {
    // Procurar, para cada scan em espera, uma pose com timestamp igual.
    // Quando encontramos o par, atualizamos a grelha e retiramos o scan
    // da fila: a mesma medição não deve ser integrada duas vezes.
    for (auto it = pending_scans_.begin(); it != pending_scans_.end();) {
      const auto pose_it = poses_.find(stamp_ns((*it)->header.stamp));
      if (pose_it == poses_.end()) {
        ++it;  // A pose ainda não chegou; tentar novamente mais tarde.
        continue;
      }

      update_map_from_scan(*(*it), pose_it->second);
      it = pending_scans_.erase(it);
    }
  }

  void update_map_from_scan(const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose)
  {
    if (scan.angle_increment <= 0.0f) {
      return;  // Scan sem incremento angular válido.
    }

    // Cada elemento ranges[i] representa a distância observada num raio.
    // O ângulo do raio no mundo é a orientação do barco + ângulo do sensor.
    // O GridCore calcula a extremidade, percorre as células com Bresenham
    // e atualiza os log-odds de livre/ocupado.
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double relative_angle =
        scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double global_angle = pose.yaw + relative_angle;

      grid_->integrate_beam(
        pose.x, pose.y, global_angle,
        scan.ranges[i], scan.range_min, scan.range_max);
    }

    // Contamos scans integrados, não o número de raios individualmente.
    ++processed_scans_;
  }

  void publish_map()
  {
    nav_msgs::msg::OccupancyGrid map;

    // Frame e instante da publicação. A grelha não roda em relação a "map".
    map.header.stamp = now();
    map.header.frame_id = "map";
    map.info.map_load_time = map_load_time_;
    map.info.resolution = static_cast<float>(grid_->resolution());
    map.info.width = static_cast<uint32_t>(grid_->width());
    map.info.height = static_cast<uint32_t>(grid_->height());
    map.info.origin.position.x = grid_->origin_x();
    map.info.origin.position.y = grid_->origin_y();
    map.info.origin.orientation.w = 1.0;

    // Conversão de log-odds para o formato esperado pelo ROS:
    // -1: célula nunca observada; 0..100: percentagem de ocupação.
    // O vetor é organizado por linhas, com índice = y * width + x.
    map.data = grid_->occupancy_values();
    map_publisher_->publish(map);

    // Informação de diagnóstico no máximo a cada 5 segundos,
    // para não encher o terminal com mensagens a 2 Hz.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Mapa publicado (%zu scans integrados)", processed_scans_);
  }

  // --- Estado interno do mapper ---
  rclcpp::Time map_load_time_;
  std::unique_ptr<tsb_mapping::GridCore> grid_;  // Grelha e algoritmo matemático.

  int max_pending_ = 30;
  std::size_t processed_scans_ = 0;
  std::map<int64_t, Pose2D> poses_;  // Poses recentes, indexadas por timestamp.
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> pending_scans_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  // Inicializa o ROS 2, executa os callbacks e mantém o nó ativo.
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MappingNode>());
  } catch (const std::exception & error) {
    // Se falhar a configuração (por exemplo, parâmetros inválidos),
    // mostrar a causa e terminar com código de erro.
    RCLCPP_ERROR(rclcpp::get_logger("mapping_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
