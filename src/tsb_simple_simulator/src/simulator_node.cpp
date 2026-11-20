#include <algorithm>   // std::min, std::max, std::clamp, std::swap
#include <chrono>      // Temporizadores
#include <cmath>        // sin, cos, atan2
#include <functional>  // std::bind
#include <limits>      // Infinito
#include <memory>      // std::unique_ptr, std::make_shared
#include <stdexcept>   // std::runtime_error
#include <vector>       // std::vector

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double EPSILON = 1e-12;

// Um obstáculo (ou uma parede) é representado por um retângulo
// cujos lados são paralelos aos eixos X e Y.
struct Rectangle {
  double xmin;
  double ymin;
  double xmax;
  double ymax;
};

// Distância do ponto (x, y) ao ponto mais próximo de um retângulo, ao quadrado.
// Se o ponto estiver dentro do retângulo, a distância é zero.
// É utilizada para detetar colisões entre o barco (círculo) e os obstáculos.
double distance_to_rectangle_squared(double x, double y, const Rectangle & rect)
{
  const double closest_x = std::clamp(x, rect.xmin, rect.xmax);
  const double closest_y = std::clamp(y, rect.ymin, rect.ymax);
  const double delta_x = x - closest_x;
  const double delta_y = y - closest_y;
  return delta_x * delta_x + delta_y * delta_y;
}

// Calcula a distância da origem de um raio à primeira interseção positiva
// com um retângulo. Retorna infinito quando não há interseção à frente.
//
// (origin_x, origin_y): posição do sensor
// (dir_x, dir_y): direção do raio, com comprimento 1
// rect: retângulo a testar
//
// O método calcula os intervalos de distância em que o raio está dentro
// do retângulo, primeiro segundo X e depois segundo Y.
double ray_rectangle_distance(
  double origin_x, double origin_y,
  double dir_x, double dir_y,
  const Rectangle & rect)
{
  double entry = -std::numeric_limits<double>::infinity();
  double exit = std::numeric_limits<double>::infinity();

  // Atualiza os limites de entrada/saída do raio num dos eixos.
  auto intersect_axis = [&](double origin, double direction, double min_value, double max_value) {
    if (std::abs(direction) < EPSILON) {
      // Raio paralelo a este eixo: só pode cruzar o retângulo
      // se já estiver entre os dois limites desse eixo.
      return origin >= min_value && origin <= max_value;
    }

    double t1 = (min_value - origin) / direction;
    double t2 = (max_value - origin) / direction;
    if (t1 > t2) {
      std::swap(t1, t2);
    }

    entry = std::max(entry, t1);
    exit = std::min(exit, t2);
    return entry <= exit;
  };

  if (!intersect_axis(origin_x, dir_x, rect.xmin, rect.xmax) ||
      !intersect_axis(origin_y, dir_y, rect.ymin, rect.ymax)) {
    return std::numeric_limits<double>::infinity();
  }

  if (exit < 0.0) {
    return std::numeric_limits<double>::infinity(); // Interseção atrás do sensor
  }

  // Se o sensor estiver fora do retângulo, a primeira interseção é a entrada.
  // Se estiver dentro (como acontece com os limites do mundo), é a saída.
  return entry >= 0.0 ? entry : exit;
}

} // namespace

class SimpleSimulator : public rclcpp::Node
{
public:
  SimpleSimulator() : Node("simple_simulator")
  {
    load_parameters();
    setup_ros_communication();

    last_update_ = now();

    // O temporizador chama update() periodicamente (10 vezes por segundo,
    // quando rate_hz = 10). Não é necessário criar um ciclo while manual.
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimpleSimulator::update, this));

    RCLCPP_INFO(
      get_logger(), "Simulador iniciado: /odom, /scan, /cmd_vel; movimento automatico=%s",
      auto_motion_ ? "true" : "false");
  }

private:
  // -----------------------------------------------------------------------
  // 1. CONFIGURAÇÃO DO MUNDO E DO BARCO
  // -----------------------------------------------------------------------
  void load_parameters()
  {
    // Posição inicial do centro do barco (metros) e orientação (radianos).
    x_ = declare_parameter<double>("initial_x", -7.0);
    y_ = declare_parameter<double>("initial_y", -6.0);
    yaw_ = declare_parameter<double>("initial_yaw", 0.0);

    // O mundo é um quadrado limitado por x,y = +/- world_half_size.
    // O barco é aproximado por um círculo, para verificar colisões.
    world_half_size_ = declare_parameter<double>("world_half_size", 12.0);
    boat_radius_ = declare_parameter<double>("robot_radius", 0.35);

    // Configuração do LiDAR virtual.
    num_beams_ = declare_parameter<int>("scan_beams", 360);
    min_range_ = declare_parameter<double>("range_min", 0.10);
    max_range_ = declare_parameter<double>("range_max", 20.0);
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);

    // Movimento automático, se ainda não tiver chegado um comando /cmd_vel.
    auto_motion_ = declare_parameter<bool>("auto_motion", true);
    forward_speed_ = declare_parameter<double>("forward_speed", 0.6);
    turn_speed_ = declare_parameter<double>("yaw_rate", 0.12);
    command_timeout_ = declare_parameter<double>("command_timeout", 1.0);

    // Cada grupo de quatro números representa um retângulo:
    // [xmin, ymin, xmax, ymax]. Estes valores também podem vir do YAML.
    const auto coordinates = declare_parameter<std::vector<double>>(
      "obstacles",
      {-2.0, -1.0, 0.0, 3.0,
       3.0, -5.0, 5.0, -2.0,
       4.0, 3.0, 7.0, 5.0,
       -7.0, 5.0, -4.0, 7.0});

    if (coordinates.size() % 4 != 0 || world_half_size_ <= 0.0 ||
        boat_radius_ <= 0.0 || num_beams_ < 2 || rate_hz_ <= 0.0 ||
        min_range_ <= 0.0 || max_range_ <= min_range_ || command_timeout_ <= 0.0) {
      throw std::runtime_error("Parametros do simulador invalidos");
    }

    for (std::size_t i = 0; i < coordinates.size(); i += 4) {
      Rectangle rect{coordinates[i], coordinates[i + 1],
                     coordinates[i + 2], coordinates[i + 3]};
      if (rect.xmin >= rect.xmax || rect.ymin >= rect.ymax) {
        throw std::runtime_error("Retangulo invalido: xmin < xmax e ymin < ymax");
      }
      obstacles_.push_back(rect);
    }

    if (has_collision(x_, y_)) {
      throw std::runtime_error("Posicao inicial do barco tem uma colisao");
    }
  }

  // -----------------------------------------------------------------------
  // 2. COMUNICAÇÃO ROS 2
  // -----------------------------------------------------------------------
  void setup_ros_communication()
  {
    // Publishers: enviam mensagens para outros nós.
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // Subscriber: recebe velocidades pedidas por um nó externo.
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&SimpleSimulator::on_velocity_command, this, std::placeholders::_1));

    // Publica a transformação entre o mapa e a posição/orientação do barco.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  void on_velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    received_command_ = true;
    last_command_ = now();
    commanded_speed_ = msg->linear.x;  // m/s
    commanded_turn_ = msg->angular.z;  // rad/s
  }

  // -----------------------------------------------------------------------
  // 3. MOVIMENTO DO BARCO E DETEÇÃO DE COLISÕES
  // -----------------------------------------------------------------------
  bool has_collision(double x, double y) const
  {
    // O centro do barco tem de manter uma margem igual ao seu raio.
    if (std::abs(x) + boat_radius_ >= world_half_size_ ||
        std::abs(y) + boat_radius_ >= world_half_size_) {
      return true;
    }

    for (const auto & obstacle : obstacles_) {
      if (distance_to_rectangle_squared(x, y, obstacle) <= boat_radius_ * boat_radius_) {
        return true;
      }
    }
    return false;
  }

  void update()
  {
    const auto timestamp = now();
    const double dt = std::clamp((timestamp - last_update_).seconds(), 0.0, 0.2);
    last_update_ = timestamp;

    // Velocidade automática, se estiver ligada e não tiver sido usado /cmd_vel.
    double speed = (auto_motion_ && !received_command_) ? forward_speed_ : 0.0;
    double turn = (auto_motion_ && !received_command_) ? turn_speed_ : 0.0;

    // Comandos externos têm prioridade. Se expirarem, o barco pára.
    if (received_command_ &&
        (timestamp - last_command_).seconds() <= command_timeout_) {
      speed = commanded_speed_;
      turn = commanded_turn_;
    }

    // Modelo cinemático simples (integração de Euler):
    // dx/dt = v*cos(yaw), dy/dt = v*sin(yaw), dyaw/dt = turn.
    const double next_x = x_ + speed * std::cos(yaw_) * dt;
    const double next_y = y_ + speed * std::sin(yaw_) * dt;
    const double next_yaw = std::atan2(
      std::sin(yaw_ + turn * dt), std::cos(yaw_ + turn * dt));

    if (!has_collision(next_x, next_y)) {
      x_ = next_x;
      y_ = next_y;
    } else {
      speed = 0.0; // Impede a translação, mas permite continuar a rodar.
    }
    yaw_ = next_yaw;

    publish_odometry(timestamp, speed, turn);
    publish_lidar(timestamp);
  }

  // -----------------------------------------------------------------------
  // 4. PUBLICAÇÃO DA POSIÇÃO DO BARCO
  // -----------------------------------------------------------------------
  void publish_odometry(const rclcpp::Time & timestamp, double speed, double turn)
  {
    // Em ROS 2, a orientação é representada por um quaternion.
    // Para uma rotação apenas em torno de Z: qz=sin(yaw/2), qw=cos(yaw/2).
    const double qz = std::sin(yaw_ / 2.0);
    const double qw = std::cos(yaw_ / 2.0);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = timestamp;
    odom.header.frame_id = "map";       // Referencial fixo do mundo
    odom.child_frame_id = "base_link";   // Referencial do barco
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = speed;
    odom.twist.twist.angular.z = turn;
    odom_pub_->publish(odom);

    // Publica a mesma pose como transformação TF (map -> base_link).
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  // -----------------------------------------------------------------------
  // 5. CÁLCULO DO LIDAR VIRTUAL E PUBLICAÇÃO DO /scan
  // -----------------------------------------------------------------------
  double measure_distance(double angle) const
  {
    // Ângulo global do raio = orientação atual do barco + ângulo do sensor.
    const double world_angle = yaw_ + angle;
    const double dx = std::cos(world_angle);
    const double dy = std::sin(world_angle);

    // Começamos por ver a distância até às paredes exteriores.
    const Rectangle walls{-world_half_size_, -world_half_size_,
                          world_half_size_, world_half_size_};
    double closest_hit = ray_rectangle_distance(x_, y_, dx, dy, walls);

    // Verificamos os obstáculos e ficamos com a menor distância positiva.
    for (const auto & obstacle : obstacles_) {
      closest_hit = std::min(
        closest_hit, ray_rectangle_distance(x_, y_, dx, dy, obstacle));
    }

    // Um objeto fora do alcance não produz um ponto ocupado no mapa.
    if (!std::isfinite(closest_hit) || closest_hit > max_range_) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(closest_hit, min_range_);
  }

  void publish_lidar(const rclcpp::Time & timestamp)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = timestamp;
    // O LiDAR foi colocado no centro do barco, sem deslocamento adicional.
    scan.header.frame_id = "base_link";

    scan.angle_min = static_cast<float>(-PI);
    scan.angle_increment = static_cast<float>(2.0 * PI / num_beams_);
    scan.angle_max = scan.angle_min + (num_beams_ - 1) * scan.angle_increment;
    scan.time_increment = 0.0f; // Todos os raios observam o mesmo instante.
    scan.scan_time = static_cast<float>(1.0 / rate_hz_);
    scan.range_min = static_cast<float>(min_range_);
    scan.range_max = static_cast<float>(max_range_);

    scan.ranges.resize(static_cast<std::size_t>(num_beams_));
    for (int i = 0; i < num_beams_; ++i) {
      const double angle = scan.angle_min + i * scan.angle_increment;
      scan.ranges[i] = static_cast<float>(measure_distance(angle));
    }
    scan_pub_->publish(scan);
  }

  // -----------------------------------------------------------------------
  // 6. ESTADO DO SIMULADOR: VARIÁVEIS QUE PERSISTEM ENTRE ATUALIZAÇÕES
  // -----------------------------------------------------------------------
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double world_half_size_{12.0}, boat_radius_{0.35};
  double min_range_{0.1}, max_range_{20.0}, rate_hz_{10.0};
  double forward_speed_{0.6}, turn_speed_{0.12}, command_timeout_{1.0};
  int num_beams_{360};
  bool auto_motion_{true};
  bool received_command_{false};
  double commanded_speed_{0.0}, commanded_turn_{0.0};
  std::vector<Rectangle> obstacles_;

  rclcpp::Time last_update_, last_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SimpleSimulator>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("simple_simulator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
