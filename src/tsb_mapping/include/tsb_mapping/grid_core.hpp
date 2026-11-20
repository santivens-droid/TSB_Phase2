#pragma once

// Algoritmos matematicos do occupancy grid, independentes do ROS 2.
// Assim podemos testar o mapping sem sensores ou processos ROS.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace tsb_mapping {

class GridCore {
public:
  GridCore(int width, int height, double resolution, double origin_x, double origin_y,
           double probability_free, double probability_occupied,
           double min_log_odds, double max_log_odds)
    : width_(width), height_(height), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y),
      min_log_odds_(min_log_odds), max_log_odds_(max_log_odds)
  {
    if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0 ||
        probability_free <= 0.0 || probability_free >= 0.5 ||
        probability_occupied <= 0.5 || probability_occupied >= 1.0 ||
        min_log_odds_ >= 0.0 || max_log_odds_ <= 0.0) {
      throw std::invalid_argument("Parametros de occupancy grid invalidos");
    }

    // Prior de ocupacao = 0.5 => log-odds inicial = 0.
    free_increment_ = std::log(probability_free / (1.0 - probability_free));
    occupied_increment_ = std::log(probability_occupied / (1.0 - probability_occupied));
    log_odds_.assign(static_cast<std::size_t>(width_) * height_, 0.0);
    observed_.assign(log_odds_.size(), false);
  }

  bool world_to_cell(double x, double y, int & cx, int & cy) const
  {
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return inside(cx, cy);
  }

  // Atualiza o mapa usando um raio do LiDAR (angulo em coordenadas globais).
  // Retorna false se a leitura for invalida ou se o barco estiver fora do mapa.
  bool integrate_beam(double sensor_x, double sensor_y, double global_angle,
                      double reading, double range_min, double range_max)
  {
    int sx = 0, sy = 0;
    if (!world_to_cell(sensor_x, sensor_y, sx, sy) ||
        !std::isfinite(global_angle) || range_min <= 0.0 || range_max <= range_min ||
        (std::isnan(reading) || (std::isinf(reading) && reading < 0.0)) ||
        (std::isfinite(reading) && reading < range_min)) {
      return false;
    }

    // Leitura finita dentro do alcance => houve obstaculo na ponta do raio.
    // +inf ou leitura superior ao alcance => apenas espaco livre ate range_max.
    const bool hit = std::isfinite(reading) && reading <= range_max;
    const double distance = hit ? reading : range_max;

    const double dx = distance * std::cos(global_angle);
    const double dy = distance * std::sin(global_angle);
    const double end_x = sensor_x + dx;
    const double end_y = sensor_y + dy;

    // Cortar o segmento na fronteira da grelha, preservando a direcao do raio.
    // A fronteira superior e exclusiva na conversao mundo -> celula.
    const double eps = resolution_ * 1e-6;
    const double max_x = origin_x_ + width_ * resolution_ - eps;
    const double max_y = origin_y_ + height_ * resolution_ - eps;
    const double min_x = origin_x_;
    const double min_y = origin_y_;
    double fraction = 1.0;

    if (dx > 0.0 && end_x > max_x) fraction = std::min(fraction, (max_x - sensor_x) / dx);
    if (dx < 0.0 && end_x < min_x) fraction = std::min(fraction, (min_x - sensor_x) / dx);
    if (dy > 0.0 && end_y > max_y) fraction = std::min(fraction, (max_y - sensor_y) / dy);
    if (dy < 0.0 && end_y < min_y) fraction = std::min(fraction, (min_y - sensor_y) / dy);
    fraction = std::clamp(fraction, 0.0, 1.0);

    int ex = 0, ey = 0;
    if (!world_to_cell(sensor_x + fraction * dx, sensor_y + fraction * dy, ex, ey)) {
      return false;
    }

    // Bresenham: percorre todas as celulas atraves das quais passa o raio.
    // Marca tudo como livre, exceto a ultima celula quando houve uma colisao
    // com um obstaculo verdadeiramente DENTRO do mapa.
    const bool hit_inside_map = hit && fraction >= 1.0 - 1e-12;
    const int step_x = (sx < ex) ? 1 : -1;
    const int step_y = (sy < ey) ? 1 : -1;
    const int delta_x = std::abs(ex - sx);
    const int delta_y = -std::abs(ey - sy);
    int error = delta_x + delta_y;
    int x = sx, y = sy;

    while (true) {
      const bool last = (x == ex && y == ey);
      add_evidence(x, y, last && hit_inside_map ? occupied_increment_ : free_increment_);
      if (last) break;
      const int twice_error = 2 * error;
      if (twice_error >= delta_y) { error += delta_y; x += step_x; }
      if (twice_error <= delta_x) { error += delta_x; y += step_y; }
    }
    return true;
  }

  // Converte as probabilidades para o formato ROS OccupancyGrid:
  // -1 = desconhecido; 0 = livre; 100 = ocupado.
  std::vector<int8_t> occupancy_values() const
  {
    std::vector<int8_t> values(log_odds_.size(), -1);
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (!observed_[i]) continue;
      const double probability = 1.0 / (1.0 + std::exp(-log_odds_[i]));
      values[i] = static_cast<int8_t>(std::lround(100.0 * probability));
    }
    return values;
  }

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }

private:
  bool inside(int x, int y) const { return x >= 0 && x < width_ && y >= 0 && y < height_; }

  void add_evidence(int x, int y, double increment)
  {
    const std::size_t index = static_cast<std::size_t>(y) * width_ + x;
    observed_[index] = true;
    log_odds_[index] = std::clamp(log_odds_[index] + increment, min_log_odds_, max_log_odds_);
  }

  int width_, height_;
  double resolution_, origin_x_, origin_y_;
  double free_increment_, occupied_increment_;
  double min_log_odds_, max_log_odds_;
  std::vector<double> log_odds_;
  std::vector<bool> observed_;
};

}  // namespace tsb_mapping
