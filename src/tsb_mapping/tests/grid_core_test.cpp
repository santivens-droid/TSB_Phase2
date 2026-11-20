#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "tsb_mapping/grid_core.hpp"

int main()
{
  using tsb_mapping::GridCore;
  auto cell_value = [](GridCore & g, double x, double y) {
    int cx = 0, cy = 0;
    assert(g.world_to_cell(x, y, cx, cy));
    return static_cast<int>(g.occupancy_values()[static_cast<std::size_t>(cy) * g.width() + cx]);
  };

  // Uma leitura de 2 m a partir da origem: espaco livre ate ao obstaculo.
  GridCore grid(100, 100, 0.1, -5.0, -5.0, 0.35, 0.75, -5.0, 5.0);
  assert(cell_value(grid, 1.0, 0.0) == -1);
  assert(grid.integrate_beam(0.0, 0.0, 0.0, 2.0, 0.1, 10.0));
  assert(cell_value(grid, 1.0, 0.0) < 50);
  assert(cell_value(grid, 2.0, 0.0) > 50);
  assert(cell_value(grid, 3.0, 0.0) == -1);

  // Repetir a observacao aumenta a confianca, com clipping.
  const int first_occupied = cell_value(grid, 2.0, 0.0);
  for (int i = 0; i < 50; ++i) grid.integrate_beam(0.0, 0.0, 0.0, 2.0, 0.1, 10.0);
  assert(cell_value(grid, 2.0, 0.0) >= first_occupied);
  assert(cell_value(grid, 2.0, 0.0) <= 100);

  // +infinito: raio sem colisao, nunca marca o endpoint como ocupado.
  GridCore no_hit(100, 100, 0.1, -5.0, -5.0, 0.35, 0.75, -5.0, 5.0);
  assert(no_hit.integrate_beam(0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(), 0.1, 2.0));
  assert(cell_value(no_hit, 2.0, 0.0) < 50);

  // Feixe limitado pela borda do mapa: nao criar obstaculo artificial na borda.
  GridCore clipped(100, 100, 0.1, -5.0, -5.0, 0.35, 0.75, -5.0, 5.0);
  assert(clipped.integrate_beam(0.0, 0.0, 0.0, 10.0, 0.1, 20.0));
  assert(cell_value(clipped, 4.9, 0.0) < 50);

  // Leitura NaN e invalida e nao altera o mapa.
  GridCore invalid(100, 100, 0.1, -5.0, -5.0, 0.35, 0.75, -5.0, 5.0);
  assert(!invalid.integrate_beam(0.0, 0.0, 0.0,
    std::numeric_limits<double>::quiet_NaN(), 0.1, 10.0));
  assert(cell_value(invalid, 1.0, 0.0) == -1);

  // Geometria do exemplo real do simulador: barco (-7,-6), parede x=12.
  GridCore world(280, 280, 0.1, -14.0, -14.0, 0.35, 0.75, -5.0, 5.0);
  assert(world.integrate_beam(-7.0, -6.0, 0.0, 19.0, 0.1, 20.0));
  assert(cell_value(world, 11.0, -6.0) < 50);
  assert(cell_value(world, 12.0, -6.0) > 50);
  assert(!world.integrate_beam(-7.0, -6.0, 0.0,
    -std::numeric_limits<double>::infinity(), 0.1, 20.0));

  // Um feixe diagonal deve atravessar celulas de ambos os eixos.
  GridCore diagonal(100, 100, 0.1, -5.0, -5.0, 0.35, 0.75, -5.0, 5.0);
  assert(diagonal.integrate_beam(0.0, 0.0, std::atan2(1.0, 1.0),
                                 std::sqrt(2.0), 0.1, 5.0));
  assert(cell_value(diagonal, 0.5, 0.5) < 50);
  assert(cell_value(diagonal, 1.0, 1.0) > 50);

  std::cout << "OK: raios, ocupacao, espaco livre, alcance e limites da grelha\n";
}
