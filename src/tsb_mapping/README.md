# tsb_mapping

Package ROS 2 em C++ que constrói um **OccupancyGrid 2D** a partir de `/odom` e `/scan` publicados pelo simulador. O mapper **não recebe a lista de obstáculos**: utiliza apenas a pose conhecida e as distâncias medidas.

## Nó: `mapping_node`

| Tópico | Tipo | Direção | Função |
| --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | Subscreve | Posição `(x,y)` e orientação do barco no frame `map` |
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscreve | Distâncias dos raios no frame `base_link` |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Publica | Probabilidade de ocupação de cada célula |

O simulador envia `/odom` e `/scan` com **o mesmo timestamp**. O mapper emparelha as mensagens por esse valor, converte a orientação de quaternion para `yaw` e transforma cada ângulo do LiDAR para o mapa: `ângulo_global = yaw + ângulo_do_raio`.

Para cada raio, a classe [`GridCore`](include/tsb_mapping/grid_core.hpp) usa **Bresenham** para percorrer as células: as atravessadas recebem evidência de espaço livre, e a última recebe evidência de ocupação **apenas quando há uma deteção válida**. A evidência é acumulada em **log-odds** para suportar várias observações. Raios `+inf` indicam ausência de deteção: libertam o espaço até ao alcance máximo, mas não colocam um obstáculo artificial. O mapa começa desconhecido (`-1`); células observadas são publicadas entre `0` (livre) e `100` (ocupada).

## Compilar e executar

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tsb_simple_simulator tsb_mapping --symlink-install
source install/setup.bash
ros2 launch tsb_mapping mapping_system.launch.py
```

Noutro terminal:

```bash
source /ws/install/setup.bash
ros2 topic hz /map
ros2 topic echo /map --once --field info
```

Para iniciar **só o mapper** (o simulador deve estar em execução noutro terminal):

```bash
ros2 launch tsb_mapping mapping.launch.py
```

## Parâmetros

Ficheiro [`config/mapping.yaml`](config/mapping.yaml):

| Parâmetro | Valor | Significado |
| --- | --- | --- |
| `width`, `height` | `280`, `280` | Dimensões em células |
| `resolution` | `0.1` | Metros por célula |
| `origin_x`, `origin_y` | `-14`, `-14` | Origem do mapa em metros |
| `p_free` | `0.35` | Evidência de espaço livre |
| `p_occupied` | `0.75` | Evidência de ocupação |
| `log_odds_min`, `log_odds_max` | `-5`, `5` | Limites do acumulador |
| `publish_rate_hz` | `2.0` | Frequência de publicação de `/map` |
| `max_pending_scans` | `30` | Limite de mensagens em espera pela pose |

## Testes automáticos

Os seis grupos estão em [`tests/grid_core_test.cpp`](tests/grid_core_test.cpp). Para correr **sem ROS 2**:

```bash
cd /ws/src/tsb_mapping
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  tests/grid_core_test.cpp -o /tmp/grid_core_test
/tmp/grid_core_test
```

Também se podem correr pelo `colcon` depois da compilação:

```bash
cd /ws
colcon test --packages-select tsb_mapping --event-handlers console_direct+
colcon test-result --verbose
```

Resultado esperado: `PASSOU: 6 grupos de testes do mapping`. Os testes verificam espaço livre/ocupado, repetição de medições, `+inf`, limites do mapa, valores inválidos e geometria/Bresenham diagonal.

## Visualização / exemplo de mapa

Com a Bridge instalada e sem outra instância a correr:

```bash
ros2 launch tsb_mapping mapping_system.launch.py start_foxglove:=true
```

No Foxglove, ligar a `ws://localhost:10000` **quando** o Docker encaminha `10000:8765`; no painel 3D, selecionar `Fixed frame = map` e ativar `/map`.

![Mapa observado no Foxglove, com algumas zonas ainda desconhecidas](docs/foxglove_partial_map.png)

As regiões cinzentas são **desconhecidas** (ocultas atrás de obstáculos); não são necessariamente obstáculos triangulares. A reconstrução é parcial porque o barco não observa todas as faces.

## Ficheiros e limitações

- [`src/mapping_node.cpp`](src/mapping_node.cpp): comunicação ROS 2, emparelhamento de timestamps e publicação do mapa.
- [`include/tsb_mapping/grid_core.hpp`](include/tsb_mapping/grid_core.hpp): Bresenham, log-odds e representação da grelha.
- [`launch/mapping.launch.py`](launch/mapping.launch.py): só mapper.
- [`launch/mapping_system.launch.py`](launch/mapping_system.launch.py): simulador + mapper; Bridge opcional.
- Não é SLAM: a posição do barco é conhecida. O modelo assume mundo estático, LiDAR centrado no barco e timestamps coincidentes.
