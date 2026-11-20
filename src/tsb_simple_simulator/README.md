# tsb_simple_simulator

Package ROS 2 em C++ que simula um barco num mundo 2D com paredes e quatro obstáculos retangulares. **Não depende de HoloOcean.**

## Nó: `simple_simulator`

| Tópico | Tipo | Direção | Significado |
| --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | Publica | Pose `(x, y, yaw)` e velocidades; frame `map`, child `base_link` |
| `/scan` | `sensor_msgs/msg/LaserScan` | Publica | Distâncias do LiDAR virtual no frame `base_link` |
| `/tf` | `tf2_msgs/msg/TFMessage` | Publica | Transformação `map → base_link` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Subscreve | Velocidade linear `x` e angular `z` (opcional) |

O nó atualiza a posição com **cinemática 2D de Euler**: `x += v*cos(yaw)*dt`, `y += v*sin(yaw)*dt`, `yaw += w*dt`. As colisões bloqueiam a translação. Para o LiDAR, cada raio é intersectado com as paredes e os obstáculos e guarda-se a menor distância positiva. `+inf` significa que não foi detetado um obstáculo dentro do alcance máximo.

## Compilar e executar

No container ROS 2 Humble:

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tsb_simple_simulator --symlink-install
source install/setup.bash
ros2 launch tsb_simple_simulator simulator.launch.py
```

Noutro terminal do mesmo container:

```bash
source /ws/install/setup.bash
ros2 topic hz /scan
ros2 topic echo /odom --once --field pose.pose.position
```

## Configuração

Ficheiro [`config/simulator.yaml`](config/simulator.yaml) (utilizado pelo launch):

| Parâmetro | Valor por omissão | Para quê |
| --- | --- | --- |
| `initial_x`, `initial_y`, `initial_yaw` | `-7`, `-6`, `0` | Pose inicial |
| `world_half_size` | `12.0` | Paredes em `x,y = ±12` m |
| `robot_radius` | `0.35` | Raio usado para verificar colisões |
| `scan_beams` | `360` | Número de raios |
| `range_min`, `range_max` | `0.1`, `20.0` | Alcance LiDAR, em metros |
| `rate_hz` | `10.0` | Frequência de simulação |
| `auto_motion` | `true` | Movimento automático ao iniciar |
| `forward_speed`, `yaw_rate` | `0.6`, `0.12` | Velocidades automáticas |
| `obstacles` | 4 retângulos | Lista `[xmin, ymin, xmax, ymax, ...]` |

Para alterar o ambiente, editar o YAML e reiniciar o launch. Com `--symlink-install`, as alterações em YAML normalmente não exigem compilação; alterações em C++ exigem novo `colcon build`.

## Exemplos de teste

**1. Movimento:** com `auto_motion: true`, a posição em `/odom` deverá variar ao longo do tempo (exceto quando o barco estiver bloqueado por colisão).

**2. Geometria do LiDAR:** com o barco parado em `(-7, -6)` e orientação `0`, o raio frontal (índice 180) atinge a parede em `x=12`, a **19 m**. Se `initial_x=-6`, a distância esperada passa a **18 m**. Para iniciar imóvel sem editar o YAML:

```bash
ros2 run tsb_simple_simulator simulator_node --ros-args \
  --params-file /ws/src/tsb_simple_simulator/config/simulator.yaml \
  -p auto_motion:=false -p initial_x:=-7.0
```

Num segundo terminal, para imprimir apenas o raio frontal:

```bash
ros2 topic echo /scan --once --field ranges | python3 -c 'import sys; s=sys.stdin.read(); a=s[s.index("[")+1:s.rindex("]")].split(","); print("Raio frontal (m):", a[180].strip())'
```

**3. Mapping completo:** utilizar `ros2 launch tsb_mapping mapping_system.launch.py` (ver README principal). Não arrancar um segundo simulador se já estiver ativo.

## Ficheiros e limitações

- [`src/simulator_node.cpp`](src/simulator_node.cpp): cálculo do movimento, colisões, LiDAR e publicação ROS, com comentários.
- [`launch/simulator.launch.py`](launch/simulator.launch.py): arranca o nó com o YAML.
- [`config/simulator.yaml`](config/simulator.yaml): parâmetros editáveis.
- Modelo didático: sem física de fluidos, ruído de sensor ou estimativa da pose.
