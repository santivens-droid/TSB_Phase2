# Técnico Solar Boat — Phase 2 Mapping (Option B)

**ROS 2 Humble · C++17 · Simulador 2D + occupancy grid**

Este trabalho implementa a **Opção B (Simulator Track)**: um simulador publica a posição de um barco e medições LiDAR virtuais; um segundo nó usa esses dados para construir um mapa probabilístico. **Não utiliza HoloOcean nem SLAM.**

## 1. Quickstart

Executar dentro do Docker/DevContainer ROS 2 Humble do projeto original, na raiz do workspace (`/ws`):

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tsb_simple_simulator tsb_mapping --symlink-install
source install/setup.bash
ros2 launch tsb_mapping mapping_system.launch.py
```

**Noutro terminal do mesmo container** (sem parar o launch):

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic list
ros2 topic hz /map
ros2 topic echo /map --once --field info
```

**Resultado esperado:** tópicos `/odom`, `/scan`, `/tf` e `/map`. O mapa tem **280 × 280 células**, resolução **0,10 m** e origem **(-14, -14)**. O `/map` publica a cerca de **2 Hz**. Para terminar o sistema, usar `Ctrl + C` no terminal do launch.

## 2. Como funciona

```text
 tsb_simple_simulator
   ├── /odom (posição e orientação) ─┐
   └── /scan (360 raios LiDAR) ──────┤
                                     v
                               tsb_mapping
                           Bresenham + log-odds
                                     |
                                     v
                            /map (OccupancyGrid)
```

O simulador conhece os retângulos do mundo, mas **não entrega a sua localização ao mapper**. O mapper recebe apenas as medições LiDAR e a posição conhecida do barco.

| Package / nó | Entrada | Saída | Função |
| --- | --- | --- | --- |
| [`tsb_simple_simulator`](src/tsb_simple_simulator/README.md) / `simple_simulator` | `/cmd_vel` (opcional) | `/odom`, `/scan`, `/tf` | Movimento 2D, colisões e LiDAR virtual |
| [`tsb_mapping`](src/tsb_mapping/README.md) / `mapping_node` | `/odom`, `/scan` | `/map` | Integra raios e estima ocupação das células |

## 3. Outras formas de executar

**Só o simulador:**

```bash
ros2 launch tsb_simple_simulator simulator.launch.py
```

**Só o mapper** (requer que `/odom` e `/scan` estejam a ser publicados noutro terminal):

```bash
ros2 launch tsb_mapping mapping.launch.py
```

**Com visualização no Foxglove**, se a Bridge estiver instalada e ainda não estiver em execução:

```bash
ros2 launch tsb_mapping mapping_system.launch.py start_foxglove:=true
```

No Windows, abrir Foxglove e ligar a `ws://localhost:10000` **se** o Docker publicar `10000:8765`. No painel **3D**, escolher `Fixed frame = map`, mostrar `/map` e usar uma vista superior. A captura de um exemplo de **mapa parcialmente observado** está em [`src/tsb_mapping/docs/foxglove_partial_map.png`](src/tsb_mapping/docs/foxglove_partial_map.png).

## 4. Testes

O núcleo matemático do mapping tem **6 grupos de testes automáticos** (grelha livre/ocupada, repetição de medições, `+inf`, limites, valores inválidos e raios diagonais):

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tsb_mapping --symlink-install
colcon test --packages-select tsb_mapping --event-handlers console_direct+
colcon test-result --verbose
```

Alternativa rápida, independente do ROS 2 (compila apenas o núcleo matemático):

```bash
cd /ws/src/tsb_mapping
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude tests/grid_core_test.cpp -o /tmp/grid_core_test
/tmp/grid_core_test
```

Resultado esperado: `PASSOU: 6 grupos de testes do mapping`.

**Teste de integração com o sistema a correr:** `ros2 topic info /map`, `ros2 topic hz /map` e `ros2 topic echo /map --once --field info`. A confirmação de dimensões e frequência verifica se o mapa está a ser publicado; não substitui a validação visual.

## 5. Parâmetros, interfaces e ficheiros

- Simulador: [`src/tsb_simple_simulator/config/simulator.yaml`](src/tsb_simple_simulator/config/simulator.yaml). Mundo de 24 × 24 m, LiDAR de 360 raios, máximo 20 m, movimento automático ativado por omissão.
- Mapping: [`src/tsb_mapping/config/mapping.yaml`](src/tsb_mapping/config/mapping.yaml). Grelha 28 × 28 m, Bresenham e log-odds com `p_free=0.35` e `p_occupied=0.75`.
- Os detalhes de **cada nó, parâmetros e comandos** estão nos READMEs dos packages, ligados na tabela da secção 2.
- O código está documentado em `src/*/src/*.cpp` e `src/tsb_mapping/include/tsb_mapping/grid_core.hpp`.

## 6. Limitações

O movimento usa cinemática plana, sem hidrodinâmica. O LiDAR é ideal (sem ruído), a pose é conhecida e os obstáculos são estáticos. Regiões ocultas atrás de obstáculos continuam **desconhecidas** até serem observadas; podem surgir zonas cinzentas triangulares no mapa. Não foi implementado planeamento de trajetórias nem exploração manual.

## 7. Entrega em Git ou ZIP

Este ZIP contém **o README principal e os dois packages desenvolvidos**. Para submeter o **repositório original completo**, copiar os ficheiros para a raiz do workspace existente, preservando os restantes packages, o `.gitignore` e a configuração Docker, e só depois versionar.

```bash
cd /ws
git status
git add README.md src/tsb_simple_simulator src/tsb_mapping
git commit -m "Final version: 2D simulator and occupancy grid mapping"
git tag -a v1.0-final -m "Final submission"
```

Se estiver configurado um remoto Git, enviar o branch e a tag com `git push origin <nome-do-branch>` e `git push origin v1.0-final`. Caso contrário, entregar o **arquivo ZIP completo** do repositório. **A tag não é criada automaticamente por este ZIP.** Antes de submeter, atualizar os campos `maintainer` de ambos os `package.xml` e confirmar os testes no próprio container.
