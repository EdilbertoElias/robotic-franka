# aided_building

Projeto simplificado para simulação do braço Franka FR3 no Gazebo com controle via `ros2_control`.

## Visão geral
- Apenas o braço FR3 (sem garra/ferramenta)
- Comunicação ROS 2 simples via `aided_building_comm`
- Simulação no Gazebo Sim com `gz_ros2_control`
- Controle por trajetórias com `JointTrajectoryController` (`fr3_arm_controller`)
- Suporte a **SAFE_ZONE** e **WORK_ZONE** parametrizáveis
- Mão/PCB do operador simulados no Gazebo
- Movimento automático para **SAFE_ZONE** na inicialização

## Pacotes
- `aided_building_description`: URDF/xacro do FR3 para Gazebo + configuração do `ros2_control`, modelos/configurações de zonas
- `aided_building_bringup`: Launch da simulação (Gazebo, spawn do robô, controladores, RViz, mão do operador)
- `aided_building_comm`: Nó para envio de trajetórias, auto-movimento para SAFE_ZONE, serviços para ir a SAFE/WORK_ZONE

## Interfaces
| Interface | Tipo | Descrição |
| --- | --- | --- |
| `/franka/joint_targets` | `std_msgs/msg/Float64MultiArray` | Envia 7 posições de juntas (fr3_joint1..7) para o nó de comando |
| `/franka/go_safe_zone` | `std_srvs/srv/Trigger` | Move o robô para a posição SAFE_ZONE |
| `/franka/go_work_zone` | `std_srvs/srv/Trigger` | Move o robô para a posição WORK_ZONE |
| `/franka/home` | `std_srvs/srv/Trigger` | Alias para `/franka/go_safe_zone` (mantido por compatibilidade) |
| `/franka/stop` | `std_srvs/srv/Trigger` | Mantém a pose atual |
| `/joint_states` | `sensor_msgs/msg/JointState` | Estados das juntas publicados pelo `joint_state_broadcaster` |
| `/fr3_arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Trajetórias enviadas pelo `franka_comm` para o controlador |

## Configuração de Zonas

As zonas são configuradas por **parâmetros ROS** (passados pelo launch). Também há um arquivo de referência em `aided_building_description/config/zones.yaml` com perfis por cenário.

- **SAFE_ZONE**: Posição inicial/predefinida e segura (longe da mão do operador). O robô vai para essa posição automaticamente ao iniciar (configurável).
- **WORK_ZONE**: Posição de trabalho próxima à mão/PCB do operador, para onde o robô se move após estar na SAFE_ZONE.

## Como rodar (passo a passo)

### 1) Configurar ambiente
```bash
source /opt/ros/jazzy/setup.bash
cd /caminho/para/aided_building_ws  # ex: /work/aided_building_ws
source install/setup.bash
```

### 2) Compilar (se houver alterações)
```bash
colcon build --symlink-install
source install/setup.bash
```

### 3) Inicializar simulação
```bash
ros2 launch aided_building_bringup sim.launch.py
```

Customização opcional:
```bash
ros2 launch aided_building_bringup sim.launch.py \
  scenario:=scenario_1 \
  spawn_operator_hand:=true \
  hand_x:=0.4 hand_y:=0.0 hand_z:=0.25 \
  auto_move_to_safe_zone:=true startup_delay:=10.0 move_duration:=3.0 \
  safe_zone_joints:="[0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]" \
  work_zone_joints:="[0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.5]"
```

### 4) Verificar controladores
```bash
ros2 control list_controllers
```
Devem estar `active`: `joint_state_broadcaster` e `fr3_arm_controller`.

### 5) Controlar o robô
```bash
# Ir para SAFE_ZONE
ros2 service call /franka/go_safe_zone std_srvs/srv/Trigger {}

# Ir para WORK_ZONE
ros2 service call /franka/go_work_zone std_srvs/srv/Trigger {}

# Enviar posição customizada
ros2 topic pub --once /franka/joint_targets std_msgs/msg/Float64MultiArray "{data: [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.5]}"

# Parar/manter
ros2 service call /franka/stop std_srvs/srv/Trigger {}
```

## Simulando a mão do operador
A mão/PCB é spawnada via modelo SDF em `aided_building_description/models/operator_hand.sdf`. Ajuste com `hand_x,y,z,hand_roll,hand_pitch,hand_yaw` ou desative com `spawn_operator_hand:=false`. Para calibrar `work_zone_joints`, mova o robô até a posição desejada e leia `/joint_states`.

## Verificação/debug
```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 node list
ros2 service list | grep franka
```

## Build
```bash
colcon build --symlink-install --packages-select aided_building_description aided_building_bringup aided_building_comm
source install/setup.bash
```

## Observações
- Apenas 7 juntas (FR3), sem garra/ferramenta
- Apenas simulação no Gazebo
- Comande apenas após controladores estarem `active`
- Ambiente: ROS 2 Jazzy, Gazebo Sim, `ros_gz`, `ros2_control`, `gz_ros2_control`, `franka_description`
