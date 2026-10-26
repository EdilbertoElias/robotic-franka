# aided_building_hand_perception

Detecção das mãos por câmera (MediaPipe Hand Landmarker), com a posição expressa
no frame da base do robô (`fr3_link0`).

A câmera do notebook é monocular (sem profundidade). Cada ponto da mão vira um
raio, e esse raio é cruzado com um plano paralelo à bancada
(`z = plane_height` no frame `workbench`). A posição fica correta enquanto as
mãos trabalham sobre a mesa, que é o caso da solda.

```text
fr3_link0 ──(workbench.yaml: PROVISÓRIO)──► workbench ──(calibração ArUco)──► camera_optical_frame
```

## Dependências

```bash
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-camera-calibration
pip install --user --break-system-packages --no-deps mediapipe absl-py flatbuffers sounddevice
wget -O ~/models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

Use `--no-deps`, porque sem ele o pip instala `numpy` 2.x e `opencv-contrib-python`
em `~/.local`, e isso quebra o `cv_bridge` do ROS (`KeyError: 16`).

## Uso

```bash
colcon build --symlink-install --packages-select aided_building_hand_perception
source install/setup.bash

# 1. Calibração intrínseca da câmera (uma vez por câmera)
#    resultado: ~/.ros/camera_info/notebook_cam.yaml

# 2. Imprimir o marcador ArUco (100 mm, em 100%, sem ajustar à página)
ros2 run aided_building_hand_perception generate_marker -- --output marcador.svg

# 3. Calibração câmera -> bancada (com a câmera já na posição final e o marcador na mesa)
ros2 run v4l2_camera v4l2_camera_node --ros-args --params-file \
  $(ros2 pkg prefix aided_building_hand_perception)/share/aided_building_hand_perception/config/webcam.yaml
ros2 run aided_building_hand_perception workbench_calibration   # outro terminal
#    resultado: ~/.ros/aided_building/workbench_camera.yaml

# 4. Detecção
ros2 launch aided_building_hand_perception hand_perception.launch.py
#    com a simulação do grupo: use_sim_time:=true
#    com o driver da câmera já rodando: start_camera:=false
```

Sem o arquivo do passo 3, o launch usa uma pose aproximada da câmera
(`fallback_camera` em `config/workbench.yaml`). Nesse caso as posições servem
só para testar o fluxo.

**Quando o robô estiver disponível:** meça onde o centro do marcador fica em
relação à base do robô e atualize `robot_base_to_workbench` em
`config/workbench.yaml`.

## Tópicos publicados

### Zona da mão (para análise de contato)

Vista de cima, cada mão ocupa uma **zona**: o menor retângulo orientado em
volta dos 21 pontos da mão projetados na mesa, ampliado em `zone_margin`
(padrão 3 cm) de cada lado. A zona se estende da mesa (z = 0 no `workbench`)
até `zone_height` (padrão 10 cm). A margem cobre a pele além das articulações
detectadas e o erro de rastreamento.

| Tópico | Tipo | Conteúdo |
| --- | --- | --- |
| `/hands/zones` | `vision_msgs/Detection3DArray` | uma caixa orientada por mão: `bbox.center` (pose) + `bbox.size`, `results[0].hypothesis.class_id` = `left`/`right`, `score` = confiança |
| `/hands/{left,right}/zone` | `geometry_msgs/PolygonStamped` | os 4 cantos da pegada da zona (vista de cima), no nível da mesa |
| `/hands/tool_distance` | `std_msgs/Float32` | menor distância entre a origem de `tool_frame` e as zonas (0 = ferramenta dentro da zona) |
| `/hands/min_distance` | `std_msgs/Float32` | menor distância entre as origens dos links `fr3_link0..8` e as zonas |
| `/hands/top_view` | `sensor_msgs/Image` | vista de cima: zonas, robô (com alcance de 855 mm), ferramenta e câmera |

`bbox` usa o mesmo formato (pose + tamanho) de um `SolidPrimitive` BOX do
MoveIt, então dá para inserir a zona diretamente como objeto de colisão.

### Posição e movimento

| Tópico | Tipo | Conteúdo |
| --- | --- | --- |
| `/hands/poses` | `geometry_msgs/PoseArray` | centro da palma de cada mão (vazio sem mãos) |
| `/hands/{left,right}/palm` | `geometry_msgs/PointStamped` | palma filtrada (One Euro), por lado da pessoa |
| `/hands/{left,right}/velocity` | `geometry_msgs/Vector3Stamped` | velocidade da palma (m/s) |
| `/hands/markers` | `visualization_msgs/MarkerArray` | esqueleto, caixa da zona e rótulo para o RViz |
| `/hands/debug_image` | `sensor_msgs/Image` | imagem da câmera com esqueleto e caixa da zona |

As imagens `/hands/debug_image` e `/hands/top_view` só são geradas quando há
assinante. Para vê-las:
`ros2 run rqt_image_view rqt_image_view /hands/top_view`.

Todos usam o `header.stamp` da imagem e o frame `fr3_link0`. Com
`use_sim_time:=true`, o carimbo vem do relógio do nó, porque a câmera usa
relógio real e o Gazebo usa relógio simulado.

Esses tipos são provisórios, até o grupo definir `aided_building_interfaces/HumanState`.

## Problemas conhecidos

- **Use `v4l2_camera`, não `usb_cam` 0.8.1.** O `usb_cam` carimba as imagens com
  0 a 1 s de atraso, e esse erro ainda cresce cerca de 1 ms/s (bug em `utils.hpp`
  na conversão de `tv_usec`).
- **Assine imagens com QoS RELIABLE.** Com BEST_EFFORT, as imagens de 900 kB
  chegavam a 9–19 Hz em vez de 30 Hz.
- **VS Code instalado via Snap:** janelas (calibrador, RViz) quebram com
  `symbol lookup error ... libpthread`. Rode `unset GTK_PATH GTK_EXE_PREFIX
  GIO_MODULE_DIR GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GTK_IM_MODULE_FILE
  LOCPATH GSETTINGS_SCHEMA_DIR` antes.
- **Rede compartilhada:** com `ROS_DOMAIN_ID` 0 e descoberta `SUBNET`, o TF de
  outras simulações da rede se mistura ao seu. Use
  `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` ou um `ROS_DOMAIN_ID` próprio.
- O rótulo Left/Right do MediaPipe supõe imagem espelhada (selfie). O nó
  inverte os rótulos (`image_is_mirrored: false`). Confira com as suas mãos.
- O marcador precisa de uma margem branca de pelo menos uma célula. O SVG gerado
  já tem duas, então não recorte rente ao preto.

## Testes

```bash
colcon test --packages-select aided_building_hand_perception
colcon test-result --verbose
```
