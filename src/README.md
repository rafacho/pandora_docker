# PANDORA ROBOT
**PAN**tographic
**D**evice
**O**n a
**R**obot for
**A**griculture

Robô com pernas de mecanismo de 4 barras (SEA - series elastic actuator) e rodas,
simulado em **ROS2 Lyrical Luth** com `ros2_control` + **Gazebo Sim** (gz-sim),
rodando em um container Docker.

## Instalação

Todo o ambiente (ROS2 Lyrical, ros2_control, gz-sim, ros_gz) vem pronto na imagem
Docker definida em `.devcontainer/Dockerfile` — não é necessário instalar ROS
manualmente no host.

### 1. Clonar o repositório

O pacote `ComplianceRos` é um git submodule (fork próprio, portado para ROS2) —
use `--recurse-submodules` ou ele fica vazio:

```bash
git clone --recurse-submodules git@github.com:rafacho/pandora_docker.git
cd pandora_docker
```

Se já clonou sem o submodule:
```bash
git submodule update --init --recursive
```

### 2. Subir o container

Duas formas equivalentes:

**a) VS Code Dev Containers** (recomendado para desenvolvimento): abra a pasta no
VS Code com a extensão "Dev Containers" instalada e escolha "Reopen in
Container" — usa `.devcontainer/devcontainer.json`/`Dockerfile`, monta o projeto
no path original (com espaços/acentos do host) e já roda `rosdep`/`colcon build`
automaticamente no `postCreateCommand`.

**b) docker-compose** (recomendado para rodar simulação/testar): monta `src/` em
um path limpo dentro do container, sem espaços/acentos:
```bash
docker compose up -d
docker exec -it pandora bash
```

> **Importante:** não compile o workspace com um `colcon build` rodado direto no
> host (fora do container) — se você tiver alguma outra instalação de ROS no
> sistema (ex: ROS Humble nativo), o build vai pegar o toolchain errado, e o
> path do projeto no host (com espaços/acentos) quebra as macros do
> `rosidl_generate_interfaces`. Compile sempre dentro do container.

### 3. Compilar o workspace

Dentro do container (via `docker exec` ou o terminal do devcontainer):
```bash
cd ~/ros_lyrical_docker_ws   # já é o WORKDIR padrão do container
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Estrutura do projeto

- `center_of_mass` — diretório reservado para um pacote de terceiros (cálculo de
  centro de massa/publicação no RViz); atualmente **vazio**, não portado/integrado.
- `ComplianceRos` — **git submodule**, fork próprio com o plugin de atuador
  elástico (SEA) portado para `ros2_control`:
  - `custom_controller` — plugin `controller_interface::ControllerInterface`
    (`ActuatorPositionController`, `ActuatorVelocityController`).
  - `custom_messages`, `custom_services` — mensagens/serviços customizados
    (`rosidl`).
- `pandora_control` — pacote `ament_python` com os nós rclpy do controle:
  - `pandora_control/` — código-fonte dos nós (módulo Python instalável).
  - `config/pandora_controllers.yaml` — **gerado automaticamente** por
    `generate_controllers_yaml.py` a cada `ros2 launch` (não editar à mão — veja
    a seção "Ajustar ganhos dos controladores" abaixo).
  - `launch/pandora_control.launch.py` — spawners dos controllers +
    `ik_server`.
- `pandora_description` — descrição do robô (URDF/xacro/meshes):
  - Toda a descrição é feita via `.xacro`. **Não edite os arquivos `.urdf`/`.sdf`
    diretamente** — são gerados a partir do `.xacro` em tempo de launch.
- `pandora_gazebo` — mundos (`worlds/`), plugins de sensores (`urdf/plugins/`) e
  configuração da ponte ROS2↔Gazebo Sim (`config/ros_gz_bridge.yaml`).
- `pandora_launch` — `launch/bringup.launch.py`: sobe o Gazebo Sim, publica o
  `robot_description`, spawna o robô, a ponte ROS2↔Gazebo e os controllers.
- `pandora_msgs` — mensagens/serviços customizados adicionais (cinemática
  inversa, contato das rodas).

## Uso

### Lançar a simulação completa

```bash
ros2 launch pandora_launch bringup.launch.py
```

Argumentos disponíveis (todos opcionais):

| Argumento | Padrão | Descrição |
|---|---|---|
| `gui` | `true` | `false` roda o Gazebo Sim headless (sem interface gráfica) |
| `paused` | `false` | `true` inicia a simulação pausada |
| `x`, `y`, `z`, `yaw` | `0`, `0`, `0.4`, `1.5708` | pose inicial de spawn do robô |

Exemplo, headless e pausado:
```bash
ros2 launch pandora_launch bringup.launch.py gui:=false paused:=true
```

Isso já sobe automaticamente `joint_state_broadcaster`, `position_controller`
(pernas) e `velocity_controller` (rodas, `diff_drive_controller`), além do
`ik_server`.

### Rodar nós individuais

Os demais nós de `pandora_control` não sobem automaticamente com o `bringup` —
rode manualmente quando precisar:

```bash
ros2 run pandora_control stability_set_point
ros2 run pandora_control static_stability
ros2 run pandora_control wheel_control
ros2 run pandora_control h_control
ros2 run pandora_control support_polygon
ros2 run pandora_control wheel_contacts
```

### Comandar posição de uma junta

```bash
ros2 topic pub -r 10 /position_controller/command custom_messages/msg/CustomCmnd \
  "{position: [-0.3, -0.3, -0.3, -0.3], velocity: [0,0,0,0], effort: [0,0,0,0], online_gain1: [0,0,0,0], online_gain2: [0,0,0,0]}"
```

Acompanhar:
```bash
ros2 topic echo /position_controller/state
```

### Ajustar ganhos dos controladores

`pandora_control/config/pandora_controllers.yaml` é **gerado automaticamente**
por `pandora_control/pandora_control/generate_controllers_yaml.py` toda vez que
`bringup.launch.py` roda (o parser de YAML do ROS2 não suporta âncoras/aliases,
então esse script é a fonte única de verdade para os ganhos compartilhados entre
as 4 juntas). Para mudar os ganhos permanentemente, edite os dicionários
`DEFAULT_GAINS`/`DEFAULT_SEA` (ou `GAIN_OVERRIDES`/`SEA_OVERRIDES` para uma junta
específica) nesse arquivo — **não edite `pandora_controllers.yaml` diretamente**,
suas mudanças seriam sobrescritas no próximo launch.

