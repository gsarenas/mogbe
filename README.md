**!!! EM CONSTRUÇÃO !!!**

# MOGBE

**MOGBE** - **Mo**bile **G**eneral Ro**b**ot for **E**ducation: um robô autônomo para aprendizagem de robótica móvel com ROS no ensino superior

<div style="display: flex;">
  <img src="img/mogbe_render_small.png" alt="mogbe_render_small" style="width: 30%;"/>
  <img src="img/demo_tcc_small.gif" alt="demo_tcc_small" style="width: 50%;"/>
</div>

## Conteúdos

- [Direcionamento](#direcionamento)
- [Configuração de OS: Dev Machine](#configuração-de-os-dev-machine)
- [Configuração de OS: Raspberry Pi](#configurando-de-os-raspberry-pi)
- [Flashing de firmware: Arduino Nano](#flashing-de-firmware-arduino-nano)
- [Configuração da área de trabalho](#configuração-da-área-de-trabalho)
- [Execução de simulação](#execução-de-simulação)
- [Execução do robô físico](#execução-do-robô-físico)
- [(Opcional) Configuração de câmera](#opcional-configuração-de-câmera)
- [Links úteis](#links-úteis)

## Direcionamento

- O MOGBE utiliza um computador portátil Raspberry Pi 3B+ em conjunto com Arduino Nano para controle efetivo da plataforma física. Um terceiro computador (dev machine) é utilizado para executar operações complexas de SLAM, Navegação Autônoma e Simulações. A seguir será detalhada a configuração inicial de cada plataforma. 

- Caso já tenha instalado o OS Ubuntu 22.04, ROS 2 Humble Hawksbill e configurado o Arduino Nano, siga para a seção de [Configuração da área de trabalho](#configuração-da-área-de-trabalho).

- Se pretende executar apenas simulações, siga os passos da seção de [Configuração de OS: Dev Machine](#configuração-de-os-dev-machine), depois [Configuração da área de trabalho](#configuração-da-área-de-trabalho) e siga para [Executando uma simulação](#executando-uma-simulação).

- Se possui a máquina virtual pré-configurada para aulas de Robótica com a imagem `Ubuntu 22.04.4 ROS2`, siga diretamente para a seção de [Execução de simulação](#execução-de-simulação) ou [Execução do robô real](#execução-do-robô-real).

## Configuração de OS: Dev Machine

- Certifique-se de que esteja rodando [Ubuntu Desktop versão Jammy Jellyfish 22.04.4 LTS](https://releases.ubuntu.com/jammy/).

- **Devido a limitações da ferramenta RViz, é necessário configurar o nome de usuário da dev mechine como `dev`. Caso não seja possível, será necessário fazer alterações nos arquivos `camera.xacro` e `lidar.xacro` da pasta `description` no pacote `mogbe`.**

- Instale o ROS 2 versão Humble Hawksbill seguindo o [tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) da documentação oficial. Prossiga até instalar `ros-humble-desktop` e `ros-dev-tools`, não é necessário instalar `ros-humble-base`.

- Para verificar a instalação, abra um terminal e rode o exemplo de `talker` em C++:

```bash
source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker
```

- Abra um novo terminal e rode o exemplo de `listener` em Python:

```bash
source /opt/ros/humble/setup.bash && ros2 run demo_nodes_py listener
```

- Nesse exemplo, espera-se ver o `talker` publicando suas mensagens e o `listener` reproduzindo o que foi publicado.

- Para evitar a necessidade de configurar as variáveis de ambiente do diretório de instalação de ROS com o comando `source` toda vez que for rodar um programa, edite o seu arquivo `~/.bashrc`:

```bash
sudo nano ~/.bashrc
```

- Adicione `source /opt/ros/humble/setup.bash`à última linha do arquivo, feche `Ctrl+X`, salve `Y` e confirme `Enter`.

## Configurando de OS: Raspberry Pi

- Certifique-se de que esteja rodando Ubuntu Server versão Jammy Jellyfish 22.04.4 LTS. O sistema do Raspberry Pi será rodado no modo headless, ou seja, sem vídeo. É recomendado utilizar o `Raspberry Pi Imager` para fazer a instalação do sistema operacional: 
  - Selecione o modelo do dispositivo.
  - Em sistema operacional, selecione `Other general-purpose OS` -> `Ubuntu` -> `Ubuntu Server 22.04.4 LTS (64-bit)`.
  - Adicione as suas configurações personalizadas e garanta que a opção de habilitar conexão SSH foi configurada e habilitada.
  - Utilize um cartão SD com pelo menos 16 GB de memória (32 GB é recomendado).

- Após gravar o sistema operacional e ligar o Raspberry Pi, inicialize uma sessão `ssh` para comandá-lo. 

- Antes de seguir para a instalação do ROS 2 no sistema, vamos fazer algumas configurações que facilitarão o processo de desenvolvimento. Primeiro, configure seu `needrestart.conf`:

```bash
sudo nano /etc/needrestart/needrestart.conf
```

- Localize a linha com `#$nrconf{restart} = 'i';` e troque por `$nrconf{restart} = 'a';`. Isso fará com que o sistema reinicie os serviços necessários após atualizações automaticamente sem solicitar nossa intervenção.

- Caso esteja utilizando um Raspberry Pi com 1 GB de memória RAM, é **extremamente** recomendado configurar uma memória swap para permitir que o sistema lide com tarefas mais intensivas. Para isso, vamos utilizar o próprio cartão SD (não é recomendado utilizar drives USB por não serem rápidos o suficiente). Crie um arquivo swap de 4 GB (escolha o tamanho de acordo com sua disponibilidade). Note que o caminho da pasta `pi` se dá em função do nome de usuário:

```bash
sudo fallocate -l 4G /home/pi/swapfile
```

- Dê permissões para o usuário `root`:

```bash
sudo chmod 600 /home/pi/swapfile
```

- Converta o arquivo em memória swap:

```bash
sudo mkswap /home/pi/swapfile
```

- Ative a memória:

```bash
sudo swapon /home/pi/swapfile
```

- Verifique se foi ativada:

```bash
sudo swapon --show
```

- Torne a configuração permanente editando o arquivo `fstab`:

```bash
sudo nano /etc/fstab
```

- Adicione `/home/pi/swapfile swap swap defaults 0 0` à última linha do arquivo. Agora o Raspberry Pi irá manter a configuração de memória swap mesmo após reiniciar o dispositivo.

- Reinicie o sistema:

```bash
sudo reboot
```

- Siga o processo de instalação de ROS 2 (Humble Hawksbill) do [tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) da documentação oficial. Prossiga até instalar `ros-humble-ros-base` e `ros-dev-tools`. Não é necessário instalar `ros-humble-desktop`.

- Para verificar a instalação, instale os pacotes de exemplo:

```bash
sudo apt install ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py
```

- Abra um novo terminal e rode o exemplo de `talker` em C++:

```bash
source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker
```

- Abra um novo terminal e rode o exemplo de `listener` em Python:

```bash
source /opt/ros/humble/setup.bash && ros2 run demo_nodes_py listener
```

- Nesse exemplo, espera-se ver o `talker` publicando suas mensagens e o `listener` reproduzindo o que foi publicado.

- Para evitar a necessidade de configurar as variáveis de ambiente do diretório de instalação de ROS toda vez que for rodar um programa, edite o seu `~/.bashrc`:

```bash
sudo nano ~/.bashrc
```

- Adicione `source /opt/ros/humble/setup.bash`à última linha do arquivo, feche `Ctrl+X`, salve `Y` e confirme `Enter`.

## Flashing de firmware: Arduino Nano

- Em seu ambiente de desenvolvimento de preferência, instale a IDE de Arduino do [site oficial](https://www.arduino.cc/en/software).

- Baixe o código-fonte do firmware para Arduino utilizado no MOGBE disponível em [`gsarenas/ros_arduino_bridge`](https://github.com/gsarenas/ros_arduino_bridge) e abra-o na IDE Arduino.

- Faça as alterações necessárias para combinar com seu setup. Atente-se às definições:
  - `BAUDRATE` em `ROSArduinoBridge.ino:linha 73`.
  - `LEFT_ENC_PIN_A` em `encoder_driver.h:linha 9`.
  - `LEFT_ENC_PIN_B` em `encoder_driver.h:linha 10`.
  - `RIGHT_ENC_PIN_A` em `encoder_driver.h:linha 13`.
  - `RIGHT_ENC_PIN_B` em `encoder_driver.h:linha 14`.
  - `RIGHT_MOTOR_BACKWARD` em `motor_driver.h:linha 6`.
  - `LEFT_MOTOR_BACKWARD` em `motor_driver.h:linha 7`.
  - `RIGHT_MOTOR_FORWARD` em `motor_driver.h:linha 8`.
  - `LEFT_MOTOR_FORWARD` em `motor_driver.h:linha 9`.
  
- Assumindo que você irá utilizar a mesma placa de desenvolvimento Arduino que o MOGBE, configure a IDE para compilar e gravar o código na placa:

  - `Ferramentas` -> `Placa` -> `Arduino AVR boards` -> `Arduino Nano`.
  - `Processador` -> `ATmega 328P (Old Bootloader)` caso esteja utilizando uma placa "paralela" e tenha dificuldades com a configuração padrão.
  - `Porta` -> `COMx` de acordo com sua porta conectada.

- Grave o código no Arduino Nano e teste o funcionamento da interface antes de integrá-la com ambiente ROS. Recomendo seguir o procedimento de [teste do firmware de Arduino para controle dos motores](#teste-do-firmware-de-arduino-para-controle-dos-motores).

### Teste do firmware de Arduino para controle dos motores

- No Raspberry Pi, supondo que o Arduino esteja conectado à porta ttyUSB0:

```bash
sudo pyserial-miniterm -e /dev/ttyUSB0 57600
```

- Nesse momento, é interessante testar o funcionamento do controle dos motores, bem como o funcionamento e sentido de giro dos encoders. Troque os pinos de sentido de giro dos motores e fases dos encoders se necessário. Alguns comandos importantes:
  - `e`: posição encoder de cada motor
  - `m`: velocidade dos motores (malha fechada) [enc_counts por loop]
  - `o`: velocidade dos motores (malha aberta)
  - `r`: reset valores de encoder
  - `u`: atualiza PID

## Configuração da área de trabalho

A área de trabalho (workspace) do projeto foi desenvolvida de modo que possa ser compartilhada entre dev machine e Raspberry Pi, ou seja, o mesmo procedimento de compilação pode ser seguido para ambos os casos. O código principal do projeto está localizado no repositório [`gsarenas/mogbe`](https://github.com/gsarenas/mogbe), mas também é feito uso de outros submódulos:

- [`gsarenas/ros_arduino_bridge`](https://github.com/gsarenas/ros_arduino_bridge): firmware de Arduino para controlar os motores através de uma interface de comunicação serial.
- [`gsarenas/serial`](https://github.com/gsarenas/serial): biblioteca para interface de porta serial RS-232
- [`gsarenas/diffdrive_arduino`](https://github.com/gsarenas/diffdrive_arduino): um `nó` que faz interface entre o `diff_drive_controller` de `ros_control` e o firmware de Arduino `ros_arduino_bridge`.
- [`gsarenas/ldlidar_stl_ros2`](https://github.com/gsarenas/ldlidar_stl_ros2): SDK dsenvolvido pela Shenzhen LDROBOT Co., LTD. para interface do sensor LiDAR LD19 com ROS 2.

Apesar desses pacotes serem referenciados como submódulos e o MOGBE depender deles para funcionar, a estrutura do projeto é organizada em múltiplos repositórios. O intuito dessa escolha é que, futuramente, estudantes de robótica móvel repliquem o projeto e identifiquem cada pacote como um recurso disponível. A partir disso, espera-se que derivem seus respectivos projetos de acordo com o que for necessário, não se prendendo a um repositório específico. Em projetos com ROS, é muito comum o desenvolvedor se basear em soluções já existentes e integrá-las para sua finalidade.

- Crie a área de trabalho ROS (lembre-se que o procedimento é o mesmo para o Raspberry Pi e a dev machine). **Caso precise usar um nome de pasta diferente de `mogbe_ws` será necessário fazer alterações nos arquivos `camera.xacro` e `lidar.xacro` da pasta `description` no pacote `mogbe`**:

```bash
mkdir -p ~/mogbe_ws/src && cd ~/mogbe_ws/
```

- Clone os repositórios:

```bash
git clone https://github.com/gsarenas/mogbe.git src/mogbe && \
git clone https://github.com/gsarenas/serial.git src/serial && \
git clone https://github.com/gsarenas/diffdrive_arduino.git src/diffdrive_arduino && \
git clone https://github.com/gsarenas/ldlidar_stl_ros2.git src/ldlidar_stl_ros2
```

- Inicie `rosdep` se ainda não tiver feito:

```bash
sudo rosdep init
```

- Instale as dependências:

```bash
architecture=$(uname -m) && \
rosdep update && rosdep install --from-paths src --ignore-src -y --os=ubuntu:$architecture
```

- `rosdep` deve ser suficiente para instalar os pacotes necessários. Entretanto, em caso de erro, as dependências da dev machine se resolvem com:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-gazebo-ros2-control \
ros-humble-slam-toolbox \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-twist-mux \
ros-humble-xacro \
ros-humble-image-transport-plugins \
ros-humble-rqt-image-view
```

- No caso do Raspberry Pi, ignore qualquer erro relacionado a `gazebo`. Caso tenha algum outro, as dependências do Raspberry Pi se resolvem com:

```bash
sudo apt install ros-humble-demo-nodes-cpp \
ros-humble-demo-nodes-py \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-slam-toolbox \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-twist-mux \
ros-humble-xacro \
ros-humble-image-transport-plugins \
ros-humble-v4l2-camera
```

- Compile o repositório:

```bash
colcon build --symlink-install
```

- Ignore o aviso de stderr da compilação da biblioteca `serial`. Trata-se de um aviso de descontinuidade, não falha na compilação.

- Com isso, a área de trabalho está configurada, compilada e pronta para execução.

## Execução de simulação

- Simulações são executadas na dev machine e independem do Raspberry Pi. Para executar um exemplo, garanta que esteja na pasta `mogbe_ws` e que as variáveis de ambiente estejam configuradas:

```bash
cd ~/mogbe_ws && source install/setup.bash
```

- Execute a simulação de teste:

```bash
ros2 launch mogbe launch_sim_all.launch.py world:=./src/mogbe/worlds/wall.world
```

- O ambiente de simulação Gazebo, a ferramenta de visualização RViz e demais `nós` devem inicializar. Para controle do robô na simulação, abra um novo terminal e rode o `nó` de comando `teleop`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

## Execução do robô físico

### Raspberry Pi

- Garanta que esteja na pasta `mogbe_ws` e que as variáveis de ambiente estejam configuradas:

```bash
cd ~/mogbe_ws && source install/setup.bash
```

- Execute os `nós` de `robot_state_publisher`, `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster` e `ldlidar_stl_ros2_node`:

```bash
ros2 launch mogbe launch_robot_pi_all.launch.py
```

### Dev Machine

- Após ter inicializado o MOGBE no Raspberry Pi, abra um novo terminal, garanta que esteja na pasta de trabalho `mogbe_ws` e que as variáveis de ambiente estejam configuradas:

```bash
cd ~/mogbe_ws && source install/setup.bash
```

- Execute os demais `nós`:

```bash
ros2 launch mogbe launch_robot_dev.launch.py
```

- Para controle manual do MOGBE, é necessário abrir uma nova aba de terminal e executar o `nó` de comando `teleop`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

## (Opcional) Configuração de câmera

### Raspberry Pi

- Para configurar uma câmera, supondo que será usada um módulo de câmera Raspberry Pi, instale os drivers:

```bash
sudo apt install libraspberrypi-bin v4l-utils
```

- Abra as configurações `raspi-config`:

```bash
sudo raspi-config
```

- `Advanced Options` -> `Camera` -> `Yes` para habilitar a interface da câmera.

- `Advanced Options` -> `SPI` -> `Yes` para habilitar a interface SPI.

- `Advanced Options` -> `I2C` -> `Yes` para habilitar a interface I2C.

- Edite suas configurações de boot:

```bash
sudo nano /boot/firmware/config.txt
```

- Encontre a linha que possui `camera_auto_detect=1` e mude para `camera_auto_detect=0`. Na linha abaixo de `display_auto_detect=1`, adicione a linha `start_x=1`.

- Reinicie:

```bash
sudo reboot
```

- Execute o `nó` driver da câmera:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[240,160]" -p camera_frame_id:=camera_optical_link -p brightness:="60"
```

### Dev Machine

- Com o `nó` driver da câmera rodando no Raspberry Pi, visualize a imagem com:

 
 ```bash
ros2 run rqt_image_view rqt_image_view 
```

## Links úteis

- [Documentação ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- [Documentação ROS 2 Control](https://control.ros.org/humble/doc/getting_started/getting_started.html)
- [Documentação Gazebo ROS 2 Control](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)
- [Criando um pacote ROS 2](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#prerequisites)
- [Convenções para nomear um pacote](https://ros.org/reps/rep-0144.html)
- [Escrevendo `nós` como `publisher` e `subscriber`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Publicando em `topics` via CLI](https://control.ros.org/humble/doc/ros2_control_demos/example_3/doc/userdoc.html)
- [Desenvolvendo uma interface de hardware](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html)