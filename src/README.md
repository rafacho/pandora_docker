# PANDORA ROBOT
**PAN**tographic
**D**evice
**O**n a 
**R**obot for
**A**griculture

## Instalação:
Instale o Ubuntu 20.04
Instale o Ros Noetic full

Abra um novo terminal e digite:
```
    $ source /opt/ros/noetic/setup.bash
    $ mkdir -p ~/Pandora_ws/src    
    $ git clone https://gitlab.com/rafacho.hugo/pandora
    $ cd ~/Pandora_ws/
    $ catkin_make
```

## Instalação via Robostack:
1. Baixe e instale o miniforge: https://github.com/conda-forge/miniforge
2. Instale o mamba: `conda install mamba -c conda-forge`
3. Crie um env:
    ´´´
        mamba create -n pandora
        mamba activate pandora
    ```
4. Configure os repositórios:
    ```
        # this adds the conda-forge channel to the new created environment configuration 
        conda config --env --add channels conda-forge
        # and the robostack channel
        conda config --env --add channels robostack-staging
        # remove the defaults channel just in case, this might return an error if it is not in the list which is ok
        conda config --env --remove channels defaults
    ```
5. Instale o ROS Noetic: `mamba install ros-noetic-desktop`
6. Reinicialize o environment:
    ```
        mamba deactivate
        mamba activate pandora
    ```
7. Instale o Gazebo e pacotes: `
    ```
        conda install conda-forge::gazebo
        conda install robostack-staging::ros-noetic-gazebo-ros 
    ```




Os arquivos estão organizados da seguinte forma:
- center_of_mass -> pacote de terceiros que calcula o centro de massa e publica no Rviz
- ComplianceRos -> Plugin de terceiros que implementa o atuador elástico no ros_control
- pandora_control -> pacote que contém todos os arquivos relacionados ao controle do robô

    -- config -> Contém os arquivos .yaml de configuração dos controladores
    -- scripts -> Contém os arquivos python
    -- src -> Contém os arquivos cpp
- pandora_description -> pacote que contém a descrição do robô

    -- launch -> Contém o arquivo que lança o robô no Rviz e as configurações do Rviz
    -- meshes -> Contém os arquivos para a visualização 3D do robô
    -- urdf -> Contém a descrição do robô

        Toda a descrição do robô é feita por meio dos arquivos .xacro. NÃO EDITAR OS ARQUIVOS .urdf e .sdf pois eles são gerados automaticamente.
        
- pandora_gazebo -> Contém os arquivos de configuração do gazebo e plugins dos sensores
- pandora_launch -> Contém os arquivos de inicialização
- pandora_msgs -> Contém as configurações das mensagens e serviços customizados

Para lançar o gazebo com o robô, utilize o comando: `roslaunch pandora_launch bringup.launch`

Para ligar o controlador da suspensão, utilize o comando: `roslaunch pandora_control pandora_control`

Para ligar o set point de estabilidade, utilize o comando: `rosrun pandora_control stability_set_point.py`

Para ligar o controle de estabilidade, utilize o comando: `rosrun pandora_control static_stability.py`

Para ligar controlador das rodas, utilize o comando: `rosrun pandora_control wheel_control.py`



 