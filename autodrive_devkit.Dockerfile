####################################################
# AutoDRIVE Devkit (ROS 2 Humble)
# - Gazebo classic (gazebo_ros)
# - Nav2 (navigation2 + nav2_bringup) + slam_toolbox + robot_localization
# - Pin de pip/setuptools/wheel para evitar problemas de colcon
# - Exporta GAZEBO_* para worlds/models/plugins do f1tenth_dynsim
####################################################

FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_ROOT_USER_ACTION=ignore

# ------------------------------------------------------------------
# OSRF repo para Gazebo Classic
# ------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release && \
    install -d -m 0755 /etc/apt/keyrings && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      | gpg --dearmor -o /etc/apt/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# ------------------------------------------------------------------
# Sistema + ROS + Gazebo Classic + integração gazebo_ros
# ------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # utilitários
    sudo wget curl unzip net-tools nano vim gedit python3-pip \
    # build
    build-essential cmake git python3-colcon-common-extensions \
    # ROS usuais
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-localization \
    # SLAM/Navigation
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    # Gazebo classic + plugins ROS
    gazebo \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    # GUI/X11
    xvfb ffmpeg libgdal-dev libsm6 libxext6 dbus-x11 \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------
# Pinar pip/setuptools/wheel (evita que o colcon quebre com versões novas)
# ------------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir --upgrade \
      "pip==23.2.1" "setuptools==65.5.1" "wheel==0.41.2"
RUN printf "pip==23.2.1\nsetuptools==65.5.1\nwheel==0.41.2\n" > /tmp/pip-constraints.txt
ENV PIP_CONSTRAINT=/tmp/pip-constraints.txt

# ------------------------------------------------------------------
# Python deps do projeto
# ------------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir --ignore-installed \
    transforms3d==0.4.2 \
    attrdict==2.0.1 \
    numpy==1.26.4 \
    pillow==9.5.0 \
    opencv-contrib-python==4.9.0.80 \
    eventlet==0.33.3 \
    Flask==1.1.1 \
    Flask-SocketIO==4.1.0 \
    python-socketio==4.2.0 \
    python-engineio==3.13.0 \
    greenlet==1.1.0 \
    gevent==21.12.0 \
    gevent-websocket==0.10.1 \
    Jinja2==3.0.3 \
    itsdangerous==2.0.1 \
    werkzeug==2.0.3

# ------------------------------------------------------------------
# Workspace + código
# ------------------------------------------------------------------
WORKDIR /home/autodrive_devkit
COPY autodrive_devkit/. /home/autodrive_devkit/src/autodrive_devkit

# rosdep
RUN apt-get update && apt-get install -y --no-install-recommends python3-rosdep && rm -rf /var/lib/apt/lists/* \
 && (rosdep init || true) \
 && rosdep update \
 && rosdep install --rosdistro ${ROS_DISTRO} --from-paths src --ignore-src -r -y || echo "rosdep: algumas chaves podem ser opcionais; seguindo."

# Build colcon
ENV COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# ------------------------------------------------------------------
# GAZEBO_* paths (após o build do f1tenth_dynsim)
# ------------------------------------------------------------------
ENV F1TENTH_DYNSIM_PREFIX=/home/autodrive_devkit/install/f1tenth_dynsim
ENV GAZEBO_RESOURCE_PATH=${F1TENTH_DYNSIM_PREFIX}/share/f1tenth_dynsim:${GAZEBO_RESOURCE_PATH}
ENV GAZEBO_MODEL_PATH=${F1TENTH_DYNSIM_PREFIX}/share/f1tenth_dynsim/models:${GAZEBO_MODEL_PATH}
ENV GAZEBO_PLUGIN_PATH=${F1TENTH_DYNSIM_PREFIX}/lib:${GAZEBO_PLUGIN_PATH}

# ------------------------------------------------------------------
# Ambiente ao entrar + Gazebo setup (corrige shaders/OGRE)
# ------------------------------------------------------------------
ENV QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo 'source /usr/share/gazebo/setup.sh' >> /root/.bashrc \
 && echo 'source /home/autodrive_devkit/install/setup.bash' >> /root/.bashrc \
 && echo 'export F1TENTH_DYNSIM_PREFIX=/home/autodrive_devkit/install/f1tenth_dynsim' >> /root/.bashrc \
 && echo 'export GAZEBO_RESOURCE_PATH=$F1TENTH_DYNSIM_PREFIX/share/f1tenth_dynsim:$GAZEBO_RESOURCE_PATH' >> /root/.bashrc \
 && echo 'export GAZEBO_MODEL_PATH=$F1TENTH_DYNSIM_PREFIX/share/f1tenth_dynsim/models:$GAZEBO_MODEL_PATH' >> /root/.bashrc \
 && echo 'export GAZEBO_PLUGIN_PATH=$F1TENTH_DYNSIM_PREFIX/lib:$GAZEBO_PLUGIN_PATH' >> /root/.bashrc

# Porta (se usar Flask do projeto)
EXPOSE 4567

# Entrypoint
COPY autodrive_devkit.sh /home/autodrive_devkit.sh
RUN chmod +x /home/autodrive_devkit.sh
ENTRYPOINT ["/home/autodrive_devkit.sh"]
