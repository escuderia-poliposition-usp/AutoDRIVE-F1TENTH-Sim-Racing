####################################################
#   AutoDRIVE Devkit (DEV) - sem copiar código
####################################################
FROM osrf/ros:humble-desktop

# Sistema
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      sudo wget gedit nano vim curl unzip net-tools \
      python3-pip \
      xvfb ffmpeg libgdal-dev libsm6 libxext6 \
      dbus-x11 \
 && rm -rf /var/lib/apt/lists/*

# Python deps do projeto (mantendo numpy<2)
# IMPORTANTE: fixe zope-event antigo para não forçar setuptools novo
RUN pip3 install --no-cache-dir --upgrade \
    "numpy<2.0" pillow opencv-contrib-python \
    eventlet==0.33.3 Flask==1.1.1 Flask-SocketIO==4.1.0 \
    python-socketio==4.2.0 python-engineio==3.13.0 \
    greenlet==1.1.0 gevent==21.12.0 gevent-websocket==0.10.1 \
    "zope-event<5.0" \
    Jinja2==3.0.3 itsdangerous==2.0.1 werkzeug==2.0.3 \
    transforms3d attrdict

# PINS FINAIS (depois de tudo) p/ editable / --symlink-install do ament_python
# Estes PINS ficam POR ÚLTIMO pra nada re-upgradar o setuptools depois
RUN pip3 install --no-cache-dir --upgrade \
    "pip<24" "setuptools==65.5.0" "wheel<0.42" "packaging<23"

# ROS 2
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-$ROS_DISTRO-tf-transformations \
      ros-$ROS_DISTRO-imu-tools \
      ros-$ROS_DISTRO-navigation2 \
      ros-$ROS_DISTRO-nav2-bringup \
      ros-$ROS_DISTRO-slam-toolbox \
      ros-$ROS_DISTRO-robot-localization \
      ros-$ROS_DISTRO-robot-state-publisher \
      ros-$ROS_DISTRO-joint-state-publisher \
      ros-$ROS_DISTRO-xacro \
      ros-$ROS_DISTRO-cv-bridge \
      python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# Workspace vazio; código via bind mount
WORKDIR /home/autodrive_devkit/ws
RUN mkdir -p src && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Fica vivo aguardando você entrar
CMD ["bash", "-lc", "tail -f /dev/null"]
