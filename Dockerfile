FROM osrf/ros:foxy-desktop

# Update CMake for Cuvis Installation
RUN apt-get update \
    && apt-get -y install build-essential \
    && apt-get install -y wget \
    && rm -rf /var/lib/apt/lists/* \
    && wget https://github.com/Kitware/CMake/releases/download/v3.25.1/cmake-3.25.1-Linux-x86_64.sh \
    -q -O /tmp/cmake-install.sh \
    && chmod u+x /tmp/cmake-install.sh \
    && mkdir /opt/cmake-3.25.1 \
    && /tmp/cmake-install.sh --skip-license --prefix=/opt/cmake-3.25.1 \
    && rm /tmp/cmake-install.sh \
    && ln -s /opt/cmake-3.25.1/bin/* /usr/local/bin

# Get non-included deps for Cuvis
RUN apt-get update -y \
    && apt-get install libcrypto++6  \
    && apt-get install libgps26

#Download the Cuvis SDK
WORKDIR /install
COPY libs/* /install/cuvis_libs/
RUN cd /install/cuvis_libs && dpkg -i *.deb && cd /install 

# Install the cubert things with python bindings
RUN git clone https://github.com/cubert-hyperspectral/cuvis.pyil.git \
    && cd cuvis.pyil \
    && git submodule update --init --recursive \
    && sudo apt-get install swig -y \
    && cmake . \
    && cmake --build . --target cuvis_pyil --config Release \ 
    && cd /install \ 
    && apt install software-properties-common -y \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update -y \
    && apt install python3.9 -y \
    && apt-get install python3.9-venv -y \
    && python3.9 -m venv venv_3.9 \
    && . venv_3.9/bin/activate \
    &&  python3.9 -m pip install /install/cuvis.pyil \
    && cp /install/cuvis.pyil/_cuvis_pyil.so /install/venv_3.9/lib/python3.9/site-packages/cuvis_il/ \
    && cp /install/cuvis.pyil/cuvis_il.py /install/venv_3.9/lib/python3.9/site-packages/cuvis_il/ \
    && cd /install && git clone https://github.com/cubert-hyperspectral/cuvis.python.git \
    && python3.9 -m pip install /install/cuvis.python 

 

# ROS2 Install
RUN mkdir -p /colcon_ws/src && . /opt/ros/foxy/setup.sh \
  && cd /colcon_ws/src \
  && git clone https://github.com/cubert-hyperspectral/cuvis.ros.git \
  && cd /colcon_ws/src/cuvis.ros && git checkout ros2 \

# Copy local files over to new locations
# COPY /home/nathaniel/cubert/ros2_ws/src/cuvis_ros/cuvis_factory/* /colcon_ws/src/cuvis.ros/cuvis_factory/

# # Build workspace
# RUN cd /colcon_ws && colcon build \
#   && . install/local_setup.bash
