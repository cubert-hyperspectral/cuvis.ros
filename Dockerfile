FROM ubuntu:focal

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

ENV TZ=America/New_York
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y tzdata
RUN ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && dpkg-reconfigure -f noninteractive tzdata

# Get non-included deps for cubert
RUN apt-get update -y \
    && apt-get install libcrypto++6  -y\
    && apt-get install libgps26 -y \
    && apt-get install -y \
        libxerces-c3.2 \
        libspatialite7 \
        libopencv-calib3d4.2 \
        libopencv-imgproc4.2 \
        libopencv-features2d4.2 \
        libopencv-flann4.2 \
        libopencv-core4.2 \
        libopencv-imgcodecs4.2 \
        libtiffxx5 \
        libboost-locale-dev \
        git

# # # Install Python & essential python packages
RUN apt-get update -y
RUN apt install -y python3.9-full python3.9-dev
RUN apt install -y python3-pip ipython3
RUN python3.9 -m pip install --upgrade pip 

WORKDIR /install
COPY libs/* /install/cuvis_libs/
RUN cd /install/cuvis_libs && dpkg -i *.deb && cd /install 
RUN apt-get install git -y 

RUN cd /install \ 
    && apt install software-properties-common -y \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update -y \
    && apt install python3.9 -y \
    && apt-get install python3.9-venv -y \
    && python3.9 -m venv venv_3.9 \
    && . venv_3.9/bin/activate \
    && python3 -m pip install numpy 

# #RUN  apt install python3-all && apt-get update -y && apt-get install python3-pip -y && python3 -m pip install numpy
# # Install the cubert things with python bindings
RUN  git clone https://github.com/cubert-hyperspectral/cuvis.pyil.git \
    && cd cuvis.pyil \
    && git submodule update --init --recursive \
    && apt-get update -y \
    && apt-get install swig -y \
    # && rm -rf CMakeLists.txt \ 
    # && mv /install/cuvis_libs/CMakeLists.txt . \
    && cmake . -DCMAKE_BUILD_TYPE=Release -DDOXYGEN_BUILD_DOCUMENTATION=FALSE -DPython_ROOT_DIR="/install/venv_3.9" \
    && cmake --build . --target cuvis_pyil --config Release \ 
    && . /install/venv_3.9/bin/activate \
    && python3.9 -m pip install /install/cuvis.pyil \
    && cp /install/cuvis.pyil/_cuvis_pyil.so /install/venv_3.9/lib/python3.9/site-packages/cuvis_il/ \
    && cp /install/cuvis.pyil/cuvis_il.py /install/venv_3.9/lib/python3.9/site-packages/cuvis_il/ \
    && cd /install && git clone https://github.com/cubert-hyperspectral/cuvis.python.git \
    && python3.9 -m pip install /install/cuvis.python 

RUN . /install/venv_3.9/bin/activate && \
    apt install software-properties-common && \
    add-apt-repository universe && \
    apt update &&  apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt install -y \
    libbullet-dev \
    python3-pytest-cov \
    ros-dev-tools && \
    # install some pip packages needed for testing
    python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest && \ 
    # install Fast-RTPS dependencies
    apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev && \
    # install Cyclone DDS dependencies
    apt install --no-install-recommends -y \
    libcunit1-dev

WORKDIR /ros2_source_install/src
RUN . /install/venv_3.9/bin/activate && \
    pip install --upgrade --force-reinstall setuptools && \
    pip install -U poetry && \
    pip install empy==3.3.4  && \
    pip install catkin_pkg && \
    pip install lark-parser && \
    cd /ros2_source_install/ && vcs import --input https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos src && \
    apt upgrade -y && \
    rosdep init && \
    rosdep update

WORKDIR /ros2_source_install
RUN . /install/venv_3.9/bin/activate \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

WORKDIR /ros2_source_install
RUN . /install/venv_3.9/bin/activate && \
    colcon build --packages-skip-by-dep python_qt_binding --packages-ignore rviz_default_plugins --symlink-install

WORKDIR /colcon_ws/src
RUN apt-get install python3-catkin-tools -y \
    && . /install/venv_3.9/bin/activate \
    && pip install pyyaml rospkg \
    && git clone https://github.com/cubert-hyperspectral/cuvis.ros.git && cd cuvis.ros && git checkout ros2_docker \
    && . /ros2_source_install/install/setup.sh \
    && cd /colcon_ws && colcon build \
    && echo "source /ros2_source_install/install/setup.bash" >> ~/.bashrc \
    && echo "source /colcon_ws/install/local_setup.bash" >> ~/.bashrc \
    && echo "export CUVIS="Linux"" >> ~/.bashrc
WORKDIR /colcon_ws/