# Cuvis_ROS

### A ROS Docker image for the Cubert family of hyperspectral cameras

## Installation

### Prereqs

You need docker working on your system, with a docker group. The following instructions are provided for an Ubuntu 20.04 host, although similar installation is possible in a Windows host.

sudo apt-get install docker.io
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
docker login```

In order to pull Docker images, you will need to register an account with Docker [here](https://hub.docker.com/signup).

**DO NOT INSTALL DOCKER DESKTOP. IT WILL CONFLICT WITH THE PROPER SETTINGS FOR THE CAMERA CONNECTION!**

### Preparing Docker

Clone this repo onto your computer's ros workspace, and change the branch to the ROS Noetic Docker image. This will build the Docker Image locally on your computer.

```
mkdir cubert && cd cubert
git clone git@github.com:RIVeR-Lab/cuvis.ros.git && cd cuvis.ros && git checkout ros_noetic_docker```
./setup_scripts/install.sh
./setup_scripts/build.sh
ip link set $ETH_INTERFACE_NAME mtu 9000 # Change the variable for $ETH_INTERFACE_NAME to the port for your ethernet interface
ip addr | grep mtu
```

### Preparing Network Connections
In order to connect to the camera we need to setup the network connection. The following scripts will configure your system IP address and set a larger package size for the network. Before running, open `setup_camera_connection.sh` and `jumbo.sh` and change the values to match the correct network configuration and network interface used by the camera (this is provided in your system documentation).

_Note: If you run the Cuvis `.exe` installer in Windows, this step is not necessary as the installer contains Powershell scripts which achieve the same configuration._
```
./setup_scripts/setup_camera_connection.sh # Update IP values
./setup_scripts/jumbo.sh # This is generally eth0
```
After configuring the network parameter, reboot the computer.

## Running the Container
Start the container by running the following command.
```
./setup_scripts/incant.sh 
```
In a separate terminal, run `docker container list` to find the container ID of the running container.

You will now need to move over your camera configuration files from your host into the container. At a minimum this should include an `init.daq` and `SpRad.cu3` file.
```
docker cp <<CUVIS_FACTORY_LOCATION>>. <<DOCKER CONTAINER ID>>:/catkin_ws/src/cuvis.ros/cuvis_factory
```
In the terminal for the docker container, the following commands will start the node to grab images:
```
source devel/setup.bash
export CUVIS="Linux"
roslaunch cuvis_ros driver.launch
```
## Retrieving Camera Data
