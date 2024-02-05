# Cuvis_ROS

### A ROS2 package for the Cubert family of hyperspectral cameras

## Installation (Ubuntu)

## Prequisities


> :warning: **Experimental Code**: Make sure you know your way around an Ubuntu operating system before proceeding!

Please install [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) by building from source. ROS2 Foxy will work with Ubuntu 20, other future ROS2 versions are not yet supported.

By default ROS2 Foxy will use Python 3.8.10. The current Cuvis library requires Python >= 3.9.1. Install this version using a virtual environment or alternate Python distribution using these [instructions](https://linuxize.com/post/how-to-install-python-3-9-on-ubuntu-20-04/).

N.B., you will need to install both python3.9 and python3.9-dev to build the `cuvis.pyil` library.

Follow the standard ROS2 source install directions, but before building the `ros2_foxy` local repo clone, use the following modified instructions

```
# Create the venv
cd <<WORKING DIRECTORY>>
python3.9 -m venv venv_3.9
. venv_3.9/bin/activate

# Install missing packages
pip install empy==3.3.4 # There might be others...
# Run the build process
cd ros2_foxy
colcon build --symlink-install --packages-skip-by-dep python_qt_binding # Skips some incompatible GUIs
. ~/ros2_foxy/install/local_setup.bash # Sources local setup
```



### Installing CUVIS drivers

This ROS driver assumes the CUVIS C SDK has been installed on the local machine. Follow the instructions [here](https://cloud.cubert-gmbh.de/index.php/s/m1WfR66TjcGl96z) before proceeding.

### Extracting CUVIS Factory Files

Install the latest version of Wine: https://wiki.winehq.org/Ubuntu

Using the included flash drive with your camera. Navigate to the Cubert Utilities Installer, right click and select _Open with Wine Windows Program Loader_.

Accept license, and click through installer. Select the license file included on the USB.

### Configure Factory Directories

The `cuvis.ros` repo contains an empty folder named `cuvis_factory`. When prompted, select this folder for the factory installation. This place two files, `init.daq` and `SpRad.cu3` in the folder.

### Configure Camera Network

The `.exe` camera installer will not automatically change your network settings, as it executes PowerShell scripts in the background, which are not compatible with Linux.

However, the installer creates two temporary files: `setIP.ps1` and `setJumboFrames.ps1`.`

Each provides guidance on how to configure the camera network settings. Cubert cameras use different IP addresses based on the model. It is important to check these files for the IP address.

Before proceeding, make sure the camera is plugged in, and connected to your computer via an appropriate ethernet connection.

In Ubuntu, open *Settings* -> *Network* -> *Wired*. At this point the wired network should show its status as "connecting".

Click the IPv4 tab, and change the IPv4 method to "Manual". Referencing the values found in `setIP.ps1`, change the following values:

- DNS: e.g. 10.10.10.100
- Netmask: e.g. 255.255.255.0
- Gateway: e.g. 10.10.10.1
- Address: *This should be an address falling within the net masked address space, and not the same value as the gateway or DNS*, e.g. 10.10.10.10


Click apply wait for the connect to show as connected. If this step fails, try unplugging and replugging the camera.

#### Set Jumbo Packets

Hyperspectral data is big data, especially from video rate hyperspectral cameras. To enable better network performance, especially with high-megapixel Ultris models, we will enable a networking configuration named **jumbo frames**.

To begin, we will need the name of the network interface. In a new terminal, run `ip -c a`. Find the listed entry that has the *inet* matching the address from the previous step. The text following the number is the plaintext network name.

Open the file `./utils/jumbo.sh` and change the value for `interface_name` to the name of your network adapter. You may also change the size of the packed size, although 9000 appears to be sufficient Run the following commands:

```
sudo cp ./utils/jumbo.sh /etc/init.d/jumbo.sh
sudo chmod 775 /etc/init.d/jumbo.sh
sudo update-rc.d jumbo.sh defaults
```
Reboot your computer and run `ip link show | grep mtu` to confirm the mtu value is correctly set to 9000.

### Install Cuvis SDK
```
git clone git@github.com:cubert-hyperspectral/cuvis.pyil.git
cd cuvis.pyil
git submodule update --init --recursive
cmake .
cmake --build . --target cuvis_pyil --config Release # This step may take a considerable amount of time unless you turn off Doxygen generation
```
Move the built libraries to the set locations

```
python3.9 -m pip install .
cp _cuvis_pyil.so <<PYTHON_LIB_LOCATION>>/lib/python3.9/site-packages/cuvis_il/
cp cuvis_il.py <<PYTHON_LIB_LOCATION>>/lib/python3.9/site-packages/cuvis_il/
```
Install the Python bindings

```
git clone git@github.com:cubert-hyperspectral/cuvis.python.git
cd cuvis.python
python3.9 -m pip install .
```

### Building the ROS2 Nodes

`cd cuvis.ros && colcon build`

`source install/setup.bash` to add the new packages to the search path.

### Running the ROS Nodes

In the file `scripts/ros2_interface.py` update the shebang to match the installation location of your Python3.9 interpreter.

#### Standalone with Default Args

This step assumes there is already another ROS core instance running elsewhere.

`ros2 run cuvis_ros ros2_interface.py`

#### With Launch File

This is the file that should be edited to pass non-default arguments

`roslaunch cuvis_ros hyper_driver.launch`


### TODO

This section contains additional development goals which will be pursued as time allows.

- [ ] C++ driver implementation
- [ ] Integration with Hyper-Drive common HSI ROS library
- [ ] Add reflectance/radiance calibration measurement
- [ ] Handle loop interrupts with grace
- [X] ROS2 support