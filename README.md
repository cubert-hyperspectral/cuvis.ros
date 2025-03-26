# Cuvis_ROS

### A ROS2 package for the Cubert family of hyperspectral cameras

## Installation (Ubuntu)

## Prequisities


> :warning: **Experimental Code**: Make sure you know your way around an Ubuntu operating system before proceeding!

> :warning: **Updates Inbound**: This repo is experimental code!

Please install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) via APT

By default ROS2 Humble will use Python 3.10. The current Cuvis library requires Python >= 3.9.1, so this will work by default.

### Installing CUVIS drivers

This ROS driver assumes the CUVIS C SDK has been installed on the local machine. Follow the instructions [here](https://cloud.cubert-gmbh.de/s/qpxkyWkycrmBK9m) before proceeding.

### Extracting CUVIS Factory Files

Install the latest version of Wine: https://wiki.winehq.org/Ubuntu

Using the included flash drive with your camera. Navigate to the Cubert Utilities Installer, right click and select _Open with Wine Windows Program Loader_.

Accept license, and click through installer. Select the license file included on the USB.

*Alternatively, install the program on a Windows machine and copy the files to the Linux (ROS) system.*

### Configure Factory Directories

The `cuvis.ros` repo contains an empty folder named `cuvis_factory`. When prompted, select this folder for the factory installation. This place two files, `init.daq` and `SpRad.cu3` in the folder.

The Wine installation will also generate two `.settings` files specific to your camera installation. They should be placed in the same folder as the `init.daq` and `spRad.cu3`. Make sure to update the names of these files in the install step of `CMakeLists.txt`.

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
Download the **deb** files and install from [here](https://cloud.cubert-gmbh.de/s/qpxkyWkycrmBK9m)

Install the Python bindings

```
python3 -m pip install cuvis
```

### Building the ROS2 Nodes

`cd cuvis.ros && colcon build`

`source install/local_setup.bash` to add the new packages to the search path.

### Running the ROS Nodes

In the files `scripts/ros2_interface.py` and `scripts/datacube_sub.py` update the shebang to match the installation location of your Python3 interpreter. Code will not run without this variable set!

#### Standalone with Default Args

Run the camera driver node:
`ros2 run cuvis_ros ros2_interface.py`

Run sample subscriber node (optional): `ros2 run cuvis_ros datacube_sub.py`

### Helpful hint

Every new terminal you open needs to have the following commands added to it. If you receive errors about missing commands, this is most likely your issue.

``` 
cd <<ROS2 Workspace>>
source install/local_setup.bash
```
### TODO

This section contains additional development goals which will be pursued as time allows.

- [ ] C++ driver implementation
- [ ] Integration with Hyper-Drive common HSI ROS library
- [ ] Add reflectance/radiance calibration measurement
- [ ] Handle loop interrupts with grace
- [X] ROS2 support
- [ ] ROS2 launch file
