#!/home/nathaniel/cubert/venv_3.9/bin/python3

### Change this line to match the Python distribution

# Python imports
import os
import time
import numpy as np
from datetime import timedelta

# Custom imports
import cuvis

# ROS imports
import rclpy
import std_msgs
import ament_index_python
from rclpy.node import Node
from std_msgs.msg import String
from cuvis_ros.msg import DataCube




class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        # Initialize configuration directories
        self.declare_parameter('camera_timeout', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('integration_time', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('loop_rate', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('data_dir', rclpy.Parameter.Type.STRING)

        # Retrieve a parameter with a default value if not set
        self.timeout = self.get_parameter('camera_timeout').get_parameter_value(2500).integer_value
        self.exposure = self.get_parameter('integration_time').get_parameter_value(30).integer_value
        self.rate = self.get_parameter('loop_rate').get_parameter_value(1).integer_value # 1 Hz
        self.r = self.create_rate(self.rate)

        # self.ros_pack = rospkg.RosPack()
        package_path = ament_index_python.get_package_prefix('cuvis_ros')
        print(package_path)
        self.default_dir = os.path.join(package_path,'cuvis_factory')
        dataDir = self.get_parameter('data_dir').get_parameter_value(self.default_dir).string_value
        factoryDir = self.get_parameter('factory_dir').get_parameter_value(self.default_dir).string_value
        userSettingsDir = os.path.join(dataDir, "settings")

        rclpy.getlogger().info("loading calibration, processing and acquisition context (factory)...")
        calibration = cuvis.Calibration(factoryDir)

        rclpy.getlogger().info("loading user settings...")
        settings = cuvis.General(userSettingsDir)
        settings.set_log_level("info")
        self.processingContext = cuvis.ProcessingContext(calibration)
        self.acquisitionContext = cuvis.AcquisitionContext(calibration)

        init_rate = self.create_rate(1)
        while self.acquisitionContext.state == cuvis.HardwareState.Offline:
            print(".", end="")
            init_rate.sleep(1)

        rclpy.getlogger().info("Camera is online")
        self.acquisitionContext.operation_mode = cuvis.OperationMode.Software
        self.acquisitionContext.integration_time = self.exposure
        
        # Publisher for the raw hyperspectral image
        self.hypercube_pub = self.create_publisher('hyperspectral/raw_img', DataCube, queue_size=10)
        
    def record_img(self):
        '''
        Record raw hyperspectral image
        '''
        am = self.acquisitionContext.capture()
        mesu, res = am.get(timedelta(milliseconds=self.timeout))

        if mesu is not None:
            self.processingContext.apply(mesu)
            raw_img = mesu.data.get('cube').array

        msg = DataCube()
        h = std_msgs.msg.Header()
        h.stamp = self.get_clock().now()
        msg.header = h
        msg.height, msg.width, msg.lam = list(raw_img.shape)
        msg.data = raw_img.flatten().astype(np.int16)
        msg.integration_time = int(mesu.integration_time)
        rclpy.getlogger().info(f'Got frame! Shape:{raw_img.shape}')

        self.hypercube_pub.publish(msg)
        self.r.sleep()

def main(args=None):
    rclpy.init(args=args)
    camDriver = CameraDriver()
    while rclpy.ok():
        camDriver.record_img()
    camDriver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()