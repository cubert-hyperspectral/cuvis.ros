#!/install/venv_3.9/bin/python3.9

### Change this line to match the Python distribution

# Python imports
import os
import time
import numpy as np
from datetime import timedelta

# Custom imports
import cuvis

# ROS imports
import rospy
import std_msgs
import rospkg
from cuvis_ros.msg import DataCube


class CameraDriver:
    def __init__(self):
        # Initialize configuration directories
        self.ros_pack = rospkg.RosPack()
        self.default_dir = os.path.join(self.ros_pack.get_path('cuvis_ros'),'cuvis_factory')
        print(self.default_dir)
        dataDir = rospy.get_param('data_dir', self.default_dir)
        factoryDir = rospy.get_param('factory_dir', self.default_dir)
        self.timeout = rospy.get_param('camera_timeout', 2500) # Timeout for acquisition, this value might need to be large
        userSettingsDir = os.path.join(dataDir, "settings")
        self.exposure = rospy.get_param('integration_time', 30) # integration time in ms
        self.rate = rospy.get_param('loop_rate', 1) # Rate at which the publishing loop will run
        self.r = rospy.Rate(self.rate) 

        rospy.loginfo("loading calibration, processing and acquisition context (factory)...")
        calibration = cuvis.Calibration(factoryDir)

        rospy.loginfo("loading user settings...")
        settings = cuvis.General(userSettingsDir)
        settings.set_log_level("info")
        self.processingContext = cuvis.ProcessingContext(calibration)
        self.acquisitionContext = cuvis.AcquisitionContext(calibration)
        while self.acquisitionContext.state == cuvis.HardwareState.Offline:
            print(".", end="")
            rospy.sleep(1)

        rospy.loginfo("Camera is online")
        self.acquisitionContext.operation_mode = cuvis.OperationMode.Software
        self.acquisitionContext.integration_time = self.exposure
        
        # Publisher for the raw hyperspectral image
        self.hypercube_pub = rospy.Publisher('hyperspectral/raw_img', DataCube, queue_size=10)
        
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
        h.stamp = rospy.Time.now()
        msg.header = h
        msg.height, msg.width, msg.lam = list(raw_img.shape)
        msg.data = raw_img.flatten().astype(np.int16)
        msg.integration_time = int(mesu.integration_time)
        rospy.loginfo(f'Got frame! Shape:{raw_img.shape}')

        self.hypercube_pub.publish(msg)
        self.r.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('hyperspectral_camera_driver', anonymous=True)
        processor = CameraDriver()
        while not rospy.is_shutdown(): 
            processor.record_img()
    except rospy.ROSInterruptException:
        rospy.logerr('Exception in main processing loop!')