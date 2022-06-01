#!/usr/bin/env python3
import rospkg
import rospy
import camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraManagerNode():
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_publisher_list = {}
        self.camera_object_list = {}
        
        self.get_parameters()
        self.create_publishers()
        self.create_camera_objects()
        
    def get_parameters(self):
        self.camera_names_list = rospy.get_param("/camera_manager_parameters/camera_names")

    def create_publishers(self):
        # Create a camera publisher for every camera in the id list
        for name in self.camera_names_list:
            self.camera_publisher_list[name] = rospy.Publisher('camera_' + name + '_topic', Image, queue_size=1)

    def create_camera_objects(self):
        # Create a camera object for every camera in the id list
        for name in self.camera_names_list:
            rospack = rospkg.RosPack()
            config_path = rospack.get_path('arducam_ros')
            self.camera_object_list[name] = camera.Camera(config_path + '/config/' + name + '.json')

    def publish_images(self):
        # staart a while loop that iterates through every camera object and publishes the read image
        while not rospy.is_shutdown():
            for name in self.camera_names_list:
                cv_image = self.camera_object_list[name].read()
                msg_image = self.bridge.cv2_to_imgmsg(cv_image)
                self.camera_publisher_list[name].publish(msg_image)

if __name__ == "__main__":
    rospy.init_node('camera_manager_node', anonymous=True)
    c = CameraManagerNode()
    c.publish_images()
    

    
