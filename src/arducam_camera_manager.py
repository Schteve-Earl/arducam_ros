#!/usr/bin/env python3
import rospkg
import rospy
import camera
from arducam_ros.srv import CreateImage, CreateImageResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraManagerNode():
    def __init__(self):
        self.bridge = CvBridge()
        
        self.get_parameters()
        self.create_publishers()
        self.create_all_cameras()
        self.publish_images()
        # self.create_service_servers()
        
    def get_parameters(self):
        self.camera_names_list = rospy.get_param("/camera_manager_parameters/camera_names")
        self.top_camera_name = rospy.get_param("/camera_manager_parameters/top_camera_name")

    def create_publishers(self):
        # Create a camera publisher for every camera in the id list
        self.camera_publisher_list = {}
        for name in self.camera_names_list:
            self.camera_publisher_list[name] = rospy.Publisher('camera_' + name + '_topic', Image, queue_size=1)

    def create_service_servers(self):
        self.camera_service_server_list = {}
        for name in self.camera_names_list:
            self.camera_service_server_list[name] = rospy.Service('camera_' + name + '_service', CreateImage, lambda msg: self.camera_service_handler(msg, name))
    
    def camera_service_handler(self, req: CreateImage, name: str):
        """Initializes a camera using the given name, and then returns the requested number of images

        Args:
            req (CreateImage): request containing number of images to be returned
            name (str): name of the camera to be initialized

        Returns:
            res (CreateImageRes): response containing list of images requested
        """

        res = CreateImageResponse()
        image_list = []

        c = camera.Camera(self.get_config_path() + '{}.json'.format(name))

        for i in range(req.image_requests):
            image_list.append(self.bridge.cv2_to_imgmsg(c.read()))
        res.images = image_list
        c.release()
        return res

    def create_all_cameras(self):
        # Create a camera object for every camera in the id list
        self.camera_object_list = {}
        for name in self.camera_names_list:
            rospack = rospkg.RosPack()
            config_path = rospack.get_path('arducam_ros')
            self.camera_object_list[name] = camera.Camera(config_path + '/config/' + name + '.json')

    def publish_images(self):
        # start a while loop that iterates through every camera object and publishes the read image
        while not rospy.is_shutdown():
            for name in self.camera_names_list:
                cv_image = self.camera_object_list[name].read()
                try:
                    msg_image = self.bridge.cv2_to_imgmsg(cv_image)
                    self.camera_publisher_list[name].publish(msg_image)
                except (CvBridgeError, TypeError) as e:
                    rospy.logerr("{}: {}".format(name, e) )

    def get_config_path(self):
        """Creates and returns the path to the config folder of this package
        """
        rospack = rospkg.RosPack()
        return rospack.get_path('arducam_ros') + '/config/'

if __name__ == "__main__":
    rospy.init_node('camera_manager_node', anonymous=True)
    c = CameraManagerNode()


    rospy.spin()
    

    
