#!/usr/bin/env python3
from re import T
import cv2
import numpy as np
import subprocess
import random
import json

class Camera():
    def __init__(self, config_path):
        self.config = self.load_config(config_path)
        self.properties = self.config['properties']
        self.cap = cv2.VideoCapture(self.config['id'])
        # self.reset_all()
        # self.init_v4l2()
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # codec = 0x47504A4D  # MJPG
        # self.cap.set(cv2.CAP_PROP_FOURCC, codec)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['resolution']['width'])
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['resolution']['height'])
        # self.set_all()
        self.capture_images()

    def load_config(self, config_path):
        config = json.load(open(config_path,'r')) 
        if 'crop' in config.keys():
            self.crop = True
        if config['output']['width'] is not config['resolution']['width']:
            if self.crop:
                if config['output']['width'] is not config['crop']['width']:
                    self.scale = True
        if config['output']['height'] is not config['resolution']['height']:
            if self.crop:
                if config['output']['height'] is not config['crop']['height']:
                    self.scale = True
        if config['flip'] == True:
            self.flip = True
        dtype_list = ['int', 'list', 'float', 'bool']
        for prop_name in config['properties'].keys():
            prop = config['properties'][prop_name]
            if prop['dtype'] not in dtype_list:
                raise TypeError("Camera Config Property '{}' with unknown dtype '{}'".format(prop_name, prop['dtype']))
            if prop['randomize'] is not False:
                self.check_randomize_values(prop_name, prop)
        return config
        capture = cv2.VideoCapture(0)

    def set_cap_prop(self, prop, value):
        if prop:
            return self.cap.set(eval("cv2.{}".format(prop)), value)
        
    def set_all(self):
        for prop in self.properties:
            value = self.properties[prop]['value']                 
            if self.set_cap_prop(self.properties[prop]['cv2'], value):
                self.properties[prop]['value'] = value
    
    def capture_images(self):

        while(True):
      
            # Capture the video frame
            # by frame
            ret, frame = self.cap.read()
        
            # Display the resulting frame
            cv2.imshow('frame', frame)
            
            # the 'q' button is set as the
            # quitting button you may use any
            # desired button of your choice
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.imwrite('output.png', frame)
                break


if __name__ == "__main__":
    
    c = Camera('/home/stephen/manual_data_collector_ws/src/arducam_ros/config/imx298_top.json')



