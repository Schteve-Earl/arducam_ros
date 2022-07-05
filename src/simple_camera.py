#!/usr/bin/env python3
from re import T
import cv2
import numpy as np
import subprocess
import random
import json
import time

class Camera():
    def __init__(self, config_path):
        self.config = self.load_config(config_path)
        self.properties = self.config['properties']
        self.cap = cv2.VideoCapture(self.config['id'], cv2.CAP_V4L2)
        self.init_v4l2()
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        codec = 0x47504A4D  # MJPG
        self.cap.set(cv2.CAP_PROP_FOURCC, codec)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['resolution']['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['resolution']['height'])
        self.set_all()
        # self.capture_images()

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
    
    def init_v4l2(self):
        id = self.config['id']
        fps = self.config['fps']
        format = self.config['format']
        w = self.config['resolution']['width']
        h = self.config['resolution']['height']

        cmd="v4l2-ctl -d {} --set-fmt-video=width={},height={},pixelformat='{}' --set-parm={} ".format(id,w,h,format,fps)

        subprocess.call(cmd, shell=True)

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
            cv2.imshow('frame', cv2.resize(frame, (self.config['output']['width'], self.config['output']['height'])))
            
            # the 'q' button is set as the
            # quitting button you may use any
            # desired button of your choice
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.imwrite('output.png', frame)
                break


if __name__ == "__main__":
    # names = ["imx298_top", "imx298_side", "imx219_top"]
    # for name in names:
    #     c = Camera('/home/stephen/manual_data_collector_ws/src/arducam_ros/config/{}.json'.format(name))

    #     while(True):
        
    #         # Capture the video frame
    #         # by frame
    #         ret, frame = c.cap.read()
        
    #         # Display the resulting frame
    #         cv2.imshow('frame', cv2.resize(frame, (c.config['output']['width'], c.config['output']['height'])))
            
    #         # the 'q' button is set as the
    #         # quitting button you may use any
    #         # desired button of your choice
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             cv2.imwrite('output.png', frame)
    #             break
    a = Camera('/home/stephen/manual_data_collector_ws/src/arducam_ros/config/{}.json'.format("imx298_top"))
    b = Camera('/home/stephen/manual_data_collector_ws/src/arducam_ros/config/{}.json'.format("imx298_side"))
    c = Camera('/home/stephen/manual_data_collector_ws/src/arducam_ros/config/{}.json'.format("imx219_top"))
    while(True):

    # Capture the video frame
    # by frame
        reta, framea = a.cap.read()
        retb, frameb = b.cap.read()
        retc, framec = c.cap.read()

        cv2.imshow('framea', cv2.resize(framea, (a.config['output']['width'], a.config['output']['height'])))
        cv2.imshow('frameb', cv2.resize(frameb, (b.config['output']['width'], b.config['output']['height'])))
        # Display the resulting frame
        cv2.imshow('framec', cv2.resize(framec, (c.config['output']['width'], c.config['output']['height'])))

        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.imwrite('output.png', frame)
            break



