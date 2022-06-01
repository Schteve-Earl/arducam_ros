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
        self.reset_all()
        self.init_v4l2()
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        codec = 0x47504A4D  # MJPG
        self.cap.set(cv2.CAP_PROP_FOURCC, codec)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['resolution']['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['resolution']['height'])
        self.set_all()

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

    def get_id(self):
        return self.config['id']

    def get_resolution(self):
        return (self.properties['resolution'][['width']], self.properties['resolution'][['height']])

    def init_v4l2(self):
        id = self.config['id']
        fps = self.config['fps']
        format = self.config['format']
        w = self.config['resolution']['width']
        h = self.config['resolution']['height']
        # cmd="v4l2-ctl -d {} -set-fmt-video=width={},height={},pixelformat='{}' --set-parm={} ".format(w,h,format,fps, id)
        cmd="v4l2-ctl --set-parm={} -d {} ".format(fps, id)
        subprocess.call(cmd, shell=True)

    def crop_frame(self, img):
        return cv2.resize(img, (self.config['output']['width'], self.config['output']['height']))

    def flip_image(self, img):
        return cv2.flip(img, 0)

    def scale_frame(self, img):
        width, height = img.shape[1], img.shape[0]
        dim = (self.config['output']['width'], self.config['output']['height'])
        crop_width = dim[0] if dim[0]<img.shape[1] else img.shape[1]
        crop_height = dim[1] if dim[1]<img.shape[0] else img.shape[0] 
        mid_x, mid_y = int(width/2), int(height/2)
        cw2, ch2 = int(crop_width/2), int(crop_height/2) 
        crop_img = img[mid_y-ch2:mid_y+ch2, mid_x-cw2:mid_x+cw2]
        return crop_img

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        else:
            if self.crop:
                frame = self.crop_frame(frame)
            if self.scale:
                frame = self.scale_frame(frame)
            if self.flip:
                frame = self.flip_image(frame)
            return frame

    def release(self):
        self.cap.release()

    def set_cap_prop(self, prop, value):
        if prop:
            return self.cap.set(eval("cv2.{}".format(prop)), value)
        
        
    def check_randomize_values(self, prop_name, prop):
        dtype = prop['dtype']
        values = prop['randomize']
        if dtype == "int":
            if type(values) is not list:
                raise TypeError("ERROR: Property Randomization '{}': should be type list".format(prop_name))
            if len(values) != 2:
                raise TypeError("ERROR: Property Randomization '{}': should be list of length 2".format(prop_name))
            if type(values[0]) is not int or type(values[0]) is not int:
                raise TypeError("ERROR: Property Randomization '{}': should be list of type int".format(prop_name))
            if values[0] < prop['min']:
                raise ValueError("ERROR: Property Randomization '{}': range cannot be less than min".format(prop_name))
            if values[1] > prop['max']:
                raise ValueError("ERROR: Property Randomization '{}': range cannot be greater than max".format(prop_name))
        elif dtype == 'list':
            if type(values) is not list:
                raise TypeError("ERROR: Property Randomization '{}': should be type list".format(prop_name))
        elif dtype == 'float':
            if type(values) is not list:
                raise TypeError("ERROR: Property Randomization '{}': should be type list".format(prop_name))
            if len(values) != 2:
                raise TypeError("ERROR: Property Randomization '{}': should be list of length 2".format(prop_name))
            if type(values[0]) is not int or type(values[0]) is not int:
                raise TypeError("ERROR: Property Randomization '{}': should be list of type int".format(prop_name))
            if values[0] < prop['min']:
                raise ValueError("ERROR: Property Randomization '{}': range cannot be less than min".format(prop_name))
            if values[1] > prop['max']:
                raise ValueError("ERROR: Property Randomization '{}': range cannot be greater than max".format(prop_name))
        elif dtype == 'bool':
            if values is not True:
                raise TypeError("ERROR: Property Randomization '{}': should be type bool".format(prop_name))

    def get_random(self, dtype, values):
        if dtype == "int":
            return random.randint(values[0], values[1])
        elif dtype == 'list':
            return random.choice(values)
        elif dtype == 'float':
            return np.random.uniform(values[0], values[1])
        elif dtype == 'bool':
            return random.choice([0,1])
        else:
            return None
            
    def reset_prop(self, prop_name):
        prop = self.properties[prop_name]
        value = prop['default']                 
        if self.set_cap_prop(prop['cv2'], value):
            prop['value'] = value

    def reset_all(self):
        for prop in self.properties:
            value = self.properties[prop]['default']                 
            if self.set_cap_prop(self.properties[prop]['cv2'], value):
                self.properties[prop]['value'] = value
    
    def set_all(self):
        for prop in self.properties:
            value = self.properties[prop]['value']                 
            if self.set_cap_prop(self.properties[prop]['cv2'], value):
                self.properties[prop]['value'] = value

    def randomize(self):
        for prop in self.properties:
            if prop['randomize'] is not False:
                value = self.get_random(prop['dtype'], prop['randomize'])                    
                if self.set_cap_prop(prop['cv2'], value):
                    prop['value'] = value

    def get_values(self):
        properties = {}
        for prop in self.properties.keys():
            properties[prop] = self.properties[prop]['value']
        return properties


        



