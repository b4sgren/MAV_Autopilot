"""
mavsimPy: video making function
    - Beard & McLain, PUP, 2012
    - Update history:  
        1/10/2019 - RWB
"""
import numpy as np
import cv2
#from PIL import ImageGrab
import pyscreenshot as ImageGrab

class video_writer():
    def __init__(self, video_name="video.avi", bounding_box=(0, 0, 1000, 1000), output_rate = 0.1):
        # bbox specifies specific region (bbox= top_left_x, top_left_y, width, height)
        # set up video writer by grabbing first image and initializing 
        img = ImageGrab.grab(bbox=bounding_box)
        img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        height, width, channels = img.shape
        # Define the codec and create VideoWriter object
        #fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fourcc = cv2.VideoWriter_fourcc(*'mjpg')

        self.video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20.0, (width, height))
        self.bounding_box = bounding_box
        self.output_rate = output_rate
        self.time_of_last_frame = 0

    ###################################
    # public functions
    def update(self, time):
        print('Brendon')
        if (time-self.time_of_last_frame) >= self.output_rate:
            print('Howdy')
            img = ImageGrab.grab(bbox=self.bounding_box)
            print('Hey')
            img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
            self.video.write(img)
            self.time_of_last_frame = time

    def close(self):
        self.video.release()


