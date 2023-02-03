#!/usr/bin/env python3
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes

from rclpy.action import ActionClient,ActionServer

from sensor_msgs.msg import Image # Image is the message type
# from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import time
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import math

from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle

from action_interfaces.action import Dock

import depthai as dai 


class DockActionServer(Node):

    def __init__(self):
        super().__init__('dock_turtle_action_server')

        self._action_server = ActionServer(self,Dock,'dock_turtle',self.execute_callback)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        dock = ImageSubscriber()

        result = Dock.Result()
        result.finished = 1
        goal_handle.succeed()

        self.get_logger().info('Docking finished...')

        return result


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        #self.bridge = CvBridge()

        self.error = None
        self.angle = None
        self.stop = False
        self.img = None
        self.go = True
        self.flag = True
        self.avg_line = None

        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define source and output
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.xoutVideo = self.pipeline.create(dai.node.XLinkOut)

        self.xoutVideo.setStreamName("video")

        # preview publisher
        self.image_publisher = self.create_publisher(Image, 'color/image', 1)

        # Properties
        self.width = 1280
        self.height = 1000
        self.camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.camRgb.setVideoSize(self.width, self.height)

        self.xoutVideo.input.setBlocking(False)
        self.xoutVideo.input.setQueueSize(1)

        # Linking
        self.camRgb.video.link(self.xoutVideo.input)

        # Connect to device and start pipeline
        self.device = dai.Device(self.pipeline)

        self.video = self.device.getOutputQueue(name="video", maxSize=1, blocking=False)
        
        self._rotate = ActionClient(self,RotateAngle,'rotate_angle')
        self._dist = ActionClient(self,DriveDistance,'drive_distance')
  
        # self.sub = self.create_subscription(Image,'/color/preview/image',self.listener_callback,1)  

        while self.go:
            self.run()

    
    def get_image(self):
        videoIn = self.video.get()
        self.img = videoIn.getCvFrame()
        print(self.img.shape)
        self.frame = self.img
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

        # cv2.imshow("get img",self.img)
        # cv2.waitKey(100)
        
        if self.go == True and self.stop == False:
            print(f"called {self.go}")
        
        return self.img

    def run(self):
        self.get_image()
        self.qr_scan()

        if self.error is None: # check if QR code is recognized
            self.go = True     # if not -> get new pic
        else:
            if abs(self.angle) > 0.05: # ^= 5 degrees
                print(f"print {self.angle}")
                self.get_logger().info('Angle correction started')           
                self.control_angle()
            #elif abs(self.error) > 2: # pixel error to image middle
            #    self.get_logger().info('Translation correction started')
            #    x = 0.1
            #    self.control_translation(x)
            else:
                self.drive_straight()

            # self.go = True
            self.error = None # reset error to get new error in new frame


    def drive_straight(self):
        transl_msg = DriveDistance.Goal()
        transl_msg.max_translation_speed = 0.1
        if self.avg_line < 550:
            transl_msg.distance = 0.1
        
        elif self.avg_line < 650:
            transl_msg.distance = 0.05
        else:
            transl_msg.max_translation_speed = 0.05
            transl_msg.distance = 0.01

        #cmd = Twist()
        if self.stop == False:
            #cmd.linear.x = 0.1
            print("waiting for server...")
            self._dist.wait_for_server()
            self._dist.send_goal_async(transl_msg)
            time.sleep(abs(transl_msg.distance/transl_msg.max_translation_speed) + 1)
            print("Translation Action finished")
        
    def control_angle(self):
        self.flag = True
        # Action commands rotation
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = self.angle
        goal_msg.max_rotation_speed = 0.1 # m/s
        print("waiting for server...")
        self._rotate.wait_for_server()
        print("goal_send to robot")
        self._rotate.send_goal_async(goal_msg)

        time.sleep(abs(goal_msg.angle/goal_msg.max_rotation_speed) + 1)
        print("Angle Action finished")

        
    def qr_scan(self):

        qcd = cv2.QRCodeDetector()
        retval,_, points,_ = qcd.detectAndDecodeMulti(self.img)
        _,corners = qcd.detect(self.img)

        if retval == True and corners is not None:
            self.img = cv2.polylines(self.img, points.astype(int), True, (0, 255, 0), 3)
            dimensions = self.img.shape
            img_center = [int(dimensions[0]/2),int(dimensions[1]/2)]

            x1,y1 = corners[0][0]
            x2,y2 = corners[0][1]
            x3,y3 = corners[0][2]
            x4,y4 = corners[0][3]
            
            x = [x1,x2,x3,x4]
            y = [y1,y2,y3,y4]

            qr_x_center = int((sum(x)/4))
            qr_y_center = int((sum(y)/4))

            self.error = img_center[1]-qr_x_center
            print(f'Error: {self.error}\n')

            # FOV_degrees = 69
            FOV_degrees = self.width/1920 * 69
            
            FOV = FOV_degrees*math.pi/180

            # print(dimensions)
            d = dimensions[1]/(2*math.tan(FOV/2))

            self.angle = math.atan(self.error/d)
            # print(self.angle)
            print(f'Angle error: {self.angle*180/math.pi}°\n')

            cv2.circle(self.img,(qr_x_center,qr_y_center),3,(0,0,255),-1) 
            cv2.line(self.img,(img_center[1],0),(img_center[1],int(dimensions[0])),(255,0,0),1)

            cv2.circle(self.frame,(qr_x_center,qr_y_center),3,(0,0,255),-1) 
            cv2.line(self.frame,(img_center[1],0),(img_center[1],int(dimensions[0])),(255,0,0),1)
            print(img_center[1])

            line1 = y1-y4
            line2 = y2-y3
            print(f"line1 {line1}")
            print(f"line2 {line2}")
            self.avg_line = (abs(line1)+abs(line2))/2
            threshold = 750 # size of qr when goal reached


            if self.avg_line > threshold:
                print("Docking finished")
                self.stop = True
                self.go = False
                
            print('QR')

        else:
            print('No QR found')
            self.error = None


        msg = Image()
        msg.width = self.width
        msg.height = self.height
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = np.reshape(self.frame, (self.height, self.width * 3)).flatten().tolist()
        self.image_publisher.publish(msg)
        # cv2.imshow("Camera", self.img)
        # cv2.waitKey(1)


   

def main(args=None):
    #image_subscriber = ImageSubscriber()

    #while rclpy.ok():
    #    rclpy.spin_once(image_subscriber)

    #image_subscriber.destroy_node()
    #rclpy.shutdown()

    rclpy.init(args=args)

    action = DockActionServer()

    rclpy.spin(action)

if __name__ == '__main__':
    main()