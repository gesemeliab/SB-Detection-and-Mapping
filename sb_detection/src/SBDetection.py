#!/usr/bin/env python
## Author: Dani Esparza, Gesem Gudino, Marcos Rocha, Antonio 
## Date: April, 16, 2020
# Purpose: Ros node to detect objects using tensorflow
#import math
#import os
import sys
import cv2
import strawberry as sb

#CNN's Related imports
import tensorflow as tf
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
  try:
    for gpu in gpus:
      tf.config.experimental.set_memory_growth(gpu, True)
  except RuntimeError as e:
    print(e)
from tensorflow.keras.models import load_model


# ROS related imports
import rospy
from sensor_msgs.msg import Image

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import message_filters
from cv_bridge import CvBridge, CvBridgeError

f=1396.91

#max length in case that you want to visualize the straberries trajectorie
#centers = deque(maxlen=20)

PATH_TO_MODEL = ""
model = load_model(PATH_TO_MODEL)

image_pub = rospy.Publisher("debug_image",Image, queue_size=10)
sb_coor_pub = rospy.Publisher("sb_coordinates",PoseStamped, queue_size=10)

#Record a video of the detection
#out = cv2.VideoWriter('out.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 12, (720,480))


def image_cb(im1, im2, rover):

    bridge = CvBridge()

    try:
        image_np = bridge.imgmsg_to_cv2(im1, "bgr8")   
        dpth_image = bridge.imgmsg_to_cv2(im2, "passthrough")


    except CvBridgeError as e:
        print(e)

    if image_np is not None:
        
        #image_np.shape[0] gives the height of the image and image_np.shape[1] the width
        height = image_np.shape[0]
        width = image_np.shape[1] 

        contours, binary_img = sb.SBFilter(image_np)
        
        central_region_r = (width/2)+int(width/20)
        central_region_l = (width/2)-int(width/20)
        #coord_array = PoseArray()
        #coord_array.header.stamp = rospy.Time.now()
        #coord_array.poses = []

        ##In case that you want to visualize the contours, uncomment the line below 
        #cv2.drawContours(image_np, contours, -1, (255,0,255), 3)
        
        #In case you want to visualize the proposed central region of the image
        #cv2.rectangle(image_np,(central_region_l,0),(central_region_r,height),(255,255,255),5)

        
        for c in range (0,len(contours)):    
           
            #storing the information of the box that surrounds every contour
            rect = cv2.boundingRect(contours[c])
            x,y,w,h = rect
            x -= 5
            if x < 0:
                x = 0   
            y -= 5
            w += 15
            h += 15
            
            #Future Works
            ##Searching for a stride and kernel size to complete n sweeps in the proposed region
            ##given the width or height of the kernel, returns the stride and the resting dimension
            #stride, kw = sb.search4stride(w,3)
            #stride2, kh = sb.search4stride(h,3)
            
            #Defining the region for sweeping
            '''
            region_w = kw*3
            region_x = (x+(w/2))-(region_w/2)
            if region_x < 0:
                region_x = 0
            region_x2 = region_x + region_w                
            region_y = y-h
            region_h = kh*3
            region_y2 = region_y + region_h
            '''      

            if len(contours) >= 1:
                if x+w <= image_np.shape[1]:   
                    
                    #Calling the cnn model to verify each box
                    label, acc = sb.Predict(binary_img, (x,y,w,h), model, 128)
                    if acc >= 0.98:      
                        #Creating a variable with centroid information
                        centroid = x+w/2, y+h/2
                        if centroid[0] < central_region_r and centroid[0] > central_region_l:
                            
                            #Strawberry depth
                            depth = sb.Depth(dpth_image, centroid)
    
                            #Coordinate transformation (Pixels -> Meters)
                            coordinates_cam = sb.TransfCoord(centroid,height,width,f,depth)
                            sb_coor= PoseStamped()
                            sb_coor.header.stamp = rospy.Time.now()
                            
                            #Broadcasting the straberry coordinates 
                            sb_coor.pose.position.x = coordinates_cam[0] + rover.pose.pose.position.x   #
                            sb_coor.pose.position.z = coordinates_cam[1] + rover.pose.pose.position.z   #Right hand satys the same regardless 
                            sb_coor.pose.position.y = coordinates_cam[2] + rover.pose.pose.position.y   #the cameraposition on the rover
                            sb_coor_pub.publish(sb_coor)
    
                            #coord_array.poses.append(sb_coor)
                            
                            #appending the coordinates of each centroid in a deque with certain max length,
                            #this to visualize the past trajectory of each centroid
                            #centers.appendleft(centroid)
    
                            ##To visualize the trajectory of the centroid of each strawberry uncomment the lines below 
                            #for point in range (0,len(centers)):            
                                #cv2.circle(image_np, centers[point], 3, (0,point*10,0), -1)
                            
                            #Drawing the red strawberry box (Green when is in between the central region)
                            cv2.rectangle(image_np,(x,y),(x+w,y+h),(0,255,0),2)
    
                            #Highlight the center of the red strawberry
                            #cv2.circle(image_np, centroid, 3, (255, 0, 0), -1)
    
                            #Drawing the region (Future Works)
                            #cv2.rectangle(image_np,(region_x,region_y),(region_x2,region_y+region_h),(255,0,0),2)
    
                            #Performing the sweep
                            #conv, coordinates =sb.Scanner(image_np, region_x, region_y, region_x2, region_y2, kw, kh, stride, stride2)
    
                            if label == 0:
                                cv2.putText(image_np, 'acc: {}'.format(acc), (x,y-10), cv2.FONT_HERSHEY_SIMPLEX,  0.75, (0,0,200), 2, cv2.LINE_AA)
                                #cv2.putText(image_np, 'Red', (x+w,y+h), cv2.FONT_HERSHEY_SIMPLEX,  1, (0, 0, 255), 2, cv2.LINE_AA)
                            else:
                                cv2.putText(image_np, 'Green', (x+w,y+h), cv2.FONT_HERSHEY_SIMPLEX,  1, (0, 255, 0), 2, cv2.LINE_AA)
                        else:
                            #Drawing the box when the strawberry is not in the central region
                            cv2.rectangle(image_np,(x,y),(x+w,y+h),(0,0,200),2)
                            
                            
        #Display the image
        #in case you want to save/visualize the binary image just change the first argument 
        #in the resize function for: binary_img    
        img = cv2.resize(image_np, (480,480))
        
        #Writing a video of the detection
        #out.write(cv2.resize(image_np, (720,480)))
  
    #Broadcasting the de detection image
    image_out = Image()
    try:
        image_out = bridge.cv2_to_imgmsg(img,"bgr8")
    except CvBridgeError as e:
        print(e)
    image_out.header = im1.header
    image_pub.publish(image_out)
    #sb_coor_pub.publish(coord_array)



def main(args):
    rospy.init_node('detector_node')
    #Subscriptions
    im_1 = message_filters.Subscriber("/zedm/zed_node/left/image_rect_color", Image, queue_size = 1, buff_size=2**24)
    im_2 = message_filters.Subscriber("/zedm/zed_node/depth/depth_registered", Image)
    zed_pos = message_filters.Subscriber("/zedm/zed_node/odom", Odometry)
    ts = message_filters.TimeSynchronizer([im_1, im_2, zed_pos], 10)
    print("Registering callback")
    ts.registerCallback(image_cb)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)
