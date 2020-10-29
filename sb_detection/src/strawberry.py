import math
import cv2
import numpy as np


def Depth(image,ob_info):
  if ob_info is not None:
    #v_center = ob_info_[0] + width_ * ob_info_[1]
    #new_image = np.array(image.data[0]).astype(np.float32)
    depth = float(image[ob_info[1],ob_info[0]])
    
    if math.isinf(depth) == True:
      depth = float(0)
      #print (depth,"inf")
      #return depth
    
    elif math.isnan(depth) == True:
      depth = float(0)
      #print (depth,"Nan")

    #print (depth)
  
  if ob_info is None:
    depth = float(0)
    #print (depth,"no hay objeto")
  
  return depth


def SBFilter(src):
        #Converting the RGB image into a LAB image
    lab_image = cv2.cvtColor(src, cv2.COLOR_BGR2LAB)
        #Separating channels of the color space
    #l_channel,a_channel,b_channel = cv2.split(lab_image)
    a_channel = lab_image[:,:,1]
        #Removing big false positives
    #a_channel =a_channel-np.abs(128-b_channel) 
        #Highlighting the strawberry
    #ret2,th = cv2.threshold(a_channel,175,255,cv2.THRESH_TRUNC)
	#Binarizing the image
    ret3,th2 = cv2.threshold(a_channel,165,255,cv2.THRESH_BINARY)
    
      #Filtering small false positives
    kernel = np.ones((15,15),np.uint8)
    filtered = cv2.morphologyEx(th2, cv2.MORPH_OPEN, kernel) 
    filtered = cv2.dilate(filtered,kernel,iterations = 1)

    a_channel_3d = np.zeros((src.shape[0],src.shape[1],3))
    a_channel_3d[:,:,0] = a_channel
    a_channel_3d[:,:,1] = a_channel 
    a_channel_3d[:,:,2] = a_channel

    	##Searching for contours
    contours, hierarchy = cv2.findContours(filtered, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    return contours, a_channel_3d


  ##Searches the nearest stride and kernel size such that a given number of sweeps is satisfied	
def search4stride(kernelw, num_sweeps):
    while True:    
        if (2*kernelw) % num_sweeps == 0:
            stride = (2*kernelw) / num_sweeps
            #if stride == 0:
                #print "kernelw: ", kernelw, "num_sweeps", num_sweeps 
            return stride, kernelw 
            break
        else: 
            kernelw += 1


  ##Displays the kernel sweeping the image
def Scanner(img, rx, ry, rx2, ry2, kw, kh, stride_x, stride_y):#, num):
    conv_list =[]
    coordinates_list = []
    for i in range (ry,(ry2-kh)+1,stride_y):
        for j in range (rx,(rx2-kw)+1,stride_x):
              
            crop = img[i:i+kh, j:j+kw]
            coordinates_tuple = (i,j,i+kh,j+kw)
            
            coordinates_list.append(coordinates_tuple)
            conv_list.append(crop)
            
            cv2.rectangle(img,(j,i),(j+kw,i+kh),(255,255,255),1)
            
    conv = np.array(conv_list)
    coordinates = np.array(coordinates_list)
    
    return conv, coordinates
                
    #Coordinate transformation (Pixels -> Meters)
def TransfCoord(centroid_1,height,width,f,z):
    cx=width/2
    cy=height/2
    mat_transform=np.array([[z/f,  0 , 0 , -(cx*z)/f ], 
                            [ 0 , -z/f, 0 , (cy*z)/(f)],
                            [ 0 ,  0 , z ,     0    ]])
    coordinates_img=np.asarray(centroid_1)
    coordinates_img=np.append(coordinates_img,(1,1))
    coordinates_cam=mat_transform.dot(coordinates_img)
    return coordinates_cam

    #CNN Prediction
def Predict(image, box, cnn_model, size):
    img = image[:]
    x, y, w, h = box
    crop = img[y:y+h, x:x+w]
    crop = cv2.resize(crop, (size,size))
    crop /= 255.0
    crop = np.expand_dims(crop, axis=0)
    probs = cnn_model.predict(crop, batch_size=1)
    prediction = probs.argmax(axis=1)
    print('acc: ', probs[0][prediction], '  label: ', prediction)
    return prediction, probs[0][prediction]


