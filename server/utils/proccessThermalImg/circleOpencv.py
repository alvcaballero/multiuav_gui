import numpy as np
import cv2
import sys #import Sys. Sys will be used for reading from the command line. We give Image name parameter with extension when we will run python script


print( 'Number of arguments:'+ str( len(sys.argv))+ 'arguments.')
print('Argument List:'+ str(sys.argv))
#img = cv2.imread('/path_to_image/opencv-logo.png', 0) 
img = cv2.imread(sys.argv[1])

height, width, channels = img.shape
row, col = int(height/2), int(width/2)

cv2.circle(img,(col, row), 50, (0,255,0), -1)

cv2.imwrite(sys.argv[2], img)