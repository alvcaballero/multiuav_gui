#!/usr/bin/env python3

'''
    Python file to process the thermal image extracted from the M300 missions

'''

import sys, getopt
from thermal_base import ThermalImage

import cv2
import numpy as np

# Thermal Image annotation
from thermal_base import ThermalImageAnnotation
# Manipulation of the image
from thermal_base import utils

import json

from PIL import Image
from PIL.ExifTags import TAGS
import piexif
import piexif.helper

## Functions
# to the temperature in a single point over the image given the coordinates of the point
def show_temp(x,y,thermal_np,temp_image):
    
    # print the value over the image showed with openCV
    #cv2.putText(temp_image, str(thermal_np[x,y]), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.circle(temp_image, (x, y), 2, (0, 0, 0), -1)
    cv2.putText(temp_image," {:3.2f} C".format(thermal_np[y,x]),(x - 80, y - 15), cv2.FONT_HERSHEY_PLAIN, 1,(255,0,0),2)
    print("The temperature at point ({},{}) is {:.2f} º C".format(x,y,thermal_np[y,x]))

    return

def find_max_temp(thermal_np,temp_image):
    # Find the maximum value in the array
    max_temp = np.max(thermal_np)

    # Find the maximum value indexes
    max_idx = np.argwhere(thermal_np == max_temp)

    print("{data:{MaximumTemperature:"+ str(max_temp) + ", TemperatureIndexes:["+ str(max_idx[0][0])+","+ str(max_idx[0][1])+"] } }" )
    #print(type(max_idx[0]))
    #show_temp(max_idx[0][1],max_idx[0][0],temp_image)
    cv2.circle(temp_image, (max_idx[0][1], max_idx[0][0]), 2, (0, 0, 0), -1)
    cv2.putText(temp_image,"Max: {:.2f} C".format(thermal_np[max_idx[0][0],max_idx[0][1]]),(max_idx[0][1] - 80, max_idx[0][0] - 15), cv2.FONT_HERSHEY_PLAIN, 1,(255,255,255),2)



def main(argv):
   
    # CLI arguments parsing
    inputfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["source=","dest="])
    except getopt.GetoptError:
        print ('processThermalImg.py -i <inputfile-path> -o <outputfile-path>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ('processThermalImg.py -i <inputfile-path> -o <outputfile-path>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    print ('Input file is ', inputfile)
    print ('Output file is ', outputfile)

    # Decoding the thermal image
    # ex: /home/ubuntugrvc/thermal_cameras/examples/H20T/DJI_202307211037_002/DJI_20230721104850_0004_T.JPG
    path_to_image = inputfile

    image = ThermalImage(image_path=path_to_image, camera_manufacturer="dji")
    thermal_np = image.thermal_np           # The temperature matrix as a np array
    raw_sensor_np = image.raw_sensor_np     # The raw thermal sensor excitation values as a np array
    meta = image.meta                       # Any other metadata that exiftool picked up
    colorbar = image.generate_colorbar(cmap = cv2.COLORMAP_HOT)   # Returns: np.ndarray: A colourbar of the required height with temperature values labelled

    print(thermal_np)
    print("the shape of the thermal image is:", thermal_np.shape)
    


    

    img_ann = ThermalImageAnnotation()

    
    #print(dir(utils))                                   # View manipulation tools available
    #thermal_np = utils.change_emissivity_for_roi(...)   # Sample: Change the emissivity of an RoI

    temp_image = utils.get_temp_image(thermal_np, colormap=cv2.COLORMAP_HOT)     
    print("Thermal image as numpy image:", type(temp_image))
    

    # As we have the thermal image and a matrix of the same size with the temperature values, we can now do some image processing
    #show_temp(326,198,thermal_np, temp_image) # temperature of my head
    #show_temp(344,244,thermal_np, temp_image) # temperature of the floor
    #show_temp(141,78,thermal_np, temp_image) # temperature of the RTK

    find_max_temp(thermal_np, temp_image)

    concat = cv2.hconcat([temp_image, colorbar])
    # Saving the image
    cv2.imwrite(outputfile, concat)
    
    # insert custom data in usercomment field
    user_data = {
        # One decimal only
        "MaxTemp": str(thermal_np.max().round(1)),
        "MaxTempPos": str(np.argwhere(thermal_np == thermal_np.max())[0])
    }
    exif_out = piexif.load(inputfile)
    exif_out["Exif"][piexif.ExifIFD.UserComment] = piexif.helper.UserComment.dump(
        json.dumps(user_data),
        encoding="unicode"
    )
    # insert mutated data (serialised into JSON) into image
    piexif.insert(
        piexif.dump(exif_out),
        outputfile
    )        



if __name__ == "__main__":
   main(sys.argv[1:])