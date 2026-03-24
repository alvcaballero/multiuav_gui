#!/usr/bin/env python3

'''
    Python file to process the thermal image extracted from the M300 missions

'''

import sys
import getopt
from thermal_base import ThermalImage

import cv2
import numpy as np

# Thermal Image annotation
from thermal_base import ThermalImageAnnotation
# Manipulation of the image
from thermal_base import utils

from PIL import Image
from PIL.ExifTags import TAGS
import piexif
import piexif.helper

import json

# Functions
# to the temperature in a single point over the image given the coordinates of the point


def show_temp(x, y, thermal_np, temp_image):

    # print the value over the image showed with openCV
    # cv2.putText(temp_image, str(thermal_np[x,y]), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.circle(temp_image, (x, y), 2, (0, 0, 0), -1)
    cv2.putText(temp_image, " {:3.2f} C".format(
        thermal_np[y, x]), (x - 80, y - 15), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 2)
    print("The temperature at point ({},{}) is {:.2f} º C".format(
        x, y, thermal_np[y, x]))

    return


def find_max_temp_dji(temp_image, thermal_np):
    # Find the maximum value in the array
    max_temp = np.max(thermal_np)

    # Find the maximum value indexes
    max_idx = np.argwhere(thermal_np == max_temp)

    print("Maximum temperature:", max_temp)
    print("Indexes:", max_idx[0])
    print(type(max_idx[0]))
    # show_temp(max_idx[0][1],max_idx[0][0],temp_image)
    cv2.circle(temp_image, (max_idx[0][1], max_idx[0][0]), 2, (0, 0, 0), -1)
    cv2.putText(temp_image, "Max: {:.2f} C".format(thermal_np[max_idx[0][0], max_idx[0][1]]), (
        max_idx[0][1] - 80, max_idx[0][0] - 15), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)


def process_dji_img(path_to_image, outputfile):

    image = ThermalImage(image_path=path_to_image, camera_manufacturer="dji")
    thermal_np = image.thermal_np           # The temperature matrix as a np array
    # The raw thermal sensor excitation values as a np array
    raw_sensor_np = image.raw_sensor_np
    meta = image.meta                       # Any other metadata that exiftool picked up
    # Returns: np.ndarray: A colourbar of the required height with temperature values labelled
    colorbar = image.generate_colorbar(cmap=cv2.COLORMAP_HOT)

    print(thermal_np)
    print("the shape of the thermal image is:", thermal_np.shape)

    img_ann = ThermalImageAnnotation()

    # print(dir(utils))                                   # View manipulation tools available
    # thermal_np = utils.change_emissivity_for_roi(...)   # Sample: Change the emissivity of an RoI

    temp_image = utils.get_temp_image(thermal_np, colormap=cv2.COLORMAP_HOT)
    print("Thermal image as numpy image:", type(temp_image))
    # As we have the thermal image and a matrix of the same size with the temperature values, we can now do some image processing
    # show_temp(326,198,thermal_np, temp_image) # temperature of my head
    # show_temp(344,244,thermal_np, temp_image) # temperature of the floor
    # show_temp(141,78,thermal_np, temp_image) # temperature of the RTK

    find_max_temp_dji(temp_image, thermal_np)

    concat = cv2.hconcat([temp_image, colorbar])
    # Saving the image
    cv2.imwrite(outputfile, concat)
    return outputfile, thermal_np


def tiff_to_temperature(tiff_path):
    # Open the TIFF image using Pillow
    img = Image.open(tiff_path)

    # Ensure the image is in 'I;16' mode, which is 16-bit grayscale
    if img.mode != 'I;16':
        raise ValueError(f"Image mode must be 'I;16', but is {img.mode}")

    # Convert the image to a numpy array
    img_array = np.array(img)

    # Apply the formula to convert pixel values to temperatures
    temperatures = img_array/40 - 100

    return img_array, temperatures


def find_max_temp_wirispro(img_array, temperatures, outputfile):
    # Find the maximum temperature and its location
    max_temp = np.max(temperatures)
    max_idx = np.argwhere(temperatures == max_temp)

    print("Maximum temperature:", max_temp)
    print("Indexes:", max_idx[0])
    print(type(max_idx[0]))
    # show_temp(max_idx[0][1],max_idx[0][0],temp_image)
    # Convert the 16-bit grayscale image to 8-bit for OpenCV compatibility
    img_8bit = cv2.normalize(img_array, None, 0, 255,
                             cv2.NORM_MINMAX).astype(np.uint8)

    # Convert the 8-bit image to a 3-channel BGR image
    img_color = cv2.cvtColor(img_8bit, cv2.COLOR_GRAY2BGR)

    cv2.circle(img_color, (max_idx[0][1], max_idx[0][0]), 2, (0, 0, 0), -1)
    cv2.putText(img_color, "Max: {:.2f} C".format(temperatures[max_idx[0][0], max_idx[0][1]]), (
        max_idx[0][1] - 80, max_idx[0][0] - 15), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)
    # cv2.imshow('Max temp marked image',img_color) #Just for debugging purposes
    # write the image to the outputfile

    img_color = cv2.applyColorMap(img_color, cv2.COLORMAP_INFERNO)
    cv2.imwrite(outputfile, img_color)

    return outputfile, temperatures


def process_wirispro_img(tiff_path, outputfile_path):

    img_array, temperatures = tiff_to_temperature(tiff_path)

    return find_max_temp_wirispro(img_array, temperatures, outputfile_path)


def exif_dict_to_bytes(exif_dict):
    exif_bytes = io.BytesIO()
    exif_dict_new = {Image.ExifTags.TAGS[k]: v for k, v in exif_dict.items()}
    exif_dict_new = {k: v.encode() if isinstance(
        v, str) else v for k, v in exif_dict_new.items()}
    exif_dict_new = {k: v if isinstance(v, bytes) else bytes(
        str(v), 'utf-8') for k, v in exif_dict_new.items()}
    exif_bytes.write(exif_dict_new)
    return exif_bytes.getvalue()


def main(argv):

    # CLI arguments parsing
    inputfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["source=", "dest="])
    except getopt.GetoptError:
        print('processThermalImg.py -i <inputfile-path> -o <outputfile-path>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('processThermalImg.py -i <inputfile-path> -o <outputfile-path>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    print('Input file is ', inputfile)
    print('Output file is ', outputfile)

    # Decoding the thermal image
    # ex: /home/ubuntugrvc/thermal_cameras/examples/H20T/DJI_202307211037_002/DJI_20230721104850_0004_T.JPG
    path_to_image = inputfile

    if inputfile.split(".")[1] == "tiff":
        print('Processing a wirispro tiff image')
        outputfile, thermal_np = process_wirispro_img(inputfile, outputfile)
    elif inputfile.split(".")[1] == "JPG" or inputfile.split(".")[1] == "jpg":
        print('Processing a DJI JPG image')
        outputfile, thermal_np = process_dji_img(inputfile, outputfile)
    else:
        print('NON VALID FILE FORMAT')
        return 0

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
