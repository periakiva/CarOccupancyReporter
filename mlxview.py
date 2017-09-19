#!/usr/bin/env python

import time
import picamera
import numpy as np
import subprocess
import os, sys
import datetime
import skimage
from twilio import twiml
from twilio.rest import Client
from skimage import io, exposure, transform, img_as_float, img_as_ubyte
from time import sleep

import matplotlib
import matplotlib.pyplot as plt

# IR registration parameters
ROT = np.deg2rad(90)
SCALE = (36.2, 36.4)
OFFSET = (530, 170)

account_sid = #your accountsid
auth_token = #your auth token
client = Client(account_sid,auth_token)

def personDetection(array):
        sleep(0.25)        
	for j in range(0,array.shape[0]):
		if np.mean(array[j,:])>30:
			client.messages.create(to="INSERT YOUR PHONE NUMBER",from_="+12017293901", body = "Car Occupancy Reporter Here, There is someone in the car!")	
			print "some is in the car"
			return True
	return False

def getImage():
    fn = r'/home/pi/school/mlxd/testing/tmp.jpg';
    proc = subprocess.Popen('raspistill -o %s -w 640 -h 480 -n -t 3' %(fn),
                        shell=True, stderr=subprocess.STDOUT)
    proc.wait()
    im = io.imread(fn, as_grey=True)
    im = exposure.equalize_hist(im)
    return skimage.img_as_ubyte(im) 

im = getImage()

with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 20
    camera.start_preview()
    # get the temperature array, and align with the image
    fifo = open('/var/run/mlx9062x.sock', 'r')
    # get the whole FIFO
    ir_raw = fifo.read()
    # trim to 128 bytes
    ir_trimmed = ir_raw[0:128]
    # go all numpy on it
    ir = np.frombuffer(ir_trimmed, np.uint16)
    # set the array shape to the sensor shape (16x4)
    #ir = ir.reshape((16, 4))[::-1, ::-1]
    ir = ir.reshape(16, 4)
    ir = ir/100
    ir = ir - 273
    print(ir)
    ir = img_as_float(ir) 
    # stretch contrast on our heat map 
    p2, p98 = np.percentile(ir, (2, 98))
    ir = exposure.rescale_intensity(ir, in_range=(p2, p98))
    # increase even further? (optional)
    # ir = exposure.equalize_hist(ir)

    # turn our array into pretty colors
    cmap = plt.get_cmap('spectral')
    rgba_img = cmap(ir)
    rgb_img = np.delete(rgba_img, 3, 2) 
   
    # align the IR array with the camera
    tform = transform.AffineTransform(scale=SCALE, rotation=ROT, translation=OFFSET)
    ir_aligned = transform.warp(rgb_img, tform.inverse, mode='constant', output_shape=im.shape)
    # turn it back into a ubyte so it'll display on the preview overlay
    ir_byte = img_as_ubyte(ir_aligned)
    #add the overlay
    o = camera.add_overlay(np.getbuffer(ir_byte), layer=3, alpha=90)

    #update loop
    while True:
        camera.capture('tmp.jpg')
        sleep(0.25)        
        ir_raw = fifo.read()
        ir_trimmed = ir_raw[0:128]
        ir = np.frombuffer(ir_trimmed, np.uint16)
        ir = ir.reshape(16, 4)
        ir = ir/100
    	ir = ir - 273
	print(ir)
	
	isThere = personDetection(ir)
	if isThere == True:
		break
	ir = img_as_float(ir)  
        p2, p98 = np.percentile(ir, (2, 98))
        ir = exposure.rescale_intensity(ir, in_range=(p2, p98))
        ir = exposure.equalize_hist(ir)
        
        cmap = plt.get_cmap('spectral')
        rgba_img = cmap(ir)
        rgb_img = np.delete(rgba_img, 3, 2)    
        # align the IR array with the image
        tform = transform.AffineTransform(scale=SCALE, rotation=ROT, translation=OFFSET)
        ir_aligned = transform.warp(rgb_img, tform.inverse, mode='constant', output_shape=im.shape)
        ir_byte = img_as_ubyte(ir_aligned)

        o.update(np.getbuffer(ir_byte))

    print('Error! Closing...')
    camera.remove_overlay(o)
    fifo.close()
            


 
