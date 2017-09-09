
import os, sys, time
import numpy as np
from time import sleep

fifo = open('/var/run/mlx9062x.sock', 'r')

def personDetector(array):
	for j in range(0,array.shape[1]):
		if np.mean(array[j,:]) > 31:
			print "someone in the car"

def readdata():



while True:
    sleep(1)        
    ir_raw = fifo.read()
    ir_trimmed = ir_raw[0:128]
    ir = np.frombuffer(ir_trimmed, np.uint16)
    print(ir.size)
    ir = np.reshape(ir,(16,4))
    ir = ir/100
    ir = ir - 273
    personDetector(ir)	
    print(ir.shape[1])
    print (ir)

fifo.close()
