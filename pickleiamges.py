from __future__ import print_function
import random
import hashlib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import sys
import tarfile
from scipy import ndimage,misc
from sklearn.linear_model import LogisticRegression
from six.moves.urllib.request import urlretrieve
import cPickle as pickle
import cv2

data_root = '/home/pi/school/mlxd/training' #your path to training data of images
test_root = '/home/pi/school/mlxd/testing'  #your path to testing data of images

train_folders = [os.path.join(data_root,d) for d in sorted(os.listdir(data_root)) if os.path.isdir(os.path.join(data_root,d))]
test_folders = [os.path.join(test_root,d) for d in sorted(os.listdir(test_root)) if os.path.isdir(os.path.join(test_root,d))]

print(train_folders)
print(test_folders)

image_height = 144
image_width = 256
pixel_depth = 255.0

def load_letter(folder,min_num_images):
    image_files = os.listdir(folder)
    dataset = np.ndarray(shape=(len(image_files),image_height,image_width),dtype=np.float32)
    num_images=0
    for image in image_files:
        image_file = os.path.join(folder,image)
        try:
            image_data = misc.imresize((ndimage.imread(image_file,flatten=True).astype(float) - pixel_depth/2)/pixel_depth, [image_height,image_width])
            print(image_data.shape)
            if image_data.shape != (image_height,image_width):
                print(image_data.shape)
                print(image_height,image_width)
                raise Exception('unexpected image shape: %s' % str(image_data.shape))
            dataset[num_images,:,:] = image_data
            num_images = num_images+1
        except IOError as e:
            print('could not read', image_file, ':',e,'-it\'s ok, skipping')

    dataset = dataset[0:num_images,:,:]
    if num_images<min_num_images:
        raise Exception('many fewer images than expected: %d < %d' %(num_images,min_num_images))
    print('full dataset tensor:', dataset.shape)
    return dataset

def maybe_pickle(data_folders, min_num_images_per_class,force=False):
    dataset_names=[]
    for folder in data_folders:
        set_filename=folder+'.pickle'
        dataset_names.append(set_filename)
        if os.path.exists(set_filename) and not force:
            print('%s already present - skipping pickling' % set_filename)
        else:
            print('pickling %s' % set_filename)
            dataset = load_letter(folder,min_num_images_per_class)
            try:
                with open(set_filename,'wb') as f:
                    pickle.dump(dataset,f,pickle.HIGHEST_PROTOCOL)
            except Exception as e:
                print('unable to save data to', set_filename,':',e)
    return dataset_names
train_datasets = maybe_pickle(train_folders,1)
test_datasets = maybe_pickle(test_folders,1)
print(test_datasets)


