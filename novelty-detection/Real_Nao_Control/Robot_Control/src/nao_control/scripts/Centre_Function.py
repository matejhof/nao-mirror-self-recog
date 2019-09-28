import torch
import torch.nn as nn
import torch.utils.data as Data
from torch.utils.data.sampler import SubsetRandomSampler
# import torchvision
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import os
import seaborn as sns
import sys
import cv2
from skimage.measure import compare_ssim
import imutils

from PIL import Image
import glob
from resizeimage import resizeimage
from scipy import misc, ndimage, signal
from scipy.ndimage import gaussian_filter

def overlappingRectangle(x, y, w1, h1, post_it_x, post_it_y):
    area_detect = w1 * h1
    area_postit = 14 * 14
    if (x > post_it_x + 14 or post_it_x > x + w1) or (y > post_it_y + 14 or post_it_y > y + h1):
        area_intersection = 0
    else:
        area_intersection = (min(x + w1, post_it_x + 14) - max(x, post_it_x)) * (min(y + h1, post_it_y + 14) - max(y, post_it_y))

    # return area_intersection / (area_detect + area_postit - area_intersection)
    # return area_intersection / (area_detect)
    return area_intersection / (area_postit)


def calculate_muandsigma_autoencoder(image_list, train_indices, autoencoder, w, h):
    error_list = []
    error_sum = np.zeros_like(image_list[0])
    for i in range(len(train_indices)):                    # For calculating the mu
        original_img = np.reshape(image_list[train_indices[i]], (-1, w*h))
        _, output_img = autoencoder(torch.from_numpy(original_img).float())
        error = np.reshape(abs(255*(original_img - output_img.data.numpy().astype("float64"))), (h, w))
        error_sum = error_sum + error
        error_list.append(error)

    mu = error_sum / len(train_indices)

    variance_sum = np.zeros_like(image_list[0])
    for i in range(len(train_indices)):                    # For calculating the standard deviation
        variance_sum = variance_sum + np.power((error_list[i] - mu), 2)

    s = np.sqrt(variance_sum / (len(train_indices) - 1))

    return mu, s

def calculate_muandsigma_decoder(image_list, train_indices, joint_angle, decoder, w, h):
    error_list = []
    error_sum = np.zeros_like(image_list[0])
    for i in range(len(train_indices)):                    # For calculating the mu
        original_img = np.reshape(image_list[train_indices[i]], (-1, w*h))
        output_img = decoder(torch.from_numpy(joint_angle[train_indices[i], :]).float())
        error = np.reshape(abs(255*(original_img - output_img.data.numpy().astype("float64"))), (h, w))
        error_sum = error_sum + error
        error_list.append(error)

    mu = error_sum / len(train_indices)

    variance_sum = np.zeros_like(image_list[0])
    for i in range(len(train_indices)):                    # For calculating the standard deviation
        variance_sum = variance_sum + np.power((error_list[i] - mu), 2)

    s = np.sqrt(variance_sum / (len(train_indices) - 1))

    return mu, s

def autoencoder_accuracy(mu, s, post_it_x, post_it_y, image_list, test_indices, autoencoder, w, h, filter_size, novelty_thres, filter_option):
    accuracy_list = []
    for i in range(len(test_indices)):

        test_data = np.reshape(image_list[test_indices[i]], (-1, w*h))

        _, decoded_test_data = autoencoder(torch.from_numpy(test_data).float())

        diff = np.true_divide((abs(np.reshape(abs(255*(test_data - decoded_test_data.data.numpy().astype("float64"))), (h, w)) - mu)), (s + 0.00001))

        if filter_option:
            diff = gaussian_filter(diff, sigma=filter_size)            # Use Gaussian Filter
        else:
            pass

        thresh = cv2.threshold(diff, 255*novelty_thres, 255,
            cv2.THRESH_BINARY)[1]
        thresh = thresh.astype("uint8")
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        area=[]
        for j in range(len(cnts)):
            area.append(cv2.contourArea(cnts[j]))

        if len(area)>0:
            max_idx = np.argmax(area)
            if area[max_idx] < 30 or area == []:
                accuracy = 0

            else:
                (x, y, w1, h1) = cv2.boundingRect(cnts[max_idx])
                accuracy = overlappingRectangle(x, y, w1, h1, post_it_x[i], post_it_y[i])
        else:
            accuracy = 0

        accuracy_list.append(accuracy)

    return accuracy_list, np.sum(accuracy_list) / (len(accuracy_list))

def decoder_accuracy(mu, s, post_it_x, post_it_y, image_list, test_indices, joint_angle, decoder, w, h, filter_size, novelty_thres, filter_option):
    accuracy_list = []
    for i in range(len(test_indices)):

        test_data = np.reshape(image_list[test_indices[i]], (-1, w*h))

        decoded_test_data = decoder(torch.from_numpy(joint_angle[test_indices[i], :]).float())

        diff = np.true_divide((abs(np.reshape(abs(255*(test_data - decoded_test_data.data.numpy().astype("float64"))), (h, w)) - mu)), (s + 0.00001))

        if filter_option:
            diff = gaussian_filter(diff, sigma=filter_size)            # Use Gaussian Filter
        else:
            pass

        thresh = cv2.threshold(diff, 255*novelty_thres, 255,
            cv2.THRESH_BINARY)[1]
        thresh = thresh.astype("uint8")
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        area=[]
        for j in range(len(cnts)):
            area.append(cv2.contourArea(cnts[j]))

        if len(area)>0:
            max_idx = np.argmax(area)
            if area[max_idx] < 30 or area == []:
                accuracy = 0

            else:
                (x, y, w1, h1) = cv2.boundingRect(cnts[max_idx])
                accuracy = overlappingRectangle(x, y, w1, h1, post_it_x[i], post_it_y[i])
        else:
            accuracy = 0

        accuracy_list.append(accuracy)

    return accuracy_list, np.sum(accuracy_list) / (len(accuracy_list))
