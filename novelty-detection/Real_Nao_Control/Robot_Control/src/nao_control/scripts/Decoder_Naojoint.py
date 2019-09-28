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
import pickle
import csv
from Centre_Function import overlappingRectangle, calculate_muandsigma_decoder, decoder_accuracy

from PIL import Image
import glob
from resizeimage import resizeimage
from scipy import misc, ndimage, signal
from scipy.ndimage import gaussian_filter

# Get the path to where this file locates
path2current = sys.argv[0]
pathname = os.path.dirname(path2current)

# Hyperparameter
EPOCH = 50
BATCH_SIZE = 128
LR = 0.005         # learning rate
test_split = .2
shuffle_dataset = True
random_seed= 1
N_TEST_IMG = 3
novelty_thres = 0.018               # 0.018 is ok
filter_name = 'Gaussian'            # Choose one filter to refine the detected novelty
# filter_name = 'Mean'              # Choose one filter to refine the detected novelty
# filter_name = 'Median'            # Choose one filter to refine the detected novelty
filter_size = 2                     # Select the filter size
methods = 'cv2.TM_CCOEFF'           # For method you can choose one of ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR','cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
# colorspace = 'hsv'                  # The other option is 'grayscale'
colorspace = 'grayscale'

# Get the head filter
head_filter_name = glob.glob(pathname + '/Head_Filter/*.png')
head_filter_im = cv2.imread(head_filter_name[0])
head_filter = cv2.cvtColor(head_filter_im, cv2.COLOR_BGR2GRAY)
plt.imshow(head_filter, cmap = "gray")
plt.show()

w, h = head_filter.shape[::-1]

class Decoder(nn.Module):
    def __init__(self):
        super(Decoder, self).__init__()

        self.decoder = nn.Sequential(
            nn.Linear(2, 12),
            nn.Tanh(),
            nn.Linear(12, 64),
            nn.Tanh(),
            nn.Linear(64, 128),
            nn.Tanh(),
            nn.Linear(128, w*h),
            nn.Sigmoid(),       # compress to a range (0, 1)
        )

    def forward(self, x):
        decoded = self.decoder(x)
        return decoded


# Define a function to find the order
def array_match(training_data, batch_data, image_list):
    order = np.zeros((batch_data.shape[0], 1))
    corresp_img = np.zeros((batch_data.shape[0], w*h))
    for index_in_b in range(batch_data.shape[0]):

        for index_in_t in range(training_data.shape[0]):
            if np.all(training_data[index_in_t, :] == batch_data[index_in_b, :]):
                order[index_in_b] = index_in_t
                break
    
        corresp_img[index_in_b, :] = np.reshape(image_list[index_in_t], (-1, w*h))
    
    return corresp_img

# # --------------- Needed for training ------------------

image_list = []
for filename in sorted(glob.glob(pathname + '/Nao_Mirror_Training_Dataset/*.png'), key=os.path.getmtime): #assuming png
    im=cv2.imread(filename)
    im_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    method = eval(methods)
    res = cv2.matchTemplate(im_gray, head_filter, method)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    top_left = max_loc

    cropped_img = im_gray[top_left[1]:top_left[1] + h, top_left[0]: top_left[0] + w].copy()         # For greyscale

    image_list.append(cropped_img/255)            # For grayscale

# Load training head joint angles
with open(pathname + '/head_joint_real.csv') as csv_file:            # Read the csv file
    reader = csv.reader(csv_file, delimiter=',')
    tix=[]
    tiy=[]
    for row in reader:
        x=row[0]
        tix=np.append(tix,x)
        y=row[1]
        tiy=np.append(tiy,y)
training_input=[tix,tiy]
training_input= np.array(training_input,dtype=np.float)
joint_angle = training_input.T

# Creating data indices for training and test splits:
dataset_size = len(image_list)
indices = list(range(dataset_size))
split = int(np.floor(test_split * dataset_size))
if shuffle_dataset :
    np.random.seed(random_seed)
    np.random.shuffle(indices)
train_indices, test_indices = indices[split:], indices[:split]

# Creating PT data samplers and loaders:
train_sampler = SubsetRandomSampler(train_indices)
test_sampler = SubsetRandomSampler(test_indices)

# Draw the simulation post-it with the colors yellow, pink and green randomly on the test set
post_it_x = []
post_it_y = []
for draw_index in range(split):
    np.random.seed(draw_index)
    random_position_x = np.random.randint(0, w - 13)
    random_position_y = np.random.randint(0, h - 13)
    color_index = np.random.randint(1,4)
    if color_index == 1:
        random_color = 226          # Yellow
    elif color_index == 2:
        random_color = 185          # Pink
    else:
        random_color = 149          # Green

    cv2.rectangle(image_list[test_indices[draw_index]], (random_position_x, random_position_y), (random_position_x + 14, random_position_y + 14), random_color/255, thickness=cv2.FILLED)
    post_it_x.append(random_position_x)
    post_it_y.append(random_position_y)

# Load data for mini batch. The batch images sizes are (BATCH_SIZE, h, w)
train_loader = Data.DataLoader(dataset=joint_angle, batch_size=BATCH_SIZE, sampler=train_sampler)
# train_loader = Data.DataLoader(dataset=image_list, batch_size=BATCH_SIZE, sampler=train_sampler)
test_loader = Data.DataLoader(dataset=joint_angle, batch_size=split, sampler=test_sampler)

torch.manual_seed(1)            # Random seed, make sure each time can get the same results
decoder = Decoder()

optimizer = torch.optim.Adam(decoder.parameters(), lr=LR)
loss_func = nn.MSELoss()

# initialize figure
f, a = plt.subplots(1, N_TEST_IMG, figsize=(N_TEST_IMG, 1))
plt.ion()   # continuously plot

# original data (first row) for viewing
# view_data[0, :] = image_list[0].view(-1, 63*47)
# view_data[1, :] = image_list[3].view(-1, 63*47)
# print(np.reshape(image_list[:2], (-1, 63*47)).shape)
# view_data = np.reshape(image_list[:2], (-1, 63*47)).type(torch.FloatTensor)
view_data = np.reshape(joint_angle[:N_TEST_IMG], (-1, 2))          # when you want to see more than one reconstruction results of network, then use this line. Otherwise, comment this line, and use the commented 4 lines above.

scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=10, gamma=0.5)

loss_list_training = []
loss_list_test = []
accuracy_with_filter_list = []
accuracy_no_filter_list = []
for epoch in range(EPOCH):
    # For training
    for step, (x) in enumerate(train_loader):
        b_x = x.view(-1, 2)   # batch x, shape (batch, 63*47)
        b_xx = b_x.numpy()
        corresp_img = array_match(joint_angle, b_xx, image_list)

        decoded = decoder(b_x.float())
        
        loss = loss_func(decoded, torch.from_numpy(corresp_img).float())      # mean square error
        optimizer.zero_grad()               # clear gradients for this training step
        loss.backward()                     # backpropagation, compute gradients
        optimizer.step()                    # apply gradients

        if step % 100 == 0:
            print('Epoch: ', epoch, '| train loss: %.4f' % loss.data.numpy())

            # plotting decoded image (second row)
            decoded_data = decoder(torch.from_numpy(view_data).float())
            for i in range(N_TEST_IMG):
                a[i].clear()
                a[i].imshow(np.reshape(decoded_data[i, :].data.numpy(), (h, w)), cmap='gray')
                a[i].set_xticks(()); a[i].set_yticks(())
            plt.draw(); plt.pause(1)

    loss_list_training.append(loss.data.numpy().astype('float64'))
    scheduler.step()

    # For testing
    for _, (x_t) in enumerate(test_loader):
        b_x_t = x_t.view(-1, 2)   # batch x, shape (batch, w*h)
        b_xx_t = b_x_t.numpy()
        corresp_img_t = array_match(joint_angle, b_xx_t, image_list)

        decoded_t = decoder(b_x_t.float())

        loss_t = loss_func(decoded_t, torch.from_numpy(corresp_img_t).float())      # mean square error

        loss_list_test.append(loss_t.data.numpy().astype('float64'))

    mu, s = calculate_muandsigma_decoder(image_list, train_indices, joint_angle, decoder, w, h)

    a1, accuracy_with_filter = decoder_accuracy(mu, s, post_it_x, post_it_y, image_list, test_indices, joint_angle, decoder, w, h, filter_size, novelty_thres, True)
    a2, accuracy_no_filter = decoder_accuracy(mu, s, post_it_x, post_it_y, image_list, test_indices, joint_angle, decoder, w, h, filter_size, novelty_thres, False)
    accuracy_with_filter_list.append(accuracy_with_filter)
    accuracy_no_filter_list.append(accuracy_no_filter)

plt.ioff()
plt.show()

# Plot the loss and accuracy
plt.plot(np.arange(1, len(loss_list_training)+1), loss_list_training, color='blue', label='Training_Loss')
plt.plot(np.arange(1, len(loss_list_test)+1), loss_list_test, color='orange', label='Test_Loss')
plt.legend()
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.title('Comparison of Training Loss and Test Loss')
plt.show()
plt.plot(np.arange(1, len(accuracy_with_filter_list)+1), accuracy_with_filter_list, color='blue', label='Accuracy with Filtering')
plt.plot(np.arange(1, len(accuracy_no_filter_list)+1), accuracy_no_filter_list, color='orange', label='Accuracy without Filtering')
plt.legend()
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.title('Comparison of Accuracy between with and without Filtering (divided by post-it area)')
plt.show()

# Save the network and parameters
# torch.save(decoder.state_dict(), pathname + '/Trained_Networks/trained_decoder_' + colorspace)
# np.save(pathname + '/Variance_Parameters/mu_for_decoder', mu)            # Save the mu
# np.save(pathname + '/Variance_Parameters/sigma_for_decoder', s)          # Save the sigma
# with open(pathname + '/Plot_Dataset/accuracy_with_filter_for_decoder(train)_divideonlypostit.txt', "wb") as fp1:   #Pickling
#     pickle.dump(accuracy_with_filter_list, fp1)
# with open(pathname + '/Plot_Dataset/accuracy_no_filter_for_decoder(train)_divideonlypostit.txt', "wb") as fp2:   #Pickling
#     pickle.dump(accuracy_no_filter_list, fp2)

# with open(pathname + '/Plot_Dataset/samples_accuracy_with_filter_for_decoder_divideonlypostit.txt', "wb") as fp3:   #Pickling
#     pickle.dump(a1, fp3)
# with open(pathname + '/Plot_Dataset/samples_accuracy_no_filter_for_decoder_divideonlypostit.txt', "wb") as fp4:   #Pickling
#     pickle.dump(a2, fp4)

# with open(pathname + '/accuracy_for_autoencoder_no_q.txt', 'rb') as f:
#    mylist = pickle.load(f)
# # -------------------- Training part until here -----------------



# # -------------------- Novelty detection part starts here -----------------
# decoder = Decoder()
# decoder.load_state_dict(torch.load(pathname + '/Trained_Networks/trained_decoder_' + colorspace))
# decoder.eval()

# mu = np.load(pathname + '/Variance_Parameters/mu_for_decoder.npy')       # Load mu
# s = np.load(pathname + '/Variance_Parameters/sigma_for_decoder.npy')       # Load s

# # Load the test data and plot the Mahalanobis distance

# image_test_list = []
# for filename_test in sorted(glob.glob(pathname + '/*.png'), key=os.path.getmtime): #assuming png
#     im_test=cv2.imread(filename_test)
#     im_test_hsv = cv2.cvtColor(im_test, cv2.COLOR_BGR2HSV)
#     im_test_gray = cv2.cvtColor(im_test, cv2.COLOR_BGR2GRAY)
#     method = eval(methods)
#     res_test = cv2.matchTemplate(im_test_gray, head_filter, method)
#     min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res_test)

#     top_left = max_loc

#     # cropped_img_test = im_test_hsv[top_left[1]:top_left[1] + h, top_left[0]: top_left[0] + w, 1].copy()     # For hsv
#     cropped_img_test = im_test_gray[top_left[1]:top_left[1] + h, top_left[0]: top_left[0] + w].copy()          # For grayscale
#     image_test_list.append(cropped_img_test/255)          # For grayscale
#     # image_test_list.append(cropped_img_test / 180)          # For hsv

# test_data = np.array([[0.0779905144,0.0801204423]])
# # test_data = np.array([[0.0,0.0]])
# test_data_img = np.reshape(image_test_list[:1], (-1, w*h))

# decoded_test_data = decoder(torch.from_numpy(test_data).float())
	
# diff = np.true_divide((abs(np.reshape(abs(255*(test_data_img - decoded_test_data.data.numpy().astype("float64"))), (h, w)) - mu)), (s + 0.00001))
# # diff = (diff * 180).astype("uint8")                         # hsv
# # diff = gaussian_filter(diff, sigma=filter_size)            # Use Gaussian Filter

# # diff = ndimage.convolve(diff, (np.ones((filter_size, filter_size))/(filter_size * filter_size)))            # Use Mean Filter
# # diff = ndimage.median_filter(diff, size=filter_size)            # Use Median Filter
	
# # threshold the difference image, followed by finding contours to
# # obtain the regions of the two input images that differ
# thresh = cv2.threshold(diff, 255*novelty_thres, 255,
# 	cv2.THRESH_BINARY)[1]
# thresh = thresh.astype("uint8")
# cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
# 	cv2.CHAIN_APPROX_SIMPLE)
# cnts = imutils.grab_contours(cnts)

# area=[]
# for i in range(len(cnts)):
#     area.append(cv2.contourArea(cnts[i]))

# if len(area)>0:
#     max_idx = np.argmax(area)
#     if area[max_idx] < 30 or area == []:
#         x=0
#         y=0

#     else:
#         (x, y, w1, h1) = cv2.boundingRect(cnts[max_idx])
#         cv2.rectangle(image_test_list[0], (x, y), (x + w1, y + h1), (0, 0, 255), 2)
#         cv2.rectangle(np.reshape(decoded_test_data[0, :].data.numpy(), (h, w)), (x, y), (x + w1, y + h1), (0, 0, 255), 2)
# else:
#     x=0
#     y=0

# # show the output images
# fig, axs = plt.subplots(2, 2)
# axs[0, 0].imshow(image_test_list[0], cmap='gray')
# axs[0, 0].set_title('Original')
# axs[0, 0].axis('off')
# axs[0, 1].imshow(np.reshape(decoded_test_data[0, :].data.numpy(), (h, w)), cmap='gray')
# axs[0, 1].set_title('Output from Decoder')
# axs[0, 1].axis('off')
# axs[1, 0].imshow(diff, cmap='gray')
# axs[1, 0].set_title('Diff')
# axs[1, 0].axis('off')
# axs[1, 1].imshow(thresh, cmap='gray')
# axs[1, 1].set_title('Thresh')
# axs[1, 1].axis('off')
# plt.show()
# # -------------------- Novelty detection part ends here -----------------