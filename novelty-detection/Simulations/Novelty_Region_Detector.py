import torch
import torch.nn as nn
import torch.utils.data as Data
# import torchvision
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import os
import seaborn as sns
import sys

from PIL import Image
import glob
from resizeimage import resizeimage
from scipy import misc, ndimage
from scipy.ndimage import gaussian_filter

# Hyperparameter
EPOCH = 10
BATCH_SIZE = 128
LR = 0.005         # learning rate
N_TEST_IMG = 3
novelty_thres = 0.3
# filter_name = 'Gaussian'            # Choose one filter to refine the detected novelty
# filter_name = 'Mean'              # Choose one filter to refine the detected novelty
filter_name = 'Median'            # Choose one filter to refine the detected novelty
filter_size = 3                     # Select the filter size

image_list = []
for filename in sorted(glob.glob('Simulated_Nao/*.png'), key=os.path.getmtime): #assuming png
    im=Image.open(filename).convert('LA')
    im=im.crop((251,161,388,264))
    im=im.resize((63, 47))
    img = np.array(list(im.getdata(band=0)), float)/255
    img.shape = (im.size[1], im.size[0])
    image_list.append(img)

plt.imshow(image_list[3], cmap = "gray")
plt.show()

# Load data for mini batch. The batch images sizes are (BATCH_SIZE, 47, 63)
train_loader = Data.DataLoader(dataset=image_list, batch_size=BATCH_SIZE, shuffle=True)

class AutoEncoder(nn.Module):
    def __init__(self):
        super(AutoEncoder, self).__init__()

        self.encoder = nn.Sequential(
            nn.Linear(63*47, 128),
            nn.Tanh(),
            nn.Linear(128, 64),
            nn.Tanh(),
            nn.Linear(64, 12),
            nn.Tanh(),
            nn.Linear(12, 2),   # compress to 2 features which can be visualized in plt
        )
        self.decoder = nn.Sequential(
            nn.Linear(2, 12),
            nn.Tanh(),
            nn.Linear(12, 64),
            nn.Tanh(),
            nn.Linear(64, 128),
            nn.Tanh(),
            nn.Linear(128, 63*47),
            nn.Sigmoid(),       # compress to a range (0, 1)
        )

    def forward(self, x):
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return encoded, decoded


autoencoder = AutoEncoder()

optimizer = torch.optim.Adam(autoencoder.parameters(), lr=LR)
loss_func = nn.MSELoss()

# initialize figure
f, a = plt.subplots(2, N_TEST_IMG, figsize=(N_TEST_IMG, 2))
plt.ion()   # continuously plot

view_data = np.reshape(image_list[:N_TEST_IMG], (-1, 63*47))
for i in range(N_TEST_IMG):
    a[0, i].imshow(np.reshape(view_data[i, :], (47, 63)), cmap='gray'); a[0,i].set_xticks(()); a[0,i].set_yticks(())

for epoch in range(EPOCH):
    for step, (x) in enumerate(train_loader):
        b_x = x.view(-1, 63*47)   # batch x, shape (batch, 63*47)
        b_y = x.view(-1, 63*47)   # batch y, shape (batch, 63*47)

        encoded, decoded = autoencoder(b_x.float())
        
        loss = loss_func(decoded, b_y.float())      # mean square error
        optimizer.zero_grad()               # clear gradients for this training step
        loss.backward()                     # backpropagation, compute gradients
        optimizer.step()                    # apply gradients

        if step % 100 == 0:
            print('Epoch: ', epoch, '| train loss: %.4f' % loss.data.numpy())

            _, decoded_data = autoencoder(torch.from_numpy(view_data).float())
            for i in range(N_TEST_IMG):
                a[1,i].clear()
                a[1,i].imshow(np.reshape(decoded_data[i, :].data.numpy(), (47, 63)), cmap='gray')
                a[1,i].set_xticks(()); a[1,i].set_yticks(())
            plt.draw(); plt.pause(1)

plt.ioff()
plt.show()





image_test_list = []
for filename_test in sorted(glob.glob('*.png'), key=os.path.getmtime): #assuming png
    im_test=Image.open(filename_test).convert('LA')
    im_test=im_test.crop((251,161,388,264))
    im_test=im_test.resize((63, 47))
    img_test = np.array(list(im_test.getdata(band=0)), float)/255
    img_test.shape = (im_test.size[1], im_test.size[0])
    # img = torch.from_numpy(img)
    # im = np.array(im)
    image_test_list.append(img_test)

# print(image_list[2].shape)
plt.imshow(image_test_list[0], cmap = "gray")
# plt.title('%i' % train_data.train_labels[2])
plt.show()

test_data = np.reshape(image_test_list[:1], (-1, 63*47))

_, decoded_test_data = autoencoder(torch.from_numpy(test_data).float())

plt.imshow(np.reshape(decoded_test_data[0, :].data.numpy(), (47, 63)), cmap = "gray")
plt.show()

# Plot the novelty region
diff_image = image_test_list[0] - np.reshape(decoded_test_data[0, :].data.numpy(), (47, 63))

if filter_name == 'Gaussian':
    diff_image = gaussian_filter(diff_image, sigma=filter_size)            # Use Gaussian Filter
elif filter_name == 'Mean':
    diff_image = ndimage.convolve(diff_image, (np.ones((filter_size, filter_size))/(filter_size * filter_size)))            # Use Gaussian Filter
elif filter_name == 'Median':
    diff_image = ndimage.median_filter(diff_image, size=filter_size)            # Use Gaussian Filter
else:
    sys.exit("Filter type is wrong")

nov_matrix = np.zeros_like(diff_image)
nov_matrix[diff_image > novelty_thres] = 1
centre_y, centre_x = np.where(nov_matrix==1)                                # Get the position of novelty


# x, y = np.mgrid[0:diff_image.shape[0], 0:diff_image.shape[1]]
# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot_surface(x, y, diff_image)
# plt.show()

mask=np.zeros_like(nov_matrix)
mask[nov_matrix==0] = True
with sns.axes_style("white"):
    ax = sns.heatmap(nov_matrix, mask=mask, cmap='coolwarm', cbar=False, xticklabels=False, yticklabels=False)

ax.collections[0].set_alpha(0.5)
plt.imshow(image_test_list[0], cmap = "gray")
plt.plot(np.ceil((max(centre_x) + min(centre_x)) / 2), np.ceil((max(centre_y) + min(centre_y)) / 2), marker='o', markersize=3, color="red")
plt.show()