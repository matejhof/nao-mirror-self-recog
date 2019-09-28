import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import csv
import sys
import os
import torch
import torch.nn as nn

torch.manual_seed(1)

# Get the path to where this file locates
path2current = sys.argv[0]
pathname = os.path.dirname(path2current)

# Load input data
with open(pathname + '/centroid_samples.csv') as csv_file:
    reader = csv.reader(csv_file, delimiter=',')
    tix=[]
    tiy=[]
    for row in reader:
        x=row[0]
        tix=np.append(tix,x)
        y=row[1]
        tiy=np.append(tiy,y)
training_input=[tix,tiy]

# Load output data
with open(pathname + '/arm_samples.csv') as csv_file:
    reader = csv.reader(csv_file, delimiter=',')
    tix=[]
    tiy=[]
    tox=[]
    toy=[]
    ti=[]
    to=[]
    for row in reader:
        x=row[0]
        tix=np.append(tix,x)
        y=row[1]
        tiy=np.append(tiy,y)
        bjx=row[2]
        tox=np.append(tox,bjx)
        bjy=row[3]
        toy=np.append(toy,bjy)
        aaa=row[4]
        ti=np.append(ti,aaa)
training_output=[tix, tiy, tox, toy, ti]
training_output= np.array(training_output,dtype=np.float)
training_input= np.array(training_input,dtype=np.float)

training_output= torch.from_numpy(training_output.T).float()
training_input= torch.from_numpy(training_input.T).float()

# Normalization
train_input_max, _ = torch.max(training_input, 0)
train_input_min, _ = torch.min(training_input, 0)
training_input = (training_input - train_input_min.reshape(1, -1)) / (train_input_max.reshape(1, -1) - train_input_min.reshape(1, -1))

train_output_max, _ = torch.max(training_output, 0)
train_output_min, _ = torch.min(training_output, 0)
training_output = (training_output - train_output_min.reshape(1, -1)) / (train_output_max.reshape(1, -1) - train_output_min.reshape(1, -1))

test_input = torch.tensor(([200, 99.0]), dtype=torch.float)
test_input = (test_input - train_input_min.reshape(1, -1)) / (train_input_max.reshape(1, -1) - train_input_min.reshape(1, -1))

n_in, n_h, n_out, batch_size = 2, 10, 5, 10

model = nn.Sequential(nn.Linear(n_in, n_h),
                     nn.Sigmoid(),
                     nn.Linear(n_h, n_out),
                     nn.Sigmoid())

criterion = torch.nn.MSELoss()

optimizer = torch.optim.Adam(model.parameters(), lr=0.01)

error_plot = []
for epoch in range(10000):
    # Forward Propagation
    y_pred = model(training_input)    # Compute and print loss
    loss = criterion(y_pred, training_output)
    print('epoch: ', epoch,' loss: ', loss.item())    # Zero the gradients
    optimizer.zero_grad()
    
    # perform a backward pass (backpropagation)
    loss.backward()
    
    # Update the parameters
    optimizer.step()

    error_plot.append(loss.item())


joint_angle_prediction = model(test_input)

# Denormalization
joint_angle_prediction = (train_output_max.reshape(1, -1) - train_output_min.reshape(1, -1)) * joint_angle_prediction.reshape(1, -1) + train_output_min.reshape(1, -1)
print(joint_angle_prediction[0, 0], joint_angle_prediction[0, 1], joint_angle_prediction[0, 2], joint_angle_prediction[0, 3], joint_angle_prediction[0, 4])

fig = plt.figure()
ax = fig.add_subplot(111)
plt.plot(np.arange(1, len(error_plot)+1), error_plot)
plt.title('Training Error')
plt.ylabel('Error')
plt.xlabel('Training Iteration')
plt.show()

# # ---------Save Model----------
# torch.save(model.state_dict(), pathname + '/Trained_Networks/touching')
