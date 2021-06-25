import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torchsummary import summary
import matplotlib.pyplot as plt
import sys
import os
import cv2 as cv

sys.path.append(os.path.join("/home/jordan/Projects/self-driving-car/"))
from src.localization.test.CarlaDataset import CarlaDataset

if torch.cuda.is_available():
    print("Working with Cuda! :D")
    device = torch.device("cuda")
else:
    print("Working with Cpu :(")
    device = torch.device("cpu")

# Define Model
class VoNet(nn.Module):
    def __init__(self, n_channels, n_images):
        super(VoNet, self).__init__()

        self.conv1 = nn.Conv2d(n_channels*n_images, 64, kernel_size=7, stride=2, padding=3)
        self.bn1 = nn.BatchNorm2d(64)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)
        
        self.conv2 = nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1)
        self.bn2 = nn.BatchNorm2d(64)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1)
        self.bn3 = nn.BatchNorm2d(64)

        self.conv4 = nn.Conv2d(64, 128, kernel_size=5, stride=2, padding=1)
        self.bn4 = nn.BatchNorm2d(128)
        self.conv5 = nn.Conv2d(128, 128, kernel_size=5, stride=2, padding=1)
        self.bn5 = nn.BatchNorm2d(128)

        self.conv6 = nn.Conv2d(128, 256, kernel_size=5, stride=2, padding=1)
        self.bn6 = nn.BatchNorm2d(256)
        self.conv7 = nn.Conv2d(256, 256, kernel_size=5, stride=2, padding=1)
        self.bn7 = nn.BatchNorm2d(256)

        self.relu = nn.ReLU()

        self.fc1 = nn.Linear(22528, 1000)
        self.fc2 = nn.Linear(1000, 100)
        self.fc3 = nn.Linear(100, 50)
        self.fc4 = nn.Linear(50, 9)
    
    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)

        x = self.conv2(x)
        x = self.bn2(x)
        x = self.relu(x)

        x = self.conv3(x)
        x = self.bn3(x)
        x = self.relu(x)

        x = self.conv4(x)
        x = self.bn4(x)
        x = self.relu(x)

        x = self.conv5(x)
        x = self.bn5(x)
        x = self.relu(x)

        x = self.conv6(x)
        x = self.bn6(x)
        x = self.relu(x)

        x = self.conv7(x)
        x = self.bn7(x)
        x = self.relu(x)

        x = x.view(-1, 256*8*11)
        x = self.fc1(x)
        x = self.fc2(x)
        x = self.fc3(x)
        x = self.fc4(x)
        return x

# Load Data
dataset = CarlaDataset("/home/jordan/Datasets/CarlaDatasets/TestDataset01")
dataloader = DataLoader(dataset=dataset, batch_size=4, shuffle=True, num_workers=2)

# Test Model
def test_VoNet():
    IMG_WIDTH = 800         # width of image
    IMG_HEIGHT = 600        # height of image
    IMG_CHANNELS = 3        # number of channels per image
    NUM_IMG_EXAMPLE = 2     # number of images per example
    NUM_EXAMPLES = 5        # number of examples
    
    # show image
    ims, out_state = iter(dataloader).next()
    print(f"ims shape: {ims.shape}")
    print(f"output_state: {out_state}")
    img = torch.zeros(3, 600, 800, dtype=torch.uint8)
    img.copy_(ims[0, 0:3])
    plt.imshow(img.reshape(600, 800, 3))
    plt.show()
    
    # init net and print summary
    net = VoNet(IMG_CHANNELS, NUM_IMG_EXAMPLE).to(device)
    summary(net, (IMG_CHANNELS*NUM_IMG_EXAMPLE, IMG_HEIGHT, IMG_WIDTH))

    # run example and examine output shape
    x = ims.to(dtype=torch.float32).to(device)
    y = net(x).to(device)

# test_VoNet()

# Train Model
model = VoNet(3, 2).to(device=device)

criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
num_epochs = 10
n_total_steps = len(dataloader)

print("starting training")
for epoch in range(num_epochs):
    for i, (images, labels) in enumerate(dataloader):
        images = images.to(dtype=torch.float32, device=device)
        labels = labels.to(device)

        # Forward pass
        outputs = model(images)
        loss = criterion(outputs, labels)

        # Backward and optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if (i+1) % 100 == 0:
            print (f'Epoch [{epoch+1}/{num_epochs}], Step [{i+1}/{n_total_steps}], Loss: {loss.item():.4f}, dx pred: {outputs[0,0].item():.4f}')

print('Finished Training')





















