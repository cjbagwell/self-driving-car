"""
    Network Structure Rough Draft:

    CONV3_STRIDE = 1
    CONV3_PADDING = 1

    2 color images as inputs: (800 x 600 x 6)

    conv3 layer 64
    conv3 layer 64 x (3 x 3 x K) --> (800 x 600 x 64)
    conv3 layer 64 x (3 x 3 x 64)--> (800 x 600 x 64)

"""


import torch
import torch.nn as nn
from torchsummary import summary

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

        self.fc1 = nn.Linear(3840, 1000)
        self.fc2 = nn.Linear(1000, 100)
        self.fc3 = nn.Linear(100, 50)
        self.fc4 = nn.Linear(50, 10)
    
    def forward(self, x):
        print(f"in forward with x of shape{x.shape}")
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

        x = torch.flatten(x)
        x = self.fc1(x)
        x = self.fc2(x)
        x = self.fc3(x)
        x = self.fc4(x)
        return x

def test_VoNet():
    IMG_WIDTH = 800         # width of image (x)
    IMG_HEIGHT = 600        # height of image (y)
    IMG_CHANNELS = 3        # number of channels per image (k)
    NUM_IMG_EXAMPLE = 2     # number of images per example
    NUM_EXAMPLES = 5        # number of examples
    
    if torch.cuda.is_available():
        print("Working with Cuda! :D")
        device = torch.device("cuda")
    else:
        print("Working with Cpu :(")
        device = torch.device("cpu")
    
    # init net and print summary
    net = VoNet(IMG_CHANNELS, NUM_IMG_EXAMPLE).to(device)
    summary(net, (IMG_CHANNELS*NUM_IMG_EXAMPLE, IMG_WIDTH, IMG_HEIGHT))

    # run example and examine output shape
    x = torch.randn([NUM_EXAMPLES, IMG_CHANNELS*NUM_IMG_EXAMPLE, IMG_WIDTH, IMG_HEIGHT], device=device)
    y = net(x).to(device)
    print(y.shape)

test_VoNet()


























