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

class Block(nn.Module):
    def __init__(self, in_channels, intermediate_channels, identity_downsample=None, stride=1):
        super(Block, self).__init__()
        self.expansion = 4
        
        self.conv1 = nn.Conv2d(in_channels, 
                               intermediate_channels, 
                               kernel_size=1, 
                               stride=1, 
                               padding=0,
                               bias=False)
        self.bn1 = nn.BatchNorm2d(intermediate_channels)
        self.conv2 = nn.Conv2d(intermediate_channels, 
                               intermediate_channels, 
                               kernel_size=3, 
                               stride=stride, 
                               padding=1,
                               bias=False)
        self.bn2 = nn.BatchNorm2d(intermediate_channels)
        self.conv3 = nn.Conv2d(intermediate_channels, 
                               intermediate_channels*self.expansion, 
                               kernel_size=1, 
                               stride=1, 
                               padding=0,
                               bias=False)
        self.bn3 = nn.BatchNorm2d(intermediate_channels*self.expansion)
        self.relu = nn.ReLU()
        self.identity_downsample = identity_downsample
        self.stride = stride
    
    def forward(self, x):
        identity = x

        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.conv2(x)
        x = self.bn2(x)
        x = self.relu(x)
        x = self.conv3(x)
        x = self.bn3(x)

        if self.identity_downsample is not None:
            identity = self.identity_downsample(identity)

        x += identity
        x = self.relu(x)
        return x

class ResNet(nn.Module):
    def __init__(self, Block, layers, image_channels, num_classes):
        super(ResNet, self).__init__()
        self.in_channels = 64
        self.conv1 = nn.Conv2d(image_channels, 64, kernel_size=7, stride=2, padding=3)
        self.bn1 = nn.BatchNorm2d(64)
        self.relu = nn.ReLU()
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)

        # ResNet Layers
        self.layer1 = self._make_layer(Block, layers[0], intermediate_channels=64, stride=1)
        self.layer2 = self._make_layer(Block, layers[1], intermediate_channels=128, stride=2)
        self.layer3 = self._make_layer(Block, layers[2], intermediate_channels=256, stride=2)
        self.layer4 = self._make_layer(Block, layers[3], intermediate_channels=512, stride=2)

        self.avgpool = nn.AdaptiveAvgPool2d((1,1))
        self.fc = nn.Linear(512*4, num_classes)

    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)

        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)

        x = self.avgpool(x)
        x = x.reshape(x.shape[0], -1)
        x = self.fc(x)

        return x


    def _make_layer(self, Block, num_residual_blocks, intermediate_channels, stride):
        identity_downsample = None
        layers = []

        if stride != 1 or self.in_channels != intermediate_channels * 4:
            identity_downsample = nn.Sequential(nn.Conv2d(self.in_channels, 
                                                          intermediate_channels * 4, 
                                                          kernel_size=1,
                                                          stride=stride),
                                                nn.BatchNorm2d(intermediate_channels*4))

        layers.append(Block(self.in_channels, intermediate_channels, identity_downsample, stride))
        self.in_channels = intermediate_channels*4

        for i in range(num_residual_blocks - 1):
            layers.append(Block(self.in_channels, intermediate_channels))
        
        return nn.Sequential(*layers)

def ResNet50(img_channels=3, num_classes=1000):
    return ResNet(Block, [3, 4, 6, 3], img_channels, num_classes)

def ResNet101(img_channels=3, num_classes=1000):
    return ResNet(Block, [3, 4, 23, 3], img_channels, num_classes)

def ResNet152(img_channels=3, num_classes=1000):
    return ResNet(Block, [3, 8, 36, 3], img_channels, num_classes)

def test():
    if torch.cuda.is_available():
        print("Working with Cuda!")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # PyTorch v0.4.0
    net = ResNet50(img_channels=6, num_classes=5).to(device)
    summary(net, (6, 224, 224))
    x = torch.randn([2,6,224,224], device=device)
    y = net(x).to('cuda')
    print(y.shape)

test()