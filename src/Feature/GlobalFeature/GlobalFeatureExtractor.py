import os

import h5py
from PIL import Image, ImageFile
import NetVlad
import random
import warnings

import joblib
import numpy as np
import torch
from torch import nn, optim
from torchvision import models, transforms


class GlobalFeatureExtractor:
    def __init__(self, pthPath, HDF5Path, checkPointPath, PCAModelPath):
        self.InitialParams()
        self.pthPath = pthPath
        self.HDF5Path = HDF5Path
        self.checkPointPath = checkPointPath
        self.device = torch.device("cuda")
        warnings.filterwarnings("ignore", category=UserWarning)
        self.pca = joblib.load(PCAModelPath)

        self.encoder = models.vgg16(pretrained=False)
        self.encoder.load_state_dict(torch.load(self.pthPath))
        layers = list(self.encoder.features.children())[:-2]
        self.encoder = nn.Sequential(*layers)
        self.model = nn.Module()
        self.model.add_module('encoder', self.encoder)
        net_vlad = NetVlad.NetVLAD(num_clusters=self.num_clusters, dim=self.encoder_dim, vladv2=False)
        with h5py.File(self.HDF5Path, mode='r') as h5:
            clsts = h5.get("centroids")[...]
            traindescs = h5.get("descriptors")[...]
            net_vlad.init_params(clsts, traindescs)
            del clsts, traindescs

        self.model.add_module('pool', net_vlad)
        self.model.encoder = nn.DataParallel(self.model.encoder)
        self.model.pool = nn.DataParallel(self.model.pool)
        optimizer = optim.SGD(filter(lambda p: p.requires_grad, self.model.parameters()), lr=self.learnRate,
                              momentum=self.momentum, weight_decay=self.weightDecay)
        scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=self.learnRateStep, gamma=self.learnRateGamma)
        criterion = nn.TripletMarginLoss(margin=self.margin, p=2, reduction='sum').to(self.device)
        checkpoint = torch.load(self.checkPointPath, map_location=lambda storage, loc: storage)
        testEpoch = checkpoint['epoch']
        bestMetric = checkpoint['best_score']
        self.model.load_state_dict(checkpoint['state_dict'], strict=False)
        self.model = self.model.to(self.device)

        ImageFile.LOAD_TRUNCATED_IMAGES = True
        self.transform = transforms.Compose([
            transforms.Resize((self.newWidth, self.newHeight)),
            transforms.Lambda(lambda image: image.convert('RGB') if image.mode != 'RGB' else image),
            transforms.ToTensor(),
        ])

    def Extract(self, imagePath):
        imagePath = imagePath.replace('\\', '/')
        if not os.path.exists(imagePath):
            raise ValueError("路径不存在:" + imagePath)
        image = Image.open(imagePath)
        image = self.transform(image).to(torch.device("cuda"))
        self.model.eval()
        feature = None
        with torch.no_grad():
            feature = self.model.encoder(image).unsqueeze(0)
            feature = self.model.pool(feature).to(torch.device("cpu"))
            if self.pca is not None:
                feature = self.pca.transform(feature)
        return feature

    def InitialParams(self):
        self.num_clusters = 64
        self.encoder_dim = 512
        self.newWidth = 224
        self.newHeight = 224

        seed = 42
        random.seed(seed)
        np.random.seed(seed)
        torch.manual_seed(seed)
        torch.cuda.manual_seed(seed)

        self.margin = 1
        self.learnRate = 0.0001
        self.learnRateStep = 5
        self.learnRateGamma = 0.5
        self.momentum = 0.9
        self.weightDecay = 0.001

if __name__ == '__main__':
    import time
    globalFeatureExtractor = GlobalFeatureExtractor("vgg16-397923af.pth", "VGG16_64_desc_cen.hdf5",
                                                    "VGG16_NetVlad_NoSplit.pth.tar", "PCA_dims32768to2048.model")


    def ext():
        globalFeature = globalFeatureExtractor.Extract(
            r"D:\Dataset\FBK\FBK_collaborative_smartphone\CaffeItalia\acquisitions\merge\0_00000314.jpg")


    for i in range(30):
        start = time.time()
        ext()
        end = time.time()
        print(end - start)
# python3.10 0.126  torch==1.3
# python3.10(2) 0.877   torch==2.3