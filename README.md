
</h4>  
<p align="center">
    <img src="https://raw.githubusercontent.com/sygant/onthefly/refs/heads/main/src/UI/media/WindowIcon.png" width=138/>
</p>  
<h1 align="center">On-the-Fly SfM</h1>  
<p align="center"><strong>Running online SfM while image capturing!</strong></p>



<div align="left">Similar to conventional SfM, On-the-Fly SfM yields image poses and 3D sparse points, but we do this while the image capture. More specifically, the current image’s pose and corresponding 3D points can be estimated before next image is captured and on-the-fly to be processed, i.e., what you capture is what you get. For the latest version, please see the <a href="https://github.com/sygant/onthefly/releases">release page</a>. or visit: <a href="https://yifeiyu225.github.io/on-the-flySfMv2.github.io" target="_blank">On-the-Fly SfM Homepage</a>. Check the README for information on and <a href="#development">custom development</a>. Feel free to ask any questions in the <a href="https://github.com/sygant/onthefly/issues">issue area</a>.</div>

https://yifeiyu225.github.io/on-the-flySfMv2.github.io/On_the_Fly%20SfMv2.mp4

## Installation
You can either download one of the pre-built binaries(for windows only) or build the source code manually. 
### Pre-built Binaries
- Follow the instructions provided in the link below to complete the download and installation. <a href="https://sygant.github.io/onthefly/">document</a>

### Build from Source
The development environment is based on Windows 10 with Visual Studio Community 2022. The project is based on C++17, Qt 5.15.12, Python 3.10.11, PyTorch 1.13.1, CUDA 11.7, and cuDNN 8.8. It also incorporates a variety of open-source libraries, including `Boost`, `Ceres Solver`, `OpenCV`, `FLANN`, and `FAISS`. It is recommended to use `vcpkg` to compile and manage these libraries.

- Checkout the latest source code:
```cmd
git clone https://github.com/sygant/onthefly.git
```

- Download the <a href="https://drive.google.com/drive/folders/1L9QuDZ8azWhNjukgIJEwczfXLeg68Gd0?usp=drive_link">pretrained weights</a> for image retrival, and then copy the files to the following folder: <br /> `/src/Feature/GlobalFeature/`

- Download the <a href="https://drive.google.com/drive/folders/1Nhv3Gt9XpG7Q-4qVyvjCpcsS8oLAV2GE?usp=drive_link">Python Environment</a>, and then copy the folder to the following path: <br /> `/thirdparty/`, example file path is like: `/thirdparty/Python/python.exe`

- Modify the corresponding Python environment path in the `PyLoader` class.

## About
If you use this project for your research, please cite: <br />
``` 
@article{zhan2025sfm,
  title={SfM on-the-fly: A robust near real-time SfM for spatiotemporally disordered high-resolution imagery from multiple agents},
  author={Zhan, Zongqian and Yu, Yifei and Xia, Rui and Gan, Wentian and Xie, Hong and Perda, Giulio and Morelli, Luca and Remondino, Fabio and Wang, Xin},
  journal={ISPRS Journal of Photogrammetry and Remote Sensing},
  volume={224},
  pages={202--221},
  year={2025},
  publisher={Elsevier}
} 
```

## Acknowledgment
This work was jointly supported by the National Natural Science Foundation of China (Grant No. 42301507, 61871295), Natural Science Foundation of Hubei Province, China (No. 2022CFB727) and ISPRS Initiatives 2023.