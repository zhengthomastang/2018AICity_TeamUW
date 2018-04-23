# YOLO_LP_IPL

This is an extension to the branch of [YOLOv2](https://github.com/pjreddie/darknet), including our trained model for license plate detection on the dataset of NVIDIA AI City Challenge. 

## How It Works

We train a DCNN model to detect the license plate region in each cropped vehicle image. For each trajectory set, 3 representative views of object instances are selected for license plate recognition. The license plate detector is run on cropped vehicle images and the detected region with the highest score is chosen for comparison. 

## Getting Started

### Prerequisites

0. Linux system
1. CUDA and cuDNN
2. OpenCV 3
3. YOLOv2

The code has been tested on Ubuntu 14.04 with g++, CUDA 8, cuDNN 5.1 and OpenCV 3.1. 

### Installing

Download the package of [YOLOv2](https://github.com/pjreddie/darknet) and extract all the contents. Replace the files in YOLOv2 by those given in this directory. The trained model (weights) can be downloaded [here](https://drive.google.com/file/d/1feRI56GgkjDKCZOul4-xeAEQv3DjbuhH/view?usp=sharing). Follow the instruction in Joseph Redmon's [blog](https://pjreddie.com/darknet/yolov2/) to make and test the trained model. The given bash file provides an example of processing the entire dataset.  

<div align="center">
    <img src="demo0.jpg", width="100">
</div>

<div align="center">
    <img src="demo1.jpg", width="100">
</div>

<div align="center">
    <img src="demo2.jpg", width="100">
</div>

### Input/Output Format

For input paths of vehicle images in text, the format of each line is as follows:

\<input_vehicle_image_path\>

This text file simply lists all the cropped vehicle images for license plate detection.   

For output detection results in text, the format of each line is as follows:

\<xmin\> \<ymin\> \<width\> \<height\> \<confidence\> \<input_vehicle_image_path\>

The confidence is in percentage. The license plate image(s) can be cropped from each vehicle image based on the detection results. 