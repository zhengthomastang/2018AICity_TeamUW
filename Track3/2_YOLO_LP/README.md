# 2_YOLO_LP

This is an extension to [YOLOv2](https://pjreddie.com/darknet/yolov2/), including our trained model for license plate detection on the dataset of 2018 NVIDIA AI City Challenge. 

**We strongly encourage users to try the latest [YOLOv4 object detector](https://github.com/AlexeyAB/darknet) instead.**

## Introduction

We train a DCNN model to detect the license plate region in each cropped vehicle image. For each trajectory set, 3 representative views of object instances are selected for license plate recognition. The license plate detector is run on cropped vehicle images and the detected region with the highest score is chosen for comparison. 

## How to Build & Use

1. Download the trained model (weights) [here](https://drive.google.com/open?id=1Td3e5oeOlFheczMvP0f9t--scga9U2e2). 
2. Follow the instruction in Joseph Redmon's [blog](https://pjreddie.com/darknet/yolov2/) to make and test the trained model. The given bash files provide examples of processing the entire dataset. 

<div align="center">
    <img src="demo0.jpg", width="100">
</div>

<div align="center">
    <img src="demo1.jpg", width="100">
</div>

<div align="center">
    <img src="demo2.jpg", width="100">
</div>

### Input & Output Format

For input paths of vehicle images in text, the format of each line is as follows:

\<input_vehicle_image_path\>

This text file simply lists all the cropped vehicle images for license plate detection.   

For output detection results in text, the format of each line is as follows:

\<xmin\> \<ymin\> \<width\> \<height\> \<confidence\> \<input_vehicle_image_path\>

The confidence is in percentage. The license plate image(s) can be cropped from each vehicle image based on the detection results. 

## Disclaimer

For any question you can contact [Zheng (Thomas) Tang](https://github.com/zhengthomastang).
