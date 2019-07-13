# 3_YOLO_VEH_IPL

This is an extension to [YOLOv2](https://pjreddie.com/darknet/yolov2/), including our trained model for vehicle detection on the dataset of 2018 NVIDIA AI City Challenge. 

**We strongly encourage users to try the latest [YOLOv3 object detector](https://pjreddie.com/darknet/yolo/) instead.** 

## Introduction

We select 4,500 frames uniformly sampled from the dataset of 2018 NVIDIA AI City Challenge, where each of them contains 5 to 40 objects. The training data are manually labeled in 8 categories, including sedan, hatchback, bus, pickup, minibus, van, truck and motorcycle. YOLOv2 is used for training and testing, where the pretrained weights are used to initialize the network. 

## How to Build & Use

1. Download the trained model (weights) [here](https://drive.google.com/open?id=1k1ha7q_Zuv3V9VL_8k47vCaLB0tS8w-J). 
2. Follow the instruction in Joseph Redmon's [blog](https://pjreddie.com/darknet/yolov2/) to make and test the trained model. The given bash files provide examples of processing the entire dataset. 

<div align="center">
    <img src="demo.png", width="1000">
</div>

### Output Format

For output detection results in text, the format of each line is as follows:

\<frame_id\>,-1,\<xmin\>,\<ymin\>,\<width\>,\<height\>,\<confidence\>,-1,-1,-1,\<class\>

This is similar to the required format of [MOTChallenge](https://motchallenge.net/). The frame ID is 0-based. The confidence is in percentage. 

## Disclaimer

For any question you can contact [Zheng (Thomas) Tang](https://github.com/zhengthomastang).