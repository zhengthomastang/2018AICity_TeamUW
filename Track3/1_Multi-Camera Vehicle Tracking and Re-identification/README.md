# 1_Multi-Camera Vehicle Tracking and Re-identification

This is an implementation of multi-camera vehicle tracking and re-identification based on a fusion of histogram-based adaptive appearance models, DCNN features, detected car types and traveling time information.

## Introduction

In inter-camera tracking (ICT), the loss function is computed based on a fusion of histogram-based adaptive appearance models, DCNN features, detected car types and traveling time information. The computed distance score is combined with the result in license plate comparison to derive the final matching vehicle pairs. 

## How to Build & Use

Please refer to the package from [Hao Xiao's GitHub repository](https://github.com/AlexXiao95/Multi-camera-Vehicle-Tracking-and-Reidentification), where detailed instruction of configuration and installation is provided.

### Input & Output Format

The input is single camera tracking results for all 15 videos by using our method in Track 1. The format of each line is as follows:

\<video_id\> \<frame_id\> \<obj_id\> \<xmin\> \<ymin\> \<xmax\> \<ymax\> \<speed\> \<confidence\>
  
* \<video_id\> is the video numeric identifier, starting with 1. It represents the position of the video in the list of all track videos, sorted in alphanumeric order.
* \<frame_id\> represents the frame count for the current frame in the current video, starting with 1.
* \<obj_id\> is a numeric identifier. It is integer. It can be ignored for Track 1 (set to -1).
* The axis-aligned rectangular bounding box of the detected video will be denoted by its pixel-valued coordinates within the image canvas, \<xmin\> \<ymin\> \<xmax\> \<ymax\>, computed from the top-left corner of the image (similar to the VOC2012 challenge format). All coordinates are integers.
* \<speed\> denotes the instantaneous speed of the vehicle in the given frame, measured in miles per hour (mi/h), which is a non-negative real value.
* \<confidence\> denotes the confidence of the prediction. Should be between 0 and 1.

The output is all possible candidates which will be used for [license plate comparison](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track3/6_LP_COMP). The format of each line is as follows:

\<img_path\> \<similarity\>

* \<img_path\> is the path of probe vehicle and gallery vehicle. The first line is the probe image and the follwing is gallery images which are in a descending order in terms of similarity.
* \<similarity\> is the similarity between probe vehicle and gallery vehicle based on a fusion of histogram-based adaptive appearance models, DCNN features, detected car types and traveling time information.  

## Disclaimer

For any question you can contact [Zheng (Thomas) Tang](https://github.com/zhengthomastang).