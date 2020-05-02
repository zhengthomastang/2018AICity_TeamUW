# 4_TC_tracker

This is an implementation of tracklet clustering for 2D tracking.

**Note that this SCT method has been upgraded to TrackletNet Tracker (TNT). The corresponding paper on arXiv is [here](https://arxiv.org/abs/1811.07258). The source code (training + testing) is provided [here](https://github.com/GaoangW/TNT).**

## Introduction

In SCT, the loss function in our data association algorithm consists of motion, temporal and appearance attributes. Especially, a histogram-based adaptive appearance model is designed to encode long-term appearance change. The change of loss is incorporated with a bottom-up clustering strategy for the association of tracklets. Robust 2D-to-3D projection is achieved with EDA optimization applied to camera calibration for speed estimation. 

## How to Build & Use

Please refer to [Gaoang Wang's GitHub repository](https://github.com/GaoangW/TC_tracker), where detailed instructions of configuration and installation are provided.

### Input & Output Format

For input detection results in text, the format of each line is as follows:

\<frame_id\>,-1,\<xmin\>,\<ymin\>,\<width\>,\<height\>,\<confidence\>,-1,-1,-1,\<class\>

This is similar to the required format of [MOTChallenge](https://motchallenge.net/). The frame ID is 0-based. The confidence is in percentage. 

For output 2D tracking results in text, the format of each line is as follows:

\<frame_id\>,\<obj_id\>,\<xmin\>,\<ymin\>,\<width\>,\<height\>,\<confidence\>,-1,-1,-1,\<class\>

This is similar to the required format of [MOTChallenge](https://motchallenge.net/). The frame ID and object ID are both 0-based. The confidence is in percentage.  

## Disclaimer

For any question you can contact [Zheng (Thomas) Tang](https://github.com/zhengthomastang).
