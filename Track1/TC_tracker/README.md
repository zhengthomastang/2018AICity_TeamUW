# TC_tracker

This is an implementation of tracklet clustering for 2D tracking.

**Note that this single-camera tracking-by-detection method has been updated into TrackletNet Tracker (TNT). The corresponding paper on arXiv is [here](https://arxiv.org/abs/1811.07258). The source code (training + testing) is provided [here](https://github.com/GaoangW/TNT).**  

`TC_tracker` is mainly developed by Gaoang Wang, with the joint efforts from Zheng (Thomas) Tang, Hao Xiao, Aotian Zheng, and Prof. Jenq-Neng Hwang from the Information Processing Lab in the Department of Electrical Engineering, University of Washington. 

## How It Works

In single-camera tracking (SCT), the loss function consists of motion, temporal and appearance attributes. Especially, a histogram-based adaptive appearance model is designed to encode long-term appearance change for enhanced robustness. Then we employ a clustering method to associate tracklets into longer trajectories. The clustering operations are determined by minimizing a loss function measuring the loss in the assignment of tracklets. 

## Getting Started

### Prerequisite

MATLAB

### Installing

Please download the MATLAB toolbox from [Gaoang Wang's GitHub repository](https://github.com/GaoangW/TC_tracker), where further instruction of configuration and installation is provided.

### Input/Output Format

For input detection results in text, the format of each line is as follows:

\<frame_id\>,-1,\<xmin\>,\<ymin\>,\<width\>,\<height\>,\<confidence\>,-1,-1,-1,\<class\>

This is similar to the required format of [MOTChallenge](https://motchallenge.net/). The frame ID is 0-based. The confidence is in percentage. 

For output 2D tracking results in text, the format of each line is as follows:

\<frame_id\>,\<obj_id\>,\<xmin\>,\<ymin\>,\<width\>,\<height\>,\<confidence\>,-1,-1,-1,\<class\>

This is similar to the required format of [MOTChallenge](https://motchallenge.net/). The frame ID and object ID are both 0-based. The confidence is in percentage.  
