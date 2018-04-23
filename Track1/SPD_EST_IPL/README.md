# SPD_EST_IPL

This is an implementation of 3D speed estimation based on input of 2D tracking results and camera parameters.

`SPD_EST_IPL` is joint work with Zheng (Thomas) Tang, Gaoang Wang, Hao Xiao, Aotian Zheng, and Prof. Jenq-Neng Hwang from the Information Processing Lab in the Department of Electrical Engineering, University of Washington. 

## How It Works

For each video, the projection matrix and the 2D tracking results at each frame are input to our algorithm. Each object foot point is backprojected to the 3D space. The 3D speed is computed based on a sliding time window with smoothing. Since the precision of 3D estimation is negatively proportional to depth, we use the speed of object instances close to the camera to propagate through each entire object trajectory, if his/her speed variance is smaller than a given threshold. For an object instance whose speed is too small, we assume that s/he is stopping. Finally, for the false negatives whose detection scores are too low, the speed of the closest true positives are applied to them. 

## Getting Started

### Prerequisites

0. Windows or Linux system
1. OpenCV (included as a 3rd-party software component in the package)

The code has been tested on Windows 10 64 bit with Visual Studio 2017 (v141). 

### Installing

Build the VC++ solution in Release mode in Visual Studio. The source code is under `.\src`. The 3rd-party software components (headers, libraries and/or DLLs) are included in `.\3rdparty`.

When running the solution, make sure that you have all the required input files and folders at their corresponding locations set in the main function, i.e., at each video folder `\LocX_Y\`, the 2D tracking results in `trk2d.txt`, the camera parameters in `\camCal\camParam.txt`, and the frame images in `\img1\`. 

<div align="center">
    <img src="demo.png", width="1000">
</div>

### Input/Output Format

For input camera parameters in text, the format is as follows:

\<K_0\> \<K_1\> \<K_2\> \<K_3\> \<K_4\> \<K_5\> \<K_6\> \<K_7\> \<K_8\>

\<R_0\> \<R_1\> \<R_2\> \<R_3\> \<R_4\> \<R_5\> \<R_6\> \<R_7\> \<R_8\>

\<t_0\> \<t_1\> \<t_2\>

\<P_0\> \<P_1\> \<P_2\> \<P_3\> \<P_4\> \<P_5\> \<P_6\> \<P_7\> \<P_8\> \<P_9\> \<P_10\> \<P_11\>

K, R, t and P respectively stand for the intrinsic camera matrix, the rotation matrix, the translation vector and the projection matrix. In the 3D coordinate system, the ground plane is equivalent to the X-Y plane. The Z axis points upward and passes through the camera location. 

For input 2D tracking results in text, the format of each line is as follows:

\<frame_id\>,\<obj_id\>,\<xmin\>,\<ymin\>,\<width\>,\<height\>,\<confidence\>,-1,-1,-1,\<class\>

This is similar to the required format of [MOTChallenge](https://motchallenge.net/). The frame ID and object ID are both 0-based. They will be converted to 1-based in output. The confidence is in percentage. 

For output 3D tracking results in text, the format of each line is as follows: 

\<video_id\> \<frame_id\> \<obj_id\> \<xmin\> \<ymin\> \<xmax\> \<ymax\> \<speed\> \<confidence\>
  
* \<video_id\> is the video numeric identifier, starting with 1. It represents the position of the video in the list of all track videos, sorted in alphanumeric order.
* \<frame_id\> represents the frame count for the current frame in the current video, starting with 1.
* \<obj_id\> is a numeric identifier. It is integer. It can be ignored for Track 1 (set to -1).
* The axis-aligned rectangular bounding box of the detected video will be denoted by its pixel-valued coordinates within the image canvas, \<xmin\> \<ymin\> \<xmax\> \<ymax\>, computed from the top-left corner of the image (similar to the VOC2012 challenge format). All coordinates are integers.
* \<speed\> denotes the instantaneous speed of the vehicle in the given frame, measured in miles per hour (mi/h), which is a non-negative real value.
* \<confidence\> denotes the confidence of the prediction. Should be between 0 and 1.