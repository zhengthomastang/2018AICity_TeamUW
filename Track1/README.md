# Track 1 (Traffic Flow Analysis / Speed Estimation) 

## Coding Structure

Under the Track1 folder, there are 6 components:

1. [1_VDO2IMG](1_VDO2IMG): Converting each video file to a folder of frame images
2. [2_CAM_CAL](2_CAM_CAL): Semi-automatic camera calibration based on minimization of reprojection error by EDA optimization  
**With the access to GPS coordinates (using Google Maps or other tools), we suggest you to use our newly developed PnP-based calibration tool [here](https://github.com/zhengthomastang/Cal_PnP) instead.**  
3. [3_YOLO_VEH](3_YOLO_VEH): Extension of the YOLOv2 object detector with our trained model for vehicle detection/classification provided  
**We strongly encourage users to use the latest [YOLOv4 object detector](https://github.com/AlexeyAB/darknet) instead.**  
4. [4_TC_tracker](4_TC_tracker): Proposed tracklet-clustering-based tracking method  
**Note that this SCT method has been upgraded to TrackletNet Tracker (TNT). The corresponding paper on arXiv is [here](https://arxiv.org/abs/1811.07258). The source code (training + testing) is provided [here](https://github.com/GaoangW/TNT).**  
5. [5_APP_MDL](5_APP_MDL) **(optional)**: Extraction of histogram-based adaptive apperance models and their comparison
6. [6_SPD_EST](6_SPD_EST): Speed estimation based on input of tracking results and camera parameters

**Detailed description of each package is given in each subfolder.**
