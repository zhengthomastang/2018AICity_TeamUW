# Track 1 (Traffic Flow Analysis / Speed Estimation) 

## Code structure

Under the [Track1](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track1) folder, there are 6 components:

1. [1_VDO2IMG](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track1/1_VDO2IMG): Converting each video file to a folder of frame images
2. [2_CAM_CAL](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track1/2_CAM_CAL): Manual camera calibration based on minimization of reprojection error by EDA optimization  
**With the access to GPS coordinates (using Google Maps or other tools), you can use our newly developed PnP-based calibration tool [here](https://github.com/zhengthomastang/Cal_PnP) instead.**  
3. [3_YOLO_VEH](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track1/3_YOLO_VEH): Extension of the YOLOv2 object detector with our trained model for vehicle detection/classification  
**We strongly encourage users to try the latest [YOLOv3 object detector](https://pjreddie.com/darknet/yolo/) instead.**  
4. [4_TC_tracker](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track1/4_TC_tracker): Proposed tracklet-clustering-based tracking method  
**Note that this SCT method has been upgraded into TrackletNet Tracker (TNT). The corresponding paper on arXiv is [here](https://arxiv.org/abs/1811.07258). The source code (training + testing) is provided [here](https://github.com/GaoangW/TNT).**  
5. [5_APP_MDL](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track1/5_APP_MDL) **(optional)**: Extraction of histogram-based adaptive apperance models and their comparison
6. [6_SPD_EST](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track1/6_SPD_EST): Speed estimation based on input of tracking results and camera parameters

**Detailed description of each package is given in each subfolder.**