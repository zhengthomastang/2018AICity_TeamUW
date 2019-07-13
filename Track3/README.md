# Track 3 (Multi-camera Vehicle Detection and Reidentification)

## Coding Structure

Under the [Track3](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track3) folder, there are 3 components:

1. [1_Multi-Camera Vehicle Tracking and Re-identification](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track3/1_Multi-Camera%20Vehicle%20Tracking%20and%20Re-identification): Multi-camera vehicle tracking based on a fusion of histogram-based adaptive appearance models, DCNN features, detected car types and traveling time information
2. [2_YOLO_LP](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track3/2_YOLO_LP): Detection of license plate from each cropped vehicle image based on YOLOv2 
**We strongly encourage users to try the latest [YOLOv3 object detector](https://pjreddie.com/darknet/yolo/) instead.**  
3. [3_LP_COMP](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track3/3_LP_COMP): Comparison of license plates under low resolution

**Detailed description of each package is given in each subfolder.**

The output of [1_Multi-Camera Vehicle Tracking and Re-identification](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track3/1_Multi-Camera%20Vehicle%20Tracking%20and%20Re-identification) is the similarity score between each pair of vehicles for comparison. It can be converted into a distance score by inverse proportion. The output of [3_LP_COMP](https://github.com/zhengthomastang/2018AICity_TeamUW/tree/master/Track3/3_LP_COMP) is the distance score between each two license plates. The final distance score between two vehicles is the multiplication of the above two distance scores. 
