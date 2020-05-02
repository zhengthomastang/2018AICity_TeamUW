# Track 3 (Multi-camera Vehicle Detection and Reidentification)

## Coding Structure

Under the Track3 folder, there are 3 components:

1. [1_Multi-Camera Vehicle Tracking and Re-identification](1_Multi-Camera%20Vehicle%20Tracking%20and%20Re-identification): Multi-camera vehicle tracking based on a fusion of histogram-based adaptive appearance models, DCNN features, detected car types and traveling time information
2. [2_YOLO_LP](2_YOLO_LP): Detection of license plate from each cropped vehicle image based on YOLOv2 with our trained model provided 
**We strongly encourage users to try the latest [YOLOv4 object detector](https://github.com/AlexeyAB/darknet) instead.**  
3. [3_LP_COMP](3_LP_COMP): Comparison of license plates under low resolution

**Detailed description of each package is given in each subfolder.**

The output of [1_Multi-Camera Vehicle Tracking and Re-identification](1_Multi-Camera%20Vehicle%20Tracking%20and%20Re-identification) is the similarity scores between pairs of vehicles for comparison. It can be converted into a distance score by inverse proportion. The output of [3_LP_COMP](3_LP_COMP) is the distance score between each two license plates. The final distance score between two vehicles is the multiplication of the above two distance scores. 
