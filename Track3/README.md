# Track 3

## Introduction

Multi-camera Vehicle Detection and Reidentification - Participating teams identify all vehicles that are seen passing at least once at all of 4 different locations in a set of 15 videos. Evaluation for Challenge Track 3 is based on detection accuracy and localization sensitivity for a set of ground-truth vehicles that were driven through all camera locations at least once.

## How It Works

The proposed appearance model together with DCNN features, license plates, detected car types and traveling time information are combined for the computation of cost function in ICT. 

## Code structure

Under the `./Track3/` folder, there are 3 software packages:

1. `Multi-Camera Vehicle Tracking and Re-identification`: Multi-camera vehicle tracking based on a fusion of histogram-based adaptive appearance models, DCNN features, detected car types and traveling time information
2. `YOLO_LP_IPL`: Detection of license plate from each cropped vehicle image
3. `LP_COMP_IPL`: Comparison of license plates under low resolution

Detailed description of each package is given in each subfolder. 

The output of `Multi-Camera Vehicle Tracking and Re-identification` is the similarity score between each pair of vehicles for comparison. We can convert it into a distance score by inverse proportion. The output of `LP_COMP_IPL` is the distance score between each two license plates. The final distance score between two vehicles is the multiplication of the above two distance scores. Several vehicle pairs that enjoy low distance scores and appear in all 4 camera locations are submitted to Track 3 for evaluation. 