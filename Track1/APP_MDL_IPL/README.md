# APP_MDL_IPL

This is an implementation of the extraction of histogram-based adaptive appearance features and their comparison.

`APP_MDL_IPL` is joint work with Zheng (Thomas) Tang, Gaoang Wang, Hao Xiao, Aotian Zheng, and Prof. Jenq-Neng Hwang from the Information Processing Lab in the Department of Electrical Engineering, University of Washington. 

## How It Works

The appearance model of the j-th tracklet, noted _m_j_, contains a set of _n_m_ observed concatenated histogram vectors. In our experiments, we use a combination of RGB color histogram, HSV color histogram, Lab color histogram, Linear Binary Pattern (LBP) histogram and gradient histogram for feature description, i.e., there are 11 channels with 8 bins each. For each tracklet, we keep _n_m_ copies of continuously updated histograms to “memorize” variations of the appearance. The value in each bin is normalized between 0 and 1. An example of feature maps and the corresponding histograms is shown in the figure below. The first row respectively presents the RGB, HSV, Lab, LBP and gradient feature maps for an object instance in a tracklet, which are used to build feature histograms. The second row shows the original RGB color histograms and the third row demonstrates the Gaussian spatially weighted histograms, where the contribution of background area is suppressed. 

<div align="center">
    <img src="demo.png", width="1000">
</div>

To build and update this appearance model, each cropped object region within the detected bounding box is used to build histograms. When the observation is occluded by other(s), the occluded area is removed from the object region before our building the concatenated histograms of visual features. The pixel values for histogram construction are spatially weighted by Gaussian (kernel) distribution. The spatial weight is maximum around the center of mass where the object usually occupies, which should be emphasized in our feature description. As the spatial weight decreases when a pixel gets further from the center of mass, we can suppress the background area.

Since the object instances that are closer to the camera should enjoy more reliable appearance description, the learning rate of _m_j_ is inversely proportional to the depths of vehicles’ 3D foot points. For each tracklet, _n_m_ concatenated histogram vectors extracted from instances with the smallest depths are inserted into _m_j_. Any other encountered histogram vector can be randomly swapped with an existing element with a probability equal to the learning rate. 

The appearance change loss is equivalent to the ratio of histogram vectors from two appearance models that are mismatched. 

After a group of tracklets are associated in SCT, their appearance models are merged together based on depth information following similar update scheme. The merged appearance model is used to describe the appearance change along the entire vehicle trajectory, which will be employed in ICT.

## Getting Started

### Prerequisites

0. Windows or Linux system
1. OpenCV

The code has been tested on Ubuntu 14.04 with g++ compiler. 

### Installing

Compile the source code with g++. 

When running the code, the input format is `<executable file name> <input 3D tracking results> <input folder of frame images> <output folder of feature vectors> <weight for BGR component> <weight for HSV component> <weight for Lab component> <weight for LBP component> <weight for gradient component>`. The weight for each component should be a floating number between 0 and 1, where smaller weight will suppress the corresponding component.  

To compare two adaptive appearance models, the user can use the given function `compAppMdl()`. The required input are the folder paths and IDs of the probe and the gallery. Different types of histogram comparison are provided, including EMD, correlation, chi-square, intersection and Bhattacharyya distance. The returned value is the distance normalized to 0-1 between two appearance models. 

### Input/Output Format

For input 2D tracking results in text, the format of each line is as follows:

\<frame_id\>,\<obj_id\>,\<xmin\>,\<ymin\>,\<width\>,\<height\>,\<confidence\>,-1,-1,-1,\<class\>

This is similar to the required format of [MOTChallenge](https://motchallenge.net/). The frame ID and object ID are both 0-based. The confidence is in percentage. Note that the appearance models not only can be used in ICT - When the input 2D tracking results are from generated tracklets, the output features can also be used to compute appearance change loss in SCT. 

For output feature vectors in text, there are 1 to 32 lines in each text file. Each line contains 88 feature values forming a concatenated histogram of BGR, HSV, Lab, LBP and gradient. 