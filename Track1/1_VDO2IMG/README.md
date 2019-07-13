# 1_VDO2IMG

This is a C++ implementation to convert a video file into a folder of frame images. 

## Coding Structure

1. `./src/` folder: Source code
2. `track1.sh`: Example bash file to process the Track 1 data
3. `track3.sh`: Example bash file to process the Track 3 data

## How to Build

1. Download and make the OpenCV library. A tutorial is given [here](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html).
2. Compile using g++ in Linux environment. If you are new to g++ compilation with OpenCV, please refer to this [link](http://answers.opencv.org/question/25642/how-to-compile-basic-opencv-program-in-c-in-ubuntu/). In the command window, you can `cd` to the current directory and use the following command to compile our source code, where `bin` is the executable file generated. Note that you may need to add `sudo` at the beginning to grant the admin permission.

```g++ -I/usr/local/include/ -L/usr/local/lib/ -g -o bin ./src/main.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lm```

## How to Use

1. Run the executable file.

```./bin <input video path> <output image folder path>```

2. Or run a bash file.

```bash track1.sh```

## Disclaimer

For any question you can contact [Zheng (Thomas) Tang](https://github.com/zhengthomastang).