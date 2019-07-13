#include <iostream>
#include <sys/stat.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{
    // create video reader for input video
    cv::VideoCapture oVdoCap;
    oVdoCap = cv::VideoCapture(argv[1]);

    if (!oVdoCap.isOpened())
	{
		std::cout << "Error: The video is not captured properly" << std::endl;
		return 0;
	}

    // create folder for output images
    mkdir(argv[2], S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    int nFrmCnt = 0;
    cv::Mat oImgFrm;

    // process every frame
    while (true)
	{
        std::printf("frame %06d\n", nFrmCnt);

        oVdoCap >> oImgFrm;

        if (oImgFrm.empty())
			break;

        char acOutFrmNm[128] = { 0 };
        std::sprintf(acOutFrmNm, "/%06d.jpg", nFrmCnt);
        char acOutFrmPth[128] = { 0 };
        std::strcpy(acOutFrmPth, argv[2]);
        std::strcat(acOutFrmPth, acOutFrmNm);
        cv::imwrite(acOutFrmPth, oImgFrm);

        //cv::namedWindow("current frame", CV_WINDOW_AUTOSIZE);
        //cv::imshow("current frame", oImgFrm);
        //cv::waitKey(0);

        nFrmCnt++;
	}

	std::printf("finished\n");

    return 0;
}



