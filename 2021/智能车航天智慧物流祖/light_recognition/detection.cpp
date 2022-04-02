#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

Point left_line[2];
Point right_line[2];

Mat origin;

#define hight_rate 0.58
void process(Mat &frame);
void GET_blue(Mat &frame);
void Mark_Point(Mat &frame);
void channel(Mat &frame);

int edgeThresh = 1;

int main(int argc, char **argv)
{
	VideoCapture capture("r.mp4");

	int height = capture.get(CAP_PROP_FRAME_HEIGHT);
	int width = capture.get(CAP_PROP_FRAME_WIDTH);
	int count = capture.get(CAP_PROP_FRAME_COUNT);
	int fps = capture.get(CAP_PROP_FPS);


	namedWindow("test");
    createTrackbar("Canny threshold", "test", &edgeThresh, 100, nullptr);

	cout << height << "       " << width << "       " << count << "       " << fps << endl;

	Mat frame;
	
	while (true)
	{
		int ret = capture.read(frame);
		if (!ret)
		{
			break;
		}
		resize(frame, frame, Size(1280, 720));
		origin = frame.clone();
		frame=frame(Rect(0,frame.rows*hight_rate,frame.cols,frame.rows -frame.rows*hight_rate));

		process(frame);

		char c = waitKey();
		if (c == 27)
		{
			break;
		}
	}
}

void process(Mat &frame)
{
	channel(frame);
	Mark_Point(frame);
	imshow("origin", origin);
	imshow("  ", frame);
}

void Mark_Point(Mat &frame)
{
	for (int i = frame.rows * 3 / 5; i < frame.rows * 4 / 5; i++)
	{
		for (int j = 0; j < frame.cols; j++)
		{
			if (frame.at<Vec3b>(i, j)[0] == 0 &&
				frame.at<Vec3b>(i, j)[1] == 0 &&
				frame.at<Vec3b>(i, j)[2] == 0)
			{
				origin.at<Vec3b>(i*hight_rate, j)[0] = 0;
				origin.at<Vec3b>(i*hight_rate, j)[1] = 0;
				origin.at<Vec3b>(i*hight_rate, j)[2] = 255;
				break;
			}
		}
	}
}

void channel(Mat &frame){
	
	unsigned char pixelB, pixelG, pixelR;
	vector<Mat> channels;
	split(frame,channels);
	frame = channels.at(0) - channels.at(2);
	cv::Mat test;	frame.copyTo(test);
	erode(test, test, getStructuringElement(MORPH_RECT, Size(11, 11)));
	dilate(test, test, getStructuringElement(MORPH_RECT, Size(11, 11)));
	GaussianBlur(test, test, {11,11}, 0);
	cv::Mat edge;
	Canny(test, edge, edgeThresh,edgeThresh*3);

	imshow("test", edge);
	threshold(frame,frame,40,255,THRESH_BINARY_INV);
	int i = 0, j = 0;
	Mat element1 = getStructuringElement(MORPH_RECT, Size(7, 7));
	Mat element2 = getStructuringElement(MORPH_RECT, Size(9, 9));
    dilate(frame, frame, element1, Point(-1, -1), 1);
	erode(frame, frame, element1, Point(-1, -1), 1, 0);
}