#include<iostream>
#include<cmath>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int main()
{
	Mat srcImage = imread("/home/kyle/Programme/C++/opencv/image_resources/images/test1.png");
	if (srcImage.empty())
	{
		cout << "没有找到图片，请重新输入正确路径！\n";
		return -1;
	}
	Mat srcHSV;
	cvtColor(srcImage, srcHSV, COLOR_BGR2HSV);

//阈值化处理
	Mat dstImage;
	//蓝色
	//inRange(srcHSV, Scalar(100, 120, 125), Scalar(124, 220, 255), dstImage);
	//绿色
	inRange(srcHSV, Scalar(56, 90, 80), Scalar(90, 255, 200), dstImage);
	Mat element = getStructuringElement(MORPH_RECT, Size(1, 2),  Point(-1,-1));

	vector<vector<Point>> contours;
	vector<Point> contour;
	vector<Vec4i> hierarchy;
	findContours(dstImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());


//轮廓绘制
	Mat drawImage;
	double width = srcImage.cols;
	double height = srcImage.rows;

	for (int t = 0; t < contours.size(); t++)
	{
		double length2 = arcLength(contours[t], true);
		if (length2 > 800) {
			contour = contours[t];
			Scalar color = Scalar(250, 120, 220);
			drawContours(srcImage, contours, t, color, 2, 8, hierarchy, 0, Point(0, 0));
		}
	}
	imwrite("/home/kyle/Programme/C++/opencv/green/indicator1.png", srcImage);
//拉正图像
	RotatedRect rr = minAreaRect(contour);
	Point2f points[4];
	rr.points(points);
	Point2f points2[4];
	for (int i = 0; i < 4; i++) {
		if (points[i].x < rr.center.x) {
			if (points[i].y < rr.center.y) {
				points2[0] = points[i];
			}
			else if (points[i].y > rr.center.y) {
				points2[3] = points[i];
			}
		}
		else if (points[i].x > rr.center.x) {
			if (points[i].y > rr.center.y) {
				points2[2] = points[i];
			}
			else if (points[i].y < rr.center.y) {
				points2[1] = points[i];
			}
		}
	}

	double chang = sqrt((points2[1].y - points2[0].y) * (points2[1].y - points2[0].y) + (points2[1].x - points[0].x) * (points2[1].x - points[0].x));
	double kuan = sqrt((points2[3].y - points2[0].y) * (points2[3].y - points2[0].y) + (points2[3].x - points2[0].x) * (points2[3].x - points2[0].x));
	Point2f srcPoints[3];
	Point2f dstPoints[3];

	srcPoints[0] = points2[0];
	srcPoints[1] = points2[1];
	srcPoints[2] = points2[3];

	dstPoints[0] = Point2f(points2[0].x, points2[0].y);
	dstPoints[1] = Point2f(points2[0].x + chang, points2[0].y);
	dstPoints[2] = Point2f(points2[0].x, points2[0].y + kuan);

	//仿射变换
	Mat M1 = getAffineTransform(srcPoints, dstPoints);
	warpAffine(srcImage, srcImage, M1, srcImage.size());



	Rect rect1(points[1].x, points[1].y, chang, kuan);
	Mat after_1;
	srcImage(rect1).copyTo(after_1);
	imshow("蓝色指示牌", after_1);
	imwrite("/home/kyle/Programme/C++/opencv/green/result1.png", after_1);

	waitKey(0);
	return 0;
}