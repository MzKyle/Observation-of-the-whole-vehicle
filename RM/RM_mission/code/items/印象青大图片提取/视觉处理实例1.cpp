#include<iostream>
#include<cmath>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int main()
{
	Mat srcImage = imread("/home/kyle/Programme/C++/opencv/image_resources/images/test1.png");
	if (!srcImage.data)
	{
		cout << "读取图片错误，请重新输入正确路径！\n";
		system("pause");
		return -1;
	}
	Mat srcHSV;
	cvtColor(srcImage, srcHSV, COLOR_BGR2HSV);

	Mat dstImage;
	int ZYW;
	cout << "如果提取蓝色牌子请输入1 ，如果提取绿色牌子请输入2" << endl;
	cin >> ZYW;
	if(ZYW==1){ inRange(srcHSV, Scalar(100, 120, 125), Scalar(124, 220, 255), dstImage); }
//蓝色
//inRange函数的作用是将源图像srcHSV中的颜色值在Scalar(100, 120, 125)和Scalar(124, 220, 255)之间的像素设置为白色（或255），其余像素设置为黑色（或0）
	else if(ZYW==2){ inRange(srcHSV, Scalar(56, 90, 80), Scalar(90, 255, 200), dstImage); }

	imshow("【利用inRange()函数实现阈值化】", dstImage);
/*
getStructuringElement 是一个函数，用于创建形态学操作的结构元素。
MORPH_RECT 是一个参数，表示结构元素的形状为矩形。
Size(1, 2) 是一个参数，表示矩形的宽度为1个像素，高度为2个像素。
Point(-1, -1) 是一个参数，表示矩形的中心点位置。这里的值是(-1, -1)，表示中心点位于矩形的左上角。
*/
	Mat element = getStructuringElement(MORPH_RECT, Size(1, 2));

// 定义了一个二维向量，用于存储多个轮廓（contour）。每个轮廓由一系列点（Point）组成，因此内部向量的元素类型为Point。
	vector<vector<Point>> contours;
//定义了一个一维向量，用于存储单个轮廓的点（Point）
	vector<Point> contour;
//定义了一个一维向量，用于存储轮廓之间的层次关系。每个元素是一个四元组（Vec4i），表示轮廓之间的父子关系。
	vector<Vec4i> hierarchy;
/*dstImage：一个二值图像，其中非零像素表示对象，零像素表示背景。
contours：一个二维向量，用于存储找到的轮廓。每个轮廓由一系列点（Point）组成，表示对象的边界。
hierarchy：一个一维向量，用于存储轮廓之间的层次关系。每个元素是一个四元组（Vec4i），表示轮廓之间的父子关系。
RETR_EXTERNAL：一个标志，表示只检索最外层的轮廓。
CHAIN_APPROX_SIMPLE：一个标志，表示使用简单多边形逼近轮廓，即只保留轮廓的拐点。
Point()：一个空的点对象，表示不使用偏移量。*/
	findContours(dstImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
	//轮廓绘制
	Mat drawImage;
	double width = srcImage.cols;
	double height = srcImage.rows;


	cout << contours.size() << endl;
	for (int t = 0; t < contours.size(); t++)
	{
/*
arcLength 是一个函数，用于计算轮廓的弧长。
contours[t] 是一个参数，表示要计算弧长的轮廓。这里的t是一个索引，表示轮廓在contours向量中的位置。
true 是一个布尔型参数，表示是否使用轮廓的近似多边形来计算弧长。如果设置为true，则使用轮廓的近似多边形来计算弧长，这样可以提高计算速度。
综上，这行代码的作用是计算轮廓contours[t]的弧长，并将结果存储在变量length2中。通过使用这个函数，可以方便地计算轮廓的周长。*/
		double length2 = arcLength(contours[t], true);
		cout << "第" << t << "个轮廓长度=" << length2 << endl;
		if (length2 > 800) {
			contour = contours[t];
//Scalar(250, 120, 220) 是一个构造函数，用于创建一个具有指定分量值的Scalar对象。这里的参数分别表示红色、绿色和蓝色的分量值，范围是0到255。
			Scalar color = Scalar(250, 120, 220);
/*
在源图像 srcImage 上绘制轮廓。

drawContours 是一个函数，用于在图像上绘制轮廓。
srcImage 是输入的源图像，即要在其上绘制轮廓的图像。
contours 是一个向量，存储了要绘制的轮廓信息。
t 是一个整数，表示要绘制的轮廓的索引。
color 是一个标量（Scalar），表示绘制轮廓的颜色。
2 是一个整数，表示轮廓的线型。这里设置为2表示实心线。
8 是一个整数，表示轮廓的连接方式。这里设置为8表示闭合轮廓。
hierarchy 是一个向量，存储了轮廓之间的层次关系。
0 是一个整数，表示偏移量。这里设置为0表示不使用偏移量。
Point(0, 0) 是一个点对象，表示绘制轮廓时的起始点。这里设置为 (0, 0) 表示从图像的左上角开始绘制。
综上，这行代码的作用是在源图像 srcImage 上绘制索引为 t 的轮廓，使用指定的颜色、线型和连接方式，并考虑轮廓之间的层次关系和偏移量。
*/
			drawContours(srcImage, contours, t, color, 2, 8, hierarchy, 0, Point(0, 0));
		}
	}
	namedWindow("output", WINDOW_AUTOSIZE);
	imshow("output", srcImage);
/*
RotatedRect rr = minAreaRect(contour); 这行代码的作用是计算给定轮廓（contour）的最小外接矩形。
minAreaRect() 是一个函数，用于计算给定轮廓的最小外接矩形。
contour 是一个表示轮廓的变量，通常是一个包含点集的向量。
RotatedRect 是一个类，表示旋转的矩形。它包含了矩形的中心点、宽度、高度和旋转角度等信息。
rr 是一个 RotatedRect 类型的变量，用于存储计算出的最小外接矩形。*/
	RotatedRect rr = minAreaRect(contour);
/*
Point2f points[4]; 是一行代码，用于声明一个名为 points 的数组，该数组包含四个 Point2f 类型的元素。

Point2f 是一个数据类型，通常表示二维平面上的点。它可能具有两个浮点数成员变量，分别表示点的 x 坐标和 y 坐标。

通过声明 Point2f points[4];，我们可以创建一个大小为 4 的数组，其中每个元素都是 Point2f 类型的对象。这意味着我们可以存储四个二维平面上的点，并使用索引访问它们。*/
	Point2f points[4];
	rr.points(points);

	Point2f points2[4];
	cout << "center.x = " << rr.center.x << "center.y = " << rr.center.y << endl;
//对Points[4]数组中存储的点进行排序，排序后points2数组中的元素顺序为：左上角、左下角、右下角、右上角。
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
	for (int i = 0; i < 4; i++) {
		cout << points2[i].x << "  " << points2[i].y << endl;
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
//getAffineTransform(srcPoints, dstPoints) 函数用于计算从源图像中的点到目标图像中的点的仿射变换矩阵。
//其中，srcPoints 和 dstPoints 分别是源图像和目标图像中的三个对应点的坐标数组。
	Mat M1 = getAffineTransform(srcPoints, dstPoints);
/*
warpAffine(srcImage, srcImage, M1, srcImage.size()) 函数用于将源图像进行仿射变换，并将结果保存在源图像中。
其中，srcImage 是源图像，M1 是仿射变换矩阵，srcImage.size() 是源图像的尺寸。*/
	warpAffine(srcImage, srcImage, M1, srcImage.size());

	imshow("正", srcImage);

	Rect rect1(points[1].x, points[1].y, chang, kuan);
	Mat after_1;
	srcImage(rect1).copyTo(after_1);
	imshow("蓝色指示牌1", after_1);
	imwrite("result3.png", after_1);

	waitKey(0);
	return 0;
}