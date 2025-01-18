#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

string get_shape_name(const vector<Point>& contour) {
    string shape_name = "unidentified";
    double peri = arcLength(contour, true);
    vector<Point> approx;
    approxPolyDP(contour, approx, 0.04 * peri, true);
    int n = approx.size();
    if (n == 3) {
        shape_name = "triangle";
    } else if (n == 4) {
        double aspect_ratio = static_cast<double>(boundingRect(approx).width) / boundingRect(approx).height;
        shape_name = (aspect_ratio >= 0.95 && aspect_ratio <= 1.05) ? "square" : "rectangle";
    } else if (n == 5) {
        shape_name = "pentagon";
    } else {
        shape_name = "circle";
    }
    return shape_name;
}

void process_image(Mat& image, int area_threshold) {
    Mat clone = image.clone();
    vector<vector<Point>> contours;
    findContours(clone, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        if (area < area_threshold) {
            string shape_name = get_shape_name(contour);
            Moments M = moments(contour);
            int cX = static_cast<int>(M.m10 / M.m00);
            int cY = static_cast<int>(M.m01 / M.m00);
            drawContours(clone, vector<vector<Point>>{contour}, -1, Scalar(100, 100, 100), 1);
            putText(clone, shape_name, Point(cX - 20, cY), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(100, 100, 100), 2);
        }
    }
    imshow("Shapes", clone);
}

static void on_trackbar(int val, void* userdata) {
    Mat* image = static_cast<Mat*>(userdata);
    process_image(*image, val);
}

int main() {
    Mat image = imread("RM_misson/shape.jpg");
    if (image.empty()) {
        cout << "Could not open or find the image" << endl;
        return -1;
    }
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    Mat opening;
    morphologyEx(gray, opening, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(2, 2)), Point(-1, -1), 1);
    Mat blurred;
    GaussianBlur(opening, blurred, Size(1, 1), 0);
    Mat thresh;
    threshold(blurred, thresh, 250, 255, THRESH_BINARY_INV);
    Mat edged;
    Canny(thresh, edged, 50, 150);
    namedWindow("Shapes", WINDOW_AUTOSIZE);
    createTrackbar("Area Threshold", "Shapes", &edged, 100000, on_trackbar, &image);
    on_trackbar(0, &image);
    waitKey(0);
    return 0;
}
