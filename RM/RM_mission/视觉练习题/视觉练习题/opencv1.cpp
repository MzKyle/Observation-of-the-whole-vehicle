#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

void find_contours_and_centers(const std::string& image_path, const std::string& output_folder) {
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "Could not open or find the image" << std::endl;
        return;
    }

    cv::Mat gray, blurred, edges;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::Canny(blurred, edges, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > 100) {
            cv::Moments target = cv::moments(contours[i]);
            int x = static_cast<int>(target.m10 / target.m00);
            int y = static_cast<int>(target.m01 / target.m00);

            cv::drawContours(image, contours, static_cast<int>(i), cv::Scalar(0, 255, 0), 2);
            cv::circle(image, cv::Point(x, y), 7, cv::Scalar(255, 0, 0), -1);

            cv::imshow("img", image);
            cv::waitKey(0);
            cv::destroyAllWindows();

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contours[i], center, radius);
            cv::Rect bounding_rect(static_cast<int>(center.x - radius), static_cast<int>(center.y - radius), static_cast<int>(2 * radius), static_cast<int>(2 * radius));
            cv::Mat coin = image(bounding_rect);

            std::string output_path = output_folder + "/coins" + std::to_string(i) + ".png";
            cv::imwrite(output_path, coin);
            cv::imwrite(output_path, image);

            cv::imshow("img", coin);
            cv::waitKey(0);
            cv::destroyAllWindows();
        }
    }
}

int main() {
    find_contours_and_centers("question1.jpg", "coins");
    return 0;
}
