#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include <ctype.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp> //OpenCV highgui 模块头文件 ~
#include <opencv2/imgproc/imgproc.hpp> //OpenCV 图像处理头文件

using namespace std;
using namespace cv;

int main()
{

    Mat image = imread("b.png");

    Mat image2;
    cvtColor(image, image2, 10);
    resize(image2, image2, Size(2 * image.cols, 2 * image.rows), 3, 3, 0);
    Mat image3(Size(image2.cols, image2.rows), CV_8UC1, Scalar(0));
    Mat blue_mask;
    threshold(image2, blue_mask, 25, 255, 0);
    Mat gray;
    medianBlur(blue_mask, gray, 3);
    vector<Vec3f> circles;
    double a = 2;
    double b = 100;
    double c = 200;
    double d = 150;
    int e = 0;
    int f = 150;
    HoughCircles(gray, circles, HOUGH_GRADIENT, a, b, c, d, e, f);
    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int r = 1.5 * cvRound(circles[i][2]);
        circle(image3, center, r, Scalar(255, 255, 255), 5, 8, 0);
    }
    resize(image3, image3, Size(round(0.5 * image3.cols), round(0.5 * image3.rows)), 0.5, 0.5, 0);

    vector<vector<Point>> contours;
    findContours(image3, contours, 0, 1);
    for (unsigned long i = 0; i < contours.size(); i++)
    {

        RotatedRect juxing = minAreaRect(contours[i]);
        Point2f points[4];
        juxing.points(points);
        for (int i = 0; i < 4; i++)
        {
            if (i == 3)
            {
                line(image, points[i], points[0], Scalar(255, 255, 255), 2, 8, 0);
                break;
            }
            line(image, points[i], points[i + 1], Scalar(255, 255, 255), 2, 8, 0);
        }
    }
    imshow("result", image);
    waitKey(0);
}