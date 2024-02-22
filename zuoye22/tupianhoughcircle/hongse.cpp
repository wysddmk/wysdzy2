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

    Mat image = imread("k.png");
    Mat image2;
    image.copyTo(image2);
    resize(image, image, Size(3 * image.cols, 3 * image.rows), 3, 3, 0);
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);

    Scalar lower_red1(170, 70, 50);
    Scalar upper_red1(180, 255, 255);
    Scalar lower_red2(0, 70, 50);
    Scalar upper_red2(10, 255, 255);

    Mat red_mask1, red_mask2;
    inRange(hsv, lower_red1, upper_red1, red_mask1);

    inRange(hsv, lower_red2, upper_red2, red_mask2);

    Mat red_mask = red_mask1 + red_mask2;
    Mat gray;
    medianBlur(red_mask, gray, 3);

    vector<Vec3f> circles;
    double a = 3;
    double b = 100;
    double c = 200;
    double d = 200;
    int e = 0;
    int f = 100;
    HoughCircles(gray, circles, HOUGH_GRADIENT, a, b, c, d, e, f);

    Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
    int r = 2.5 * cvRound(circles[0][2]);
    circle(image, center, r, Scalar(255, 0, 0), 5, 8, 0);

    resize(image, image, Size(round(0.333 * image.cols), round(0.333 * image.rows)), 0.333, 0.333, 0);
    Mat hsv2, blue;
    cvtColor(image, hsv2, COLOR_BGR2HSV);

    Scalar lower_blue(120, 50, 50);
    Scalar upper_blue(130, 255, 255);
    inRange(hsv2, lower_blue, upper_blue, blue);

    vector<vector<Point>> contours;
    findContours(blue, contours, 0, 1);

    for (unsigned long i = 0; i < contours.size(); i++)
    {
        //
        RotatedRect juxing = minAreaRect(contours[i]);
        Point2f points[4];
        juxing.points(points);
        for (int i = 0; i < 4; i++)
        {
            if (i == 3)
            {
                line(image2, points[i], points[0], Scalar(0, 255, 0), 2, 8, 0);
                break;
            }
            line(image2, points[i], points[i + 1], Scalar(0, 255, 0), 2, 8, 0);
        }
    }
    imshow("result", image2);
    waitKey(0);
}