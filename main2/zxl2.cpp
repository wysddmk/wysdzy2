#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include <algorithm>
#include <cmath>
#include <ctype.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp> //OpenCV highgui 模块头文件 ~
#include <opencv2/imgproc/imgproc.hpp> //OpenCV 图像处理头文件
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
using namespace std;
using namespace cv;

vector<vector<Point>> prepare(Mat &image)
{
    Mat gray, gray1, gray2;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    threshold(gray, gray1, 25, 255, 0);
    Mat kernel = getStructuringElement(1, Size(3, 3));
    dilate(gray1, gray1, kernel);
    GaussianBlur(gray1, gray1, Size(3, 3), 10, 20);
    Canny(gray1, gray2, 90, 180, 3);
    vector<vector<Point>> contours;
    findContours(gray1, contours, 0, 1);
    return contours;
}

cv::Point2f calculateThirdPoint(const cv::Point2f &P1, const cv::Point2f &P2)
{
    // 计算向量 P2P1
    cv::Point2f vectorP2P1 = P1 - P2;

    // 计算 P2P1 的模长（即两点之间的距离）
    float magnitudeP2P1 = cv::norm(vectorP2P1);
    // 单位化向量 P2P1
    cv::Point2f unitVectorP2P1 = vectorP2P1 / magnitudeP2P1;
    float distanceFromP2 = 1.315 * magnitudeP2P1;
    // 计算第三点的位置，即在单位向量方向上移动 distanceFromP2 的距离
    cv::Point2f P3 = P2 + unitVectorP2P1 * distanceFromP2;

    return P3;
}

cv::Point3f pixelToWorld(const cv::Point2d &pixel, float d, cv::Mat &K, cv::Mat &R, cv::Mat &t)
{
    int u = pixel.x;
    int v = pixel.y;

    // 相机内参矩阵的逆(Inverse of intrinsic matrix)
    cv::Mat K_inv = K.inv();

    // 像素坐标到相机坐标系下的转换
    cv::Mat pixelCoords = (cv::Mat_<float>(3, 1) << u, v, 1);
    cv::Mat cameraCoords = K_inv * pixelCoords * d;

    // 相机坐标系到世界坐标系下的转换
    cv::Mat worldCoords = R.inv() * (cameraCoords - t);

    // cv::Mat to cv::Point3f
    cv::Point3f worldPoint;
    worldPoint.x = worldCoords.at<float>(0, 0);
    worldPoint.y = worldCoords.at<float>(1, 0);
    worldPoint.z = worldCoords.at<float>(2, 0);

    return worldPoint;
}

int main()
{
    Mat img = imread("c.png");

    vector<vector<Point>> contours1;
    contours1 = prepare(img);

    Mat imgR = imread("R.png");
    vector<vector<Point>> contourR;
    contourR = prepare(imgR);
    VideoCapture video("big.mp4");
    while (1)
    {
        Mat image2;
        video >> image2;

        vector<vector<Point>> contours2;
        contours2 = prepare(image2);

        double minArea = 3500.0; // 设置最小面积阈值
        vector<vector<Point>> filteredContours;
        vector<vector<Point>> R;
        for (size_t i = 0; i < contours2.size(); i++)
        {

            double area = contourArea(contours2[i]);
            RotatedRect rect1 = minAreaRect(contours2[i]);
            float aspectRatio = rect1.size.width / rect1.size.height;
            if (area > minArea && 0.35 < aspectRatio && aspectRatio < 2)
            {
                filteredContours.push_back(contours2[i]);
            }
            double distR = matchShapes(contours2[i], contourR[0], 1, 0);
            if (distR < 0.1 && 0.8 < aspectRatio && aspectRatio < 1.2 && area > 400)
            {
                R.push_back(contours2[i]);
            }
        }
        vector<vector<Point>> finishcontours;
        for (size_t i = 0; i < filteredContours.size(); i++)
        {

            double dist;
            dist = matchShapes(filteredContours[i], contours1[0], 1, 0);
            if (dist > 0.225)
            {
                finishcontours.push_back(filteredContours[i]);
            }
        }
        Mat_<double> intrinstic_matrix = (cv::Mat_<double>(3, 3) << 2351.55699, 0, 716.95746, 0, 2349.89714, 547.81957, 0, 0, 1);
        Mat_<double> distortion_vec = (cv::Mat_<double>(1, 5) << -0.104800, 0.140335, -0.000984, -0.000920, 0.0000000);
        Mat rvec1, tvec1;
        for (unsigned long i = 0; i < finishcontours.size(); i++)
        {
            //
            RotatedRect juxing = minAreaRect(finishcontours[i]);
            Point2f Jcenter = juxing.center;
            if (R.size() > 0)
            {
                RotatedRect RJ = minAreaRect(R[0]);
                Point2f Rresult = RJ.center;
                Point2f result = calculateThirdPoint(Jcenter, Rresult);
                circle(image2, result, 2, Scalar(0, 255, 0), 8, 3, 0);
                Point2f points[4];
                RJ.points(points);
                vector<Point2f> Point2D;
                for (int i = 0; i < 4; i++)
                {
                    Point2D.push_back(points[i]);
                }

                vector<cv::Point3f> Points3D;

                Points3D.push_back(cv::Point3f(-30, -30, 0));
                Points3D.push_back(cv::Point3f(-30, 30, 0));
                Points3D.push_back(cv::Point3f(30, 30, 0));
                Points3D.push_back(cv::Point3f(30, 30, 0));
                solvePnP(Points3D, Point2D, intrinstic_matrix, distortion_vec, rvec1, tvec1, false);
                Mat rvec1_;
                Rodrigues(rvec1, rvec1_);

                Point3f result3f = pixelToWorld(result, 1.0, intrinstic_matrix, rvec1_, tvec1);

                string coordinatesText = "(" + to_string(static_cast<int>(result3f.x)) + "," + to_string(static_cast<int>(result3f.y)) + "," + to_string(static_cast<int>(result3f.z)) + ")";

                // 设置文本显示的位置
                cv::Point textOrg(25, 65); // 距离左上角一定偏移量，避免贴边

                // 设置文本字体和大小
                int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                double fontScale = 1;
                int thickness = 2;

                // 在帧上绘制文本
                cv::putText(image2, coordinatesText, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);
            }
            drawContours(image2, finishcontours, i, Scalar(0, 0, 255), 2, 8);
        }

        imshow("result", image2);

        waitKey(50);
    }
}
