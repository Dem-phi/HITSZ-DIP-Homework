#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#define READIMAGE_ONLY
#ifndef READIMAGE_ONLY
#include <geometry_msgs/Twist.h>
#endif

using namespace cv;
using namespace std;

void show_histogram(Mat& img)
{
    //为计算直方图配置变量  
    //首先是需要计算的图像的通道，就是需要计算图像的哪个通道（bgr空间需要确定计算 b或g货r空间）  
    int channels = 0;
    //然后是配置输出的结果存储的 空间 ，用MatND类型来存储结果  
    MatND dstHist;
    //接下来是直方图的每一个维度的 柱条的数目（就是将数值分组，共有多少组）  
    int histSize[] = { 256 };       //如果这里写成int histSize = 256;   那么下面调用计算直方图的函数的时候，该变量需要写 &histSize  
    //最后是确定每个维度的取值范围，就是横坐标的总数  
    //首先得定义一个变量用来存储 单个维度的 数值的取值范围  
    float midRanges[] = { 0, 256 };
    const float *ranges[] = { midRanges };

    calcHist(&img, 1, &channels, Mat(), dstHist, 1, histSize, ranges, true, false);

    //calcHist  函数调用结束后，dstHist变量中将储存了 直方图的信息  用dstHist的模版函数 at<Type>(i)得到第i个柱条的值  
    //at<Type>(i, j)得到第i个并且第j个柱条的值  

    //开始直观的显示直方图——绘制直方图  
    //首先先创建一个黑底的图像，为了可以显示彩色，所以该绘制图像是一个8位的3通道图像  
    Mat drawImage = Mat::zeros(Size(256, 256), CV_8UC3);
    //因为任何一个图像的某个像素的总个数，都有可能会有很多，会超出所定义的图像的尺寸，针对这种情况，先对个数进行范围的限制  
    //先用 minMaxLoc函数来得到计算直方图后的像素的最大个数  
    double g_dHistMaxValue;
    minMaxLoc(dstHist, 0, &g_dHistMaxValue, 0, 0);
    //将像素的个数整合到 图像的最大范围内  
    //遍历直方图得到的数据  
    for (int i = 0; i < 256; i++)
    {
        int value = cvRound(dstHist.at<float>(i) * 256 * 0.9 / g_dHistMaxValue);

        line(drawImage, Point(i, drawImage.rows - 1), Point(i, drawImage.rows - 1 - value), Scalar(255, 255, 255));
    }

    imshow("【原图直方图】", drawImage);
}

//直方图绘制函数，参数vector<int> nums 是灰度图片256级灰度的像素个数
void drawHist(vector<int> nums)
{
    Mat hist = Mat::zeros(600, 800, CV_8UC3);
    auto Max = max_element(nums.begin(), nums.end()); //max迭代器类型,最大数目
    putText(hist, "Histogram", Point(150, 100), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255));
    //*********绘制坐标系************//
    Point o = Point(100, 550);
    Point x = Point(700, 550);
    Point y = Point(100, 150);
    //x轴
    line(hist, o, x, Scalar(255, 255, 255), 2, 8, 0);
    //y轴
    line(hist, o, y, Scalar(255, 255, 255), 2, 8, 0);

    //********绘制灰度曲线***********//
    Point pts[256];
    //生成坐标点
    for (int i = 0; i < 256; i++)
    {
        pts[i].x = i * 2 + 100;
        pts[i].y = 550 - int(nums[i] * (300.0 / (*Max))); //归一化到[0, 300]
        //显示横坐标
        if ((i + 1) % 16 == 0)
        {
            string num = format("%d", i + 1);
            putText(hist, num, Point(pts[i].x, 570), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
        }
    }
    //绘制线
    for (int i = 1; i < 256; i++)
    {
        line(hist, pts[i - 1], pts[i], Scalar(0, 255, 0), 2);
    }
    //显示图像
    imshow("直方图", hist);
}

Mat equalize_hist(Mat& input)
{
    int gray[256] = { 0 };  //记录每个灰度级别下的像素个数
    double gray_prob[256] = { 0 };  //记录灰度分布密度
    double gray_distribution[256] = { 0 };  //记录累计密度
    int gray_equal[256] = { 0 };  //均衡化后的灰度值

    int gray_sum = 0;  //像素总数   
    Mat output = input.clone();
    gray_sum = input.cols * input.rows;

    //统计每个灰度下的像素个数
    for (int i = 0; i < input.rows; i++)
    {
        uchar* p = input.ptr<uchar>(i);
        for (int j = 0; j < input.cols; j++)
        {
            int vaule = p[j];
            gray[vaule]++;
        }
    }


    //统计灰度频率
    for (int i = 0; i < 256; i++)
    {
        gray_prob[i] = ((double)gray[i] / gray_sum);
    }

    //计算累计密度
    gray_distribution[0] = gray_prob[0];
    for (int i = 1; i < 256; i++)
    {
        gray_distribution[i] = gray_distribution[i-1] +gray_prob[i];
    }

    //重新计算均衡化后的灰度值，四舍五入。参考公式：(N-1)*T+0.5
    for (int i = 0; i < 256; i++)
    {
        gray_equal[i] = (uchar)(255 * gray_distribution[i] + 0.5);
    }


    //直方图均衡化,更新原图每个点的像素值
    for (int i = 0; i < output.rows; i++)
    {
        uchar* p = output.ptr<uchar>(i);
        for (int j = 0; j < output.cols; j++)
        {
            p[j] = gray_equal[p[j]];
        }
    }
    vector<int> nums_dst(256);
    for (int i = 0; i < output.rows; i++)
    {
        uchar *p = output.ptr<uchar>(i);
        for (int j = 0; j < output.cols; j++)
        {
            nums_dst[p[j]]++;
        }
    }
    drawHist(nums_dst);
    imshow("均衡化后", output);
    return output;
}

void openCVHist(const Mat src)
{
    //需要计算图像的哪个通道（bgr空间需要确定计算 b或g或r空间）
    const int channels[1] = {0};
    //直方图的每一个维度的 柱条的数目（就是将灰度级分组）
    int histSize[] = {256}; //如果这里写成int histSize = 256; 那么下面调用计算直方图的函数的时候，该变量需要写 &histSize
    //定义一个变量用来存储 单个维度 的数值的取值范围
    float midRanges[] = {0, 256};
    //确定每个维度的取值范围，就是横坐标的总数
    const float *ranges[] = {midRanges};
    //输出的结果存储的 空间 ，用MatND类型来存储结果
    MatND dstHist;

    calcHist(&src, 1, channels, Mat(), dstHist, 1, histSize, ranges, true, false);
    //calcHist  函数调用结束后，dstHist变量中将储存了直方图的信息, 用dstHist的模版函数 at<Type>(i)得到第i个柱条的值  at<Type>(i, j)得到第i个并且第j个柱条的值
    //首先先创建一个黑底的图像，为了可以显示彩色，所以该绘制图像是一个8位的3通道图像
    Mat drawImage = Mat::zeros(Size(256, 256), CV_8UC3);

    //一个图像的某个灰度级的像素个数（最多为图像像素总数），可能会超过显示直方图的所定义的图像的尺寸，因此绘制直方图的时候，让直方图最高的地方只有图像高度的90%来显示
    //先用minMaxLoc函数来得到计算直方图后的像素的最大个数
    double g_dHistMaxValue;
    minMaxLoc(dstHist, 0, &g_dHistMaxValue, 0, 0);

    //遍历直方图得到的数据
    for (int i = 0; i < 256; i++)
    {
        int value = cvRound(256 * 0.9 * (dstHist.at<float>(i) / g_dHistMaxValue));
        line(drawImage, Point(i, drawImage.rows - 1), Point(i, drawImage.rows - 1 - value), Scalar(255, 0, 0));
    }
    imshow("OpenCVHist", drawImage);
    waitKey(0);
}

//计算直方图，统计各灰度级像素个数
void calHist(const string img)
{
    Mat src, grey;
    //读取图象
    src = imread(img);
    if (!src.data)
    {
        cout << "Image： " + img + " 读取失败" << endl;
        return;
    }
    //先转为灰度图
    cvtColor(src, grey, COLOR_BGR2GRAY);
    imshow("灰度图", grey);
    //计算各灰度级像素个数
    vector<int> nums(256);
    for (int i = 0; i < grey.rows; i++)
    {
        uchar *p = grey.ptr<uchar>(i);
        for (int j = 0; j < grey.cols; j++)
        {
            nums[p[j]]++;
        }
    }
    drawHist(nums);
}

int main(int argc, char **argv)
{
    string img = "/home/demphi/ros/dip_ws/src/image_pkg/src/cyc.jpeg";

    Mat src, grey,new_grey;
    src = imread(img);
    calHist(img);
    cvtColor(src, grey, COLOR_BGR2GRAY);
    imshow("原灰度图", src);
    Mat new_img = equalize_hist(grey);
    waitKey(0);
    return 0;
}