#pragma once

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "kri_2021/CoordinateBall.h"
#include "kri_2021/StateBall.h"

#define DEBUG
#define PARAM

#ifndef _BALL_DETECTOR
#define _BALL_DETECTOR
#endif

class BallDetector_v1{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImage cv_img_pubs_;

    ros::Publisher ball_state_tilt_pub;
    ros::Publisher ball_state_pan_pub;

    ros::Publisher ball_coordinate_pub;
    ros::Publisher state_ball_kick_pub;

    float lens_focal_camera;
    int brightness;
    int contrast;

    cv::Mat in_img_;
    cv::Mat out_img_;

    ros::Subscriber cam_info_sub_;

    sensor_msgs::CameraInfo cam_info_msg_;
    int encode_mode = 1;
    unsigned int img_encoding_;

    cv::Mat& setInputImage();

    void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfo &_msg);
    void imageProcess(const cv::Mat &src);

    static bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2);

    kri_2021::CoordinateBall coordinate_ball;

    inline void pubBallStateTilt(bool status, int pos_y, int last_pos_y);
    inline void pubBallStatePan(bool status, int pos_y, int last_pos_y);
    inline void pubBallStateKick(bool status);

    kri_2021::StateBall state_ball_tilt;
    kri_2021::StateBall state_ball_pan;
    kri_2021::StateBall state_ball_kick;

    float real_distance;
    void getRealDistance(){}
    void findContour(){}

    cv::Mat resizeImage(const cv::Mat &src, int width, int height);

    void blobDetection(){}
    void morphologyOp(){}
    void drawContour(){}

    class BallParam{
    public:
        float h_ranges[2];
        float s_ranges[2];
        float v_ranges[2];

        float h_ranges_field[2];
        float s_ranges_field[2];
        float v_ranges_field[2];

        float h_ranges_backdrop[2];
        float s_ranges_backdrop[2];
        float v_ranges_backdrop[2];

        int flag = 0,
                pos_y = -1,
                pos_x = -1,
                pos_kick = 1,
                last_pos_y = -1,
                last_pos_x = -1;

        int min_radius = 5;

        const float realBallWidth = 14.6;
        BallParam():h_ranges{0, 255}, s_ranges{1, 255}, v_ranges{1, 255},
                    h_ranges_field{35, 170}, s_ranges_field{98, 123}, v_ranges_field{108, 133},
                    h_ranges_backdrop{15, 236}, s_ranges_backdrop{112, 141}, v_ranges_backdrop{123, 144}{}
    }ball_param_;
public:
    BallDetector_v1();
    ~BallDetector_v1();
    void process();
};