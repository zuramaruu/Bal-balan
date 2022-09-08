#include "v1-ball-detector/v1_ball_detector.h"

BallDetector_v1::BallDetector_v1():
        nh_(ros::this_node::getName()),
        it_(this->nh_),
        image_sub_(it_.subscribe("/usb_cam/image_raw", 1, &BallDetector_v1::imageCallBack, this)),
        ball_coordinate_pub(nh_.advertise <kri_2021::CoordinateBall> ("KRSBI/image_processing/deteksi_bola/coordinate/", 10)),
        ball_state_tilt_pub(nh_.advertise <kri_2021::StateBall> ("KRSBI/image_processing/deteksi_bola/ball_state_tilt", 10)),
        ball_state_pan_pub(nh_.advertise <kri_2021::StateBall> ("KRSBI/image_processing/deteksi_bola/ball_state_pan", 10)),
        cam_info_sub_(nh_.subscribe("/usb_cam/camera_info", 100, &BallDetector_v1::cameraInfoCallback, this)){
}

BallDetector_v1::~BallDetector_v1(){
    cv::destroyAllWindows();
}

void BallDetector_v1::process(){
    if(cv_ptr == nullptr)return; //-- Important method;
    setInputImage() = cv_ptr->image;
#ifdef DEBUG
    cv::imshow("FRAME", in_img_);
#endif
    this->imageProcess(in_img_);
    cv::waitKey(1);
}

cv::Mat& BallDetector_v1::setInputImage(){
    return in_img_;
}

void BallDetector_v1::imageCallBack(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void BallDetector_v1::cameraInfoCallback(const sensor_msgs::CameraInfo &_msg){
    //cam_info_msg_ = *_msg;

//    ROS_INFO("CHECK...");
}

bool BallDetector_v1::compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2){
    return contourArea(contour1)> contourArea(contour2);
}

void BallDetector_v1::imageProcess(const cv::Mat &src){
    brightness = 0;contrast = 0;
    cv::Mat blur_image;
    GaussianBlur(src, blur_image, cv::Size(11, 11), 0);

    cv::Mat YUV_masking;
    cv::cvtColor(blur_image, YUV_masking, cv::COLOR_BGR2YUV);
    cv::inRange(YUV_masking, cv::Scalar(ball_param_.h_ranges_field[0],
                                        ball_param_.s_ranges_field[0],
                                        ball_param_.v_ranges_field[0]),
                cv::Scalar(ball_param_.h_ranges_field[1],
                           ball_param_.s_ranges_field[1],
                           ball_param_.v_ranges_field[1]), YUV_masking);
    cv::dilate( YUV_masking, YUV_masking,
                cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(3, 3)));

    cv::Mat BACKDROP_masking;
    cv::cvtColor(blur_image, BACKDROP_masking, cv::COLOR_BGR2YCrCb);
    cv::inRange(BACKDROP_masking, cv::Scalar(ball_param_.h_ranges_backdrop[0],
                                             ball_param_.s_ranges_backdrop[0],
                                             ball_param_.v_ranges_backdrop[0]),
                cv::Scalar(ball_param_.h_ranges_backdrop[1],
                           ball_param_.s_ranges_backdrop[1],
                           ball_param_.v_ranges_backdrop[1]), BACKDROP_masking);
//        cv::morphologyEx(BACKDROP_masking, BACKDROP_masking, cv::MORPH_OPEN,
//                         cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 2);
    cv::morphologyEx(BACKDROP_masking, BACKDROP_masking, cv::MORPH_OPEN,
                     cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(3, 3)));
    cv::morphologyEx(BACKDROP_masking, BACKDROP_masking, cv::MORPH_CLOSE,
                     cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(3, 3)));

    cv::Mat BALL_masking;
    cv::bitwise_or(YUV_masking, BACKDROP_masking, BALL_masking);
    cv::morphologyEx(BALL_masking, BALL_masking, cv::MORPH_CLOSE,
                     cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(11, 11)));
    cv::bitwise_not(BALL_masking, BALL_masking);
    cv::morphologyEx(BALL_masking, BALL_masking, cv::MORPH_CLOSE,
                     cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(11, 11)));

    //-- Decision Making
//        std::vector<cv::Point> contours;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours( BALL_masking, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
#ifdef DEBUG
    cv::drawContours(this->in_img_, contours, -1, cv::Scalar(255, 255, 255), 2);
#endif
    //-- Find the contours
    if (contours.size() > 0) {

//        std::vector<int> sortIdx(contours.size());
//        std::vector<float> areas(contours.size());
//
//        for( int n = 0; n < (int)contours.size(); n++){
//            sortIdx[n] = n;
//            areas[n] = cv::contourArea(contours[n], false);
//        }
//        std::vector<std::vector<cv::Point>> sortedContours;
//        std::sort(contours.begin(), contours.end(), this->AreaCmp(areas));

        std::sort(contours.begin(), contours.end(), BallDetector_v1::compareContourAreas);
        for (int i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
        }

        std::vector <cv::Point> largestCnt = contours[contours.size() - 1];
        std::vector <cv::Point> smallestCnt = contours[0];
//        std::cout << "largestCnt = " << largestCnt << std::endl;
//        std::cout << "smallestCnt = " << smallestCnt << std::endl;

        cv::Moments M = cv::moments(largestCnt);

        cv::Point2f center;
        float R;

        cv::minEnclosingCircle(largestCnt, center, R);
#ifdef PARAM
        std::cout << "R = " << R << std::endl;
#endif
        unsigned int inFramePosX;
        unsigned int inFramePosY;

        double x_ball_filtered;
        double y_ball_filtered;
        try {
            inFramePosX = M.m10 / M.m00;
            inFramePosY = M.m01 / M.m00;
            int radius = sqrt((M.m00 / 255) / (22 / 7));
#ifdef PARAM
            std::cout << "Pos (" << inFramePosX << ", " << inFramePosY << ")" << std::endl;
            std::cout << "Radius = " << radius << std::endl;
#endif
        } catch (const std::exception &e) {
            ROS_ERROR("CONTOURS DIDN'T FIND");
        }

        if (R > ball_param_.min_radius) {
#ifdef DEBUG
            cv::circle(in_img_, cv::Point(inFramePosX, inFramePosY), R, cv::Scalar(0, 255, 255), 3);
            cv::circle(in_img_, cv::Point(inFramePosX, inFramePosY), 3, cv::Scalar(0, 0, 255), -1);
            cv::putText(in_img_, "BOLA", cv::Point(inFramePosX + 10, inFramePosY),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
            cv::putText(in_img_, "(" + std::to_string(inFramePosX) + ", " + std::to_string(inFramePosY) + ")",
                        cv::Point(inFramePosX + 10, inFramePosY + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
#endifs
            //-- Get the real distance
            lens_focal_camera = 347.101117382;
            real_distance = (ball_param_.realBallWidth * lens_focal_camera) / R;
#ifdef PARAM
            std::cout << "Width  = " << in_img_.cols << std::endl;
            std::cout << "Height = " << in_img_.rows << std::endl;
            std::cout << "Real Distance = " << real_distance << std::endl;
#endif
            x_ball_filtered = (inFramePosX / 640) * 2 - 1;
            y_ball_filtered = (inFramePosY / 480) * 2 - 1;
            ball_param_.flag = 1;
        } else {
            ball_param_.flag = 0;
        }

        //-- TILT --//
        if (inFramePosY > 85) {
            ball_param_.pos_y = 2;
        } else if (inFramePosY > (in_img_.rows - 85)) {
            ball_param_.pos_y = 1;
        } else {
            ball_param_.pos_y = 0;
        }

        //-- PAN --//
        if (inFramePosX < 100) {
            ball_param_.pos_x = 2;
        } else if (inFramePosX > (in_img_.cols - 100)) {
            ball_param_.pos_x = 1;
        } else {
            ball_param_.pos_x = 0;
        }

        //-- KICK DECISION --//
        if (inFramePosX < ((int) (in_img_.cols / 2))) {
            ball_param_.pos_kick = 0;
        } else {
            ball_param_.pos_kick = 1;
        }

        ball_param_.last_pos_y = ball_param_.pos_y;
        ball_param_.last_pos_x = ball_param_.pos_x;

        //-- finContours Method
        coordinate_ball.x_bola = float (x_ball_filtered);
        coordinate_ball.y_bola = float (y_ball_filtered);
        coordinate_ball.z_bola = float (real_distance);

        coordinate_ball.x_pixel = float (inFramePosX);
        coordinate_ball.y_pixel = float (inFramePosY);
        coordinate_ball.radius = float (R);
        ball_coordinate_pub.publish(coordinate_ball);
    }
}

void BallDetector_v1::pubBallStateTilt(bool status, int pos_y = -1,int last_pos_y = 1){
    if (status == 1){
        state_ball_tilt.bola_state = bool(true);
    }else{
        state_ball_tilt.bola_state = bool(false);
    }
    state_ball_tilt.bola_inFrame_Pos = std::to_string(pos_y);
    state_ball_tilt.last_bola_inFrame_Pos = std::to_string(last_pos_y);
    ball_state_tilt_pub.publish(state_ball_tilt);
}

void BallDetector_v1::pubBallStatePan(bool status, int pos_y = -1, int last_pos_y = -1){
    if (status == 1){
        state_ball_pan.bola_state = bool(true);
    }else{
        state_ball_pan.bola_state = bool(false);
    }
    state_ball_pan.bola_inFrame_Pos = std::to_string(pos_y);
    state_ball_pan.last_bola_inFrame_Pos = std::to_string(last_pos_y);
    ball_state_pan_pub.publish(state_ball_pan);
}

void BallDetector_v1::pubBallStateKick(bool status){
    if (status == 1){
        state_ball_kick.bola_state_kick = bool(true);
    }else{
        state_ball_kick.bola_state_kick = bool(false);
    }
    state_ball_kick_pub.publish(state_ball_kick);
}

cv::Mat BallDetector_v1::resizeImage(const cv::Mat &src, int width, int height){
    cv::Mat img_resized_;
    cv::resize(src, img_resized_, cv::Size (width, height), 0, 0, cv::INTER_AREA);
    return img_resized_;
}

