#include "v1-ball-detector/v1_ball_detector.h"

int main(int argc, char** argv){

//    ros::init(argc, argv, "v1_ball_detector_node");
//    BallDetector_v1 BallDetector_v1;
//    ros::spin();
//    return 0;

    ros::init(argc,argv,"v1_ball_detector_node");
    BallDetector_v1 ballDetector_v1;
    ros::Rate loop_rate(30);
    while(ros::ok()){
//        std::cout << "ROS";
        ballDetector_v1.process();
        ros::spinOnce();
//        ros::spin();
        loop_rate.sleep();
    }
    return 0;
}