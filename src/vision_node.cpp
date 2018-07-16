#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "classifier.hpp"
#include "traffic_sign_recognition/sign.h"
#include "std_msgs/String.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "classification");
    ros::NodeHandle n;

    classifier visual;

    cv::HOGDescriptor hog(cv::Size(64, 64),
                      cv::Size(32, 32),
                      cv::Size(16, 16),
                      cv::Size(32, 32),
                      9, 1, -1, 0, 0.2,
                      1, 64, 1);
    ROS_INFO_STREAM("HOG Descriptor created");

    visual.imagen = cv::Mat::zeros(640, 480, CV_8UC3);
    std::vector<cv::Mat> trainImgs;
    std::vector<int> trainLabels;
    cv::Mat trainHOG;

    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    ROS_INFO_STREAM("Support Vector Machine Created");
    double area;

    cv::Mat img_denoise;
    std::vector<cv::Mat> imgs_mser(100);
    cv::Mat testHOG;

    ROS_INFO_STREAM("created testHOG");
    ros::Subscriber sub = n.subscribe("/new_image_raw",
        1, &classifier::imageCallback, &visual);
    
    // Custom message Publisher
    ros::Publisher signPub = n.advertise<traffic_sign_recognition::sign>(
        "traffic", 1);
    traffic_sign_recognition::sign msg;
    //String sss="Sign Detected!"
    ros::Publisher iothubPub = n.advertise<std_msgs::String>("iothub", 1);
    ///////// TRAINING ////////////
    std_msgs::String iotmsg;
    std::stringstream ss;       
    ROS_INFO_STREAM("CREATED PUBLISHERS AND SUBSRIBERS");
    int trained = visual.trainStage(hog, svm, trainImgs, trainLabels);
    ROS_INFO_STREAM("TRAIN STAGE");
    
    //////// CLASSIFICATION /////////
    ROS_INFO_STREAM("Detection and Classification started...");
    while (ros::ok()) {
        if (!visual.imagen.empty()) {
            // Denoise image with gaussian blur
            img_denoise = visual.deNoise(visual.imagen);

            // Get the detections using MSER
            imgs_mser = visual.MSER_Features(visual.imagen, area);

            // If there are detection in the frame:
            if (imgs_mser.size() != 0) {
                // HOG features of detections
                testHOG = visual.HOG_Features(hog, imgs_mser);

                // Evaluate using the SVM
                visual.traffic_sign = visual.SVMTesting(svm, testHOG);

                // Publish the type of sign through message
                msg.area = area;
                msg.sign_type = visual.traffic_sign;
                signPub.publish(msg);
		ROS_INFO_STREAM("Sign Detected! ");
		ROS_INFO("%f", msg.sign_type);
                ss << msg.sign_type;
		iotmsg.data = ss.str();
		ss.str("");
		iothubPub.publish(iotmsg);	
		ss.str(std::string());
		imgs_mser.clear();
            }
            // Visualization of the robot view with all the detections
            //int flagviz = visual.visualization();
        }
        ros::spinOnce();
    }
    return 0;
}
