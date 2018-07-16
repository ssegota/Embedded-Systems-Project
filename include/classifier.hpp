#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"


class classifier {
 private:

    void loadTrainingImgs(std::vector<cv::Mat> &trainImgs,
        std::vector<int> &trainLabels);


    void SVMTraining(cv::Ptr<cv::ml::SVM> &svm,
        cv::Mat trainHOG, std::vector<int> trainLabels);

 public:
    cv::Mat imagen; 

    std::vector<cv::Rect> boxes;  

    float traffic_sign;  


    void imageCallback(const sensor_msgs::ImageConstPtr& msg);


    cv::Mat deNoise(cv::Mat inputImage);


    std::vector<cv::Mat> MSER_Features(cv::Mat img, double &area);

    cv::Mat HOG_Features(cv::HOGDescriptor hog, std::vector<cv::Mat> imgs);

    int trainStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm,
        std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels);

    float SVMTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHOG);

    int visualization();
};
