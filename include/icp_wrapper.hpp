#pragma once

#include "../third_party/ICPCUDA/src/ICPOdometry.h"

class ICPCUDA{
public:
    ICPCUDA(size_t pWidth, size_t pHeight, Eigen::Matrix4f pose_init){
        icpOdom = new ICPOdometry(640, 480, 320, 240, 528, 528);
        posef = pose_init;
        depth0 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        depth1 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        width = pWidth;
        height = pHeight;
    };
    
    ICPCUDA(size_t pWidth, size_t pHeight){
        icpOdom = new ICPOdometry(640, 480, 320, 240, 528, 528);
        posef = Eigen::Matrix<float, 4, 4>::Identity();
        depth0 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        depth1 = cv::Mat::zeros(pHeight, pWidth, CV_16U);  
        width = pWidth;
        height = pHeight;
    };
    
    void setInitialPose(Eigen::Matrix4f pose_init){
        posef = pose_init;
    }
    
    void getPoseFromDepth(cv::Mat &depth0f, cv::Mat &depth1f){
         // depth maps need to be in milimeters
        
        depth0f.convertTo(depth0,CV_16U);
        depth1f.convertTo(depth1,CV_16U);
        
        // ICP
        icpOdom->initICPModel((unsigned short *)depth0.data, 20.0f, posef);
        
        icpOdom->initICP((unsigned short *)depth1.data, 20.0f);
        
        Eigen::Vector3f trans = posef.topRightCorner(3, 1);
        Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = posef.topLeftCorner(3, 3);
        
        icpOdom->getIncrementalTransformation(trans,rot,128,96);
        
        translations.push_back(trans);
        
        posef.topLeftCorner(3, 3) = rot;
        posef.topRightCorner(3, 1) = trans;
    };
public:
    Eigen::Matrix4f pose(){
        return posef;
    }
    Eigen::Matrix4f pose_inv(){
        return posef.inverse();
    };
    
private:
    ICPOdometry *icpOdom;
    Eigen::Matrix4f posef;
    cv::Mat depth0;
    cv::Mat depth1;
    std::vector<Eigen::Vector3f> translations;
    size_t width,height;
};

