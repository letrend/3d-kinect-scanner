#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rviz/panel.h>
#include <stdio.h>
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QComboBox>
#include <QTimer>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>
#include <map>
#include <visualization_msgs/Marker.h>
#include "kinect.hpp"
#include "icp_wrapper.hpp"
#include <thread>
#include <Eigen/Core>
#include <mutex>
#endif

using namespace std;
using namespace Eigen;

class KinectScannerPlugin : public rviz::Panel {
    Q_OBJECT

public:
    KinectScannerPlugin(QWidget *parent = 0);

    ~KinectScannerPlugin();

    /**
     * Load all configuration data for this panel from the given Config object.
     * @param config rviz config file
     */
    virtual void load(const rviz::Config &config);

    /**
     * Save all configuration data from this panel to the given
     * Config object.  It is important here that you call save()
     * on the parent class so the class id and panel name get saved.
     * @param config rviz config file
     */
    virtual void save(rviz::Config config) const;

    void poseEstimation();

    void publishModel();

public slots:
    void renderImages(float mean_time);

    signals:
    void imagesReady(float mean_time);

public Q_SLOTS:
    void resetPose();
private:
    ros::NodeHandlePtr nh;
    pair<uint, uint> currentID;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher marker_visualization_pub;
    boost::shared_ptr<MyFreenectDevice> device;
    cv::Mat depth0, depth1, rgb;
    boost::shared_ptr<ICPCUDA> icpcuda;
    boost::shared_ptr<std::thread> odometry_thread;
    bool getPose = true;
    Matrix4d pose;
    mutex mux;
};
