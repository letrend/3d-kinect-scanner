#include <mutex>
#include "kinectScannerPlugin.hpp"

KinectScannerPlugin::KinectScannerPlugin(QWidget *parent)
        : rviz::Panel(parent){
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    QPushButton *resetpose = new QPushButton(tr("reset pose"));
    resetpose->setObjectName("resetPose");
    connect(resetpose, SIGNAL(clicked()), this, SLOT(resetPose()));
    frameLayout->addWidget(resetpose);

    QLabel *speed = new QLabel(tr("speed: "));
    speed->setObjectName("speed");
    frameLayout->addWidget(speed);

    QLabel *rgbimage = new QLabel(tr("rgbimage"));
    rgbimage->setObjectName("rgbimage");
    frameLayout->addWidget(rgbimage);

    QLabel *depthimage = new QLabel(tr("depthimage"));
    depthimage->setObjectName("depthimage");
    frameLayout->addWidget(depthimage);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "kinect scanner plugin",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));

    marker_visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);

    device = boost::shared_ptr<MyFreenectDevice>(new MyFreenectDevice);
    device->updateFrames();
    device->getDepthMM(depth0);
    icpcuda = boost::shared_ptr<ICPCUDA>(new ICPCUDA(depth0.cols,depth0.rows, device->irCameraParams.cx,
                                                     device->irCameraParams.cy, device->irCameraParams.fx,
                                                     device->irCameraParams.fy));

    odometry_thread = boost::shared_ptr<std::thread>(new std::thread(&KinectScannerPlugin::poseEstimation, this));

    QObject::connect(this, SIGNAL(imagesReady(float)), this, SLOT(renderImages(float)));
}

KinectScannerPlugin::~KinectScannerPlugin(){
    getPose = false;
    odometry_thread->join();
}

void KinectScannerPlugin::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void KinectScannerPlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
}

void KinectScannerPlugin::resetPose(){
    QPushButton* w = this->findChild<QPushButton*>("initwalkcontroller");
    w->setText("fuck me");
}

void KinectScannerPlugin::poseEstimation(){
    ROS_INFO("start pose estimation");
    while(getPose){
        device->updateFrames();
        device->getDepthMM(depth1);
        device->getRgbMapped2Depth(rgb);
        icpcuda->getPoseFromDepth(depth0,depth1);
        emit imagesReady(icpcuda->mean_time);
        pose = icpcuda->getPose();
//        ROS_INFO_STREAM_THROTTLE(1.0,"speed: " << icpcuda->mean_time << "\npose\n" << pose);
        std::swap(depth0, depth1);
    }
    ROS_INFO("stop pose estimation");
}

void KinectScannerPlugin::publishModel(){
    visualization_msgs::Marker mesh;
    mesh.header.frame_id = "world";
    mesh.ns = "kinect_v2";
    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh.color.r = 1.0f;
    mesh.color.g = 1.0f;
    mesh.color.b = 1.0f;
    mesh.color.a = 0.5;
    mesh.scale.x = 1.0;
    mesh.scale.y = 1.0;
    mesh.scale.z = 1.0;
    mesh.lifetime = ros::Duration();
    mesh.header.stamp = ros::Time::now();
    mesh.id = 0;
    Vector3d trans = pose.topRightCorner(3,1);
    Matrix3d rot = pose.topLeftCorner(3,3);
    Quaterniond q(rot);
    mesh.pose.position.x = trans(0);
    mesh.pose.position.y = trans(1);
    mesh.pose.position.z = trans(2);
    mesh.pose.orientation.x = q.x();
    mesh.pose.orientation.y = q.y();
    mesh.pose.orientation.z = q.z();
    mesh.pose.orientation.w = q.w();
    mesh.mesh_resource = "package://kinect_pose/models/kinect_v2.STL";
    marker_visualization_pub.publish(mesh);
}

void KinectScannerPlugin::renderImages(float mean_time){
    QLabel *speed = this->findChild<QLabel *>("speed");
    speed->setText("speed: " + QString::number(mean_time) + " ms");
    speed->repaint();
    {
        QLabel *rgbmage = this->findChild<QLabel *>("rgbimage");
        int w = rgb.cols;
        int h = rgb.rows;
        QImage qim_rgb(w, h, QImage::Format_RGB32);
        float *a = (float *) rgb.data;
        QRgb pixel;
        for (int i = 0; i < w; i++) {
            for (int j = 0; j < h; j++) {
                int blue = (int) (a[0 + 3 * (i + w * j)]*255.0f);
                int green = (int) (a[1 + 3 * (i + w * j)]*255.0f);
                int red = (int) (a[2 + 3 * (i + w * j)]*255.0f);
                pixel = qRgb(red, green, blue);
                qim_rgb.setPixel(i, j, pixel);
            }
        }
        QPixmap pixmap = QPixmap::fromImage(qim_rgb);
        rgbmage->setPixmap(pixmap);
        rgbmage->repaint();
    }

    {
        QLabel *depthimage = this->findChild<QLabel *>("depthimage");
        depth0.convertTo(depth0, CV_32F);
        int w = depth0.cols;
        int h = depth0.rows;
        QImage qim_depth(w, h, QImage::Format_RGB32);
        float *a = (float *) depth0.data;
        QRgb pixel;
        for (int i = 0; i < w; i++) {
            for (int j = 0; j < h; j++) {
                // to meter divided by maximum range times max pixel value
                int gray = (int) (a[i + w * j]/1000.0f/4.5f*255.0f);
                pixel = qRgb(gray, gray, gray);
                qim_depth.setPixel(i, j, pixel);
            }
        }
        QPixmap pixmap = QPixmap::fromImage(qim_depth);
        depthimage->setPixmap(pixmap);
        depthimage->repaint();
    }
    ROS_INFO_STREAM_THROTTLE(1.0,pose);
    publishModel();
}

PLUGINLIB_EXPORT_CLASS(KinectScannerPlugin, rviz::Panel)