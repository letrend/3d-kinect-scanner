#pragma once
// std
#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
// freenect
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
// opencv
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class MyFileLogger: public libfreenect2::Logger
{
private:
    std::ofstream logfile_;
public:
    MyFileLogger(const char *filename)
    {
        if (filename)
            logfile_.open(filename);
        level_ = Debug;
    }
    bool good()
    {
        return logfile_.is_open() && logfile_.good();
    }
    virtual void log(Level level, const std::string &message)
    {
        logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
    }
};

class MyFreenectDevice{
public:
    MyFreenectDevice(){
        filelogger = new MyFileLogger("log.txt");
        if (filelogger->good())
            libfreenect2::setGlobalLogger(filelogger);
        else
            cout << "ERROR: could not initialize logger" << endl;

        rgb = frames[libfreenect2::Frame::Color];
        ir = frames[libfreenect2::Frame::Ir];
        depth = frames[libfreenect2::Frame::Depth];

        pipeline = new libfreenect2::CpuPacketPipeline();

        if(freenect2.enumerateDevices() == 0)
        {
            cout << "ERROR: no device connected!" << endl;
            return;
        }

        string serial = freenect2.getDefaultDeviceSerialNumber();
        dev = freenect2.openDevice(serial, pipeline);

        if(!dev->start())
            cout << "ERROR: could not start kinect device" << endl;

        registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

        undistorted = new libfreenect2::Frame(512, 424, 4);
        registered = new libfreenect2::Frame(512, 424, 4);

        int types = 0;
        types |= libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
        listener = new libfreenect2::SyncMultiFrameListener(types);

        dev->setColorFrameListener(listener);
        dev->setIrAndDepthFrameListener(listener);
    }

    ~MyFreenectDevice(){
        dev->stop();
        dev->close();
        delete registration;
        delete undistorted;
        delete registered;
        delete listener;
        delete filelogger;
    }

    void getVideo(cv::Mat& output) {
//        cv::Mat img = cv::Mat(rgb->height,rgb->width,CV_8UC4,rgb->data), img2;
        //        cv::flip(img, img, 1);
        output = cv::Mat(rgb->height,rgb->width,CV_8UC3);
        uchar *aOut = (uchar*)output.data;
        int i=0;
        for (int y=0; y<rgb->height; y++){
            for (int x=0; x<rgb->width; x++){
                for (int c=0; c<3; c++){
                    aOut[c + 3*(x + (size_t)rgb->width*y)] = (uchar)rgb->data[c + 4*(x + (size_t)rgb->width*y)];
                }
            }
        }
    }

    void getDepth(cv::Mat& output) {
        output = cv::Mat(undistorted->height,undistorted->width,CV_32FC1,undistorted->data);
        output/=1000.0f; // convert to meter
//        cv::flip(output, output, 1);
    }

    void getDepthMM(cv::Mat& output) {
        output = cv::Mat(undistorted->height,undistorted->width,CV_32FC1,undistorted->data);
//        cv::flip(output, output, 1);
    }

    void getRgbMapped2Depth(cv::Mat& output) {
//        cv::Mat img = cv::Mat(registered->height,registered->width,CV_8UC4,registered->data), img2;
        float *aOut = (float*)output.data;
        int i=0;
        for (int y=0; y<registered->height; y++){
            for (int x=0; x<registered->width; x++){
                for (int c=0; c<3; c++){
                    aOut[c + 3*(x + (size_t)registered->width*y)] = (float)registered->data[c + 4*(x + (size_t)registered->width*y)]/255.0f;
                }
            }
        }
    }

    bool updateFrames(){
        listener->release(frames);

        if (!listener->waitForNewFrame(frames, 10*1000)){ // wait 10 seconds
            cout << "WARNING: kinect timeout!" << endl;
            return false;
        }else{
            rgb = frames[libfreenect2::Frame::Color];
            ir = frames[libfreenect2::Frame::Ir];
            depth = frames[libfreenect2::Frame::Depth];
            registration->apply(rgb, depth, undistorted, registered);
            return true;
        }
    }

private:
    MyFileLogger *filelogger;
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::PacketPipeline *pipeline;
    libfreenect2::Registration* registration;
    libfreenect2::Frame *undistorted, *registered;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::FrameMap frames;
    libfreenect2::Frame *rgb;
    libfreenect2::Frame *ir;
    libfreenect2::Frame *depth;
};