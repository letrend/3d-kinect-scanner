#pragma once

#include <libfreenect.hpp>
#include <libfreenect_registration.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp> 
#include <libfreenect.h>

class myMutex {
public:
    myMutex() {
        pthread_mutex_init( &m_mutex, NULL );
    }
    void lock() {
        pthread_mutex_lock( &m_mutex );
    }
    void unlock() {
        pthread_mutex_unlock( &m_mutex );
    }
private:
    pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
public:
    MyFreenectDevice(freenect_context *_ctx, int _index)
    : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
            m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
            m_new_depth_frame(false),
            rgbMat(cv::Size(640,480), CV_8UC3, cv::Scalar(0)),
            ownMat(cv::Size(640,480),CV_8UC3,cv::Scalar(0)) {
        for( unsigned int i = 0 ; i < 2048 ; i++) {
            float v = i/2048.0;
            v = std::pow(v, 3)* 6;
            m_gamma[i] = v*6*256;
        }
        depth_mm = new uint16_t[640*480];
        rgb_raw = new uint8_t[640*480*3];
        rgb_registered = new uint8_t[640*480*3];
    }
    
    // Do not call directly even in child
    void VideoCallback(void* _rgb, uint32_t timestamp) {
//        std::cout << "RGB callback" << std::endl;
        m_rgb_mutex.lock();
        uint8_t* rgb = static_cast<uint8_t*>(_rgb);
        rgbMat.data = rgb;
        for (int i= 0;i<640*480*3;i++){
            rgb_raw[i] = rgb[i];
        }
        m_new_rgb_frame = true;
        m_rgb_mutex.unlock();
    };
    
    // Do not call directly even in child
    void DepthCallback(void* _depth, uint32_t timestamp) {
//        std::cout << "Depth callback" << std::endl;
        m_depth_mutex.lock();
        depth =  static_cast<uint16_t*>(_depth);
        for (int i= 0;i<640*480;i++){
            depth_mm[i] = depth[i];
        }
        m_new_depth_frame = true;
        m_depth_mutex.unlock();
    }
    
    bool getVideo(cv::Mat& output) {
        m_rgb_mutex.lock();
        if(m_new_rgb_frame) {
            cv::cvtColor(rgbMat, output, CV_RGB2BGR);
            output.convertTo(output, CV_32FC3);
            m_new_rgb_frame = false;
            m_rgb_mutex.unlock();
            return true;
        } else {
            m_rgb_mutex.unlock();
            return false;
        }
    }
    
    bool getDepth(cv::Mat& output) {
        m_depth_mutex.lock();
        if(m_new_depth_frame) {
            for (int i= 0;i<640;i++){
                for (int j=0;j<480;j++){
                    //                depthPtr[i] = depth[i];
                    output.at<float>(j,i)=depth[i+640*j];
                    output.at<float>(j,i)/=1000.0f;
                    depth_mm[i+640*j] = depth[i+640*j];
                }
            }
            m_new_depth_frame = false;
            m_depth_mutex.unlock();
            return true;
        } else {
            m_depth_mutex.unlock();
            return false;
        }
    }
    
    bool getDepthMM(cv::Mat& output) {
        m_depth_mutex.lock();
        if(m_new_depth_frame) {
            for (int i= 0;i<640;i++){
                for (int j=0;j<480;j++){
                    //                depthPtr[i] = depth[i];
                    output.at<float>(j,i)=depth[i+640*j];
                }
            }
            m_new_depth_frame = false;
            m_depth_mutex.unlock();
            return true;
        } else {
            m_depth_mutex.unlock();
            return false;
        }
    }
    
    bool getRgbMapped2Depth(cv::Mat& output) {
        m_rgb_mutex.lock();
        if(m_new_rgb_frame) {
        freenect_map_rgb_to_depth((freenect_device*)this->getDevice(), depth_mm, rgb_raw, rgb_registered);
        float *aOut = (float*)output.data;
        int i=0;
        for (int y=0; y<480; y++){
            for (int x=0; x<640; x++){
                for (int c=0; c<3; c++){
                    aOut[(3-1-c) + 3*(x + (size_t)640*y)] = (float)rgb_registered[i];
                    i++;
                }
            }
        }
        m_rgb_mutex.unlock();
        return true;
        } else {
            m_rgb_mutex.unlock();
            return false;
        }
    }
    
    void cross(float3 &a, float3 &b, float3 &c){
        c.x = a.y * b.z - a.z * b.y ;
        c.y = a.z * b.x - a.x * b.z ;
        c.z = a.x * b.y - a.y * b.x ;
    }
    
    void normalize(float3 &v){
        double norm = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
        v.x /=norm; v.y /=norm; v.z /=norm;
    }
    
    double norm(float3 &v){
        return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    }
    
    void getPoseFromAccelerometer(Eigen::Matrix4f pose){
        this->updateState();
        double dx,dy,dz;
        freenectTiltState->getAccelerometers(&dx,&dy,&dz);
        
        float3 X,Y,Z;
        X.x = dy;   Y.x = -dx;  
        X.y = dx;   Y.y = dy;   
        X.z = dz;   Y.z = -dz; 
        
        normalize(X);normalize(Y);
        cross(X,Y,Z);
        normalize(Z);

        pose(0,0) = X.x;
        pose(1,0) = X.y;
        pose(2,0) = X.z;
        pose(0,1) = Y.x;
        pose(1,1) = Y.y;
        pose(2,1) = Y.z;
        pose(0,2) = Z.x;
        pose(1,2) = Z.y;
        pose(2,2) = Z.z;
        pose/=norm(X);        
    }
    
    Freenect::FreenectTiltState *freenectTiltState;
private:
    std::vector<uint8_t> m_buffer_depth;
    std::vector<uint8_t> m_buffer_rgb;
    std::vector<uint16_t> m_gamma;
    cv::Mat depthMat;
    cv::Mat rgbMat;
    cv::Mat ownMat;
    myMutex m_rgb_mutex;
    myMutex m_depth_mutex;
    bool m_new_rgb_frame;
    bool m_new_depth_frame;
    uint16_t* depth;
    uint16_t* depth_mm;
    uint8_t* rgb_raw;
    uint8_t* rgb_registered;
};

