#ifndef ARUCO_DETECTION_MANAGER_H
#define ARUCO_DETECTION_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>



class ArUcoDetectionManager
{
public:
    /**
     * @brief 建構函數
     * @param nh ROS NodeHandle 參考
     */
    explicit ArUcoDetectionManager(ros::NodeHandle& nh);
    
    /**
     * @brief 解構函數
     */
    ~ArUcoDetectionManager();

private:
    // ROS 相關成員
    ros::NodeHandle& nh_;
    ros::Subscriber image_subscriber_;
    ros::Subscriber camera_info_subscriber_;
    
    // 影像處理相關成員
    cv::Mat current_image_;
    sensor_msgs::CameraInfo camera_info_;
    bool camera_info_received_;
    bool image_captured_;
    
    // ArUco 檢測參數
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    


    /**
     * @brief 影像回調函數 - 只接收一次影像
     * @param msg 影像訊息
     */
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    
    /**
     * @brief 相機資訊回調函數
     * @param msg 相機資訊訊息
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    
    /**
     * @brief 初始化 ArUco 檢測器
     */
    void initializeArUcoDetector();
    
    /**
     * @brief 處理接收到的影像（後續實作）
     */
    void processImage();
};

#endif // ARUCO_DETECTION_MANAGER_H