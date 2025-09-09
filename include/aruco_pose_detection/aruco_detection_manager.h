#ifndef ARUCO_DETECTION_MANAGER_H
#define ARUCO_DETECTION_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include "aruco_box_config.h"



struct BoxPose6D {
    cv::Vec3d position;
    cv::Vec4d orientation_quat;  // [w, x, y, z]
};

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
    
    // TF 相關成員
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ArUco Parameters
    int total_image_pixels_;        // 總圖片像素
    double total_image_meters_;     // 總圖片實際尺寸 (米)
    int marker_pixels_;             // 單個標記像素

    // Case
    ArUcoBoxConfig::BoxConfigType box_config_type_;

    /**
     * @brief 紀錄當前 ArUco Case
     */
    void initArUcoCaseType();


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
    
    /**
     * @brief 獲取從相機光學座標系到cam_base_link的變換
     * @param transform 輸出的變換
     * @return 是否成功獲取變換
     */
    bool getTransformFromOpticalToBase(geometry_msgs::TransformStamped& transform);
    
    /**
     * @brief 將姿態從相機光學座標系變換到cam_base_link座標系
     * @param rvec 旋轉向量
     * @param tvec 平移向量
     * @param transformed_pose 變換後的姿態
     * @return 是否成功變換
     */
    bool transformPose(const cv::Vec3d& rvec, const cv::Vec3d& tvec, 
                      geometry_msgs::PoseStamped& transformed_pose);

    /**
     * @brief 計算 marker 的權重，用於後續 Kalman Filter
     */
    double calculateWeight(int marker_id, const cv::Vec3d& tvec, const cv::Vec3d& rvec);

    BoxPose6D fuseBoxPosesOpticalFrame(const std::vector<int>& marker_ids,
                                  const std::vector<cv::Vec3d>& tvecs,
                                  const std::vector<cv::Vec3d>& rvecs);
    cv::Vec4d fuseQuaternions(const std::vector<cv::Vec4d>& quaternions,
                         const std::vector<double>& weights);
};

#endif // ARUCO_DETECTION_MANAGER_H