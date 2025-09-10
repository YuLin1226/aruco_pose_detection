#ifndef ARUCO_BOX_CONFIG_H
#define ARUCO_BOX_CONFIG_H

#include <map>
#include <vector>
#include <opencv2/opencv.hpp>

namespace ArUcoBoxConfig 
{
    // 定義箱子配置的結構
    struct BoxPose 
    {
        double x, y, z;           // 箱子中心相對於 ArUco marker 的位置
        double roll, pitch, yaw;  // 箱子中心相對於 ArUco marker 的旋轉 (弧度)
        
        BoxPose(double x = 0.0, double y = 0.0, double z = 0.0, 
                double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
            : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
        
        // 轉換為 OpenCV 格式
        cv::Vec3d getTranslation() const {
            return cv::Vec3d(x, y, z);
        }
        
        cv::Vec3d getRotation() const {
            return cv::Vec3d(roll, pitch, yaw);
        }
        
        // 轉換為齊次變換矩陣 (4x4)
        cv::Mat getTransformMatrix() const {
            cv::Mat R;
            cv::Vec3d rvec(roll, pitch, yaw);
            cv::Rodrigues(rvec, R);
            
            cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(T(cv::Rect(0, 0, 3, 3)));
            T.at<double>(0, 3) = x;
            T.at<double>(1, 3) = y;
            T.at<double>(2, 3) = z;
            
            return T;
        }
    };
    
    // 每個 case 的配置：marker_id -> BoxPose
    typedef std::map<int, BoxPose> CaseConfig;
    
    // Case1 的配置
    const CaseConfig CASE_1 = {
        {1, BoxPose(0.0, 0.0, -0.5, 0.0, 0.0, 0.0)}
    };
    // Case2a 的配置
    const CaseConfig CASE_2A = {
        {21, BoxPose( 266.0/1084.0, 0.0, -0.5, 0.0, 0.0, 0.0)},
        {22, BoxPose(-266.0/1084.0, 0.0, -0.5, 0.0, 0.0, 0.0)}
    };
    // Case2b 的配置
    const CaseConfig CASE_2B = {
        {23, BoxPose( 266.0/1084.0, -266.0/1084.0, -0.5, 0.0, 0.0, 0.0)},
        {24, BoxPose(-266.0/1084.0,  266.0/1084.0, -0.5, 0.0, 0.0, 0.0)}
    };
    // Case3 的配置
    const CaseConfig CASE_3 = {
        {31, BoxPose( 266.0/1084.0, -266.0/1084.0, -0.5, 0.0, 0.0, 0.0)},
        {32, BoxPose(-266.0/1084.0, -266.0/1084.0, -0.5, 0.0, 0.0, 0.0)},
        {33, BoxPose( 266.0/1084.0,  266.0/1084.0, -0.5, 0.0, 0.0, 0.0)},
        {34, BoxPose(-266.0/1084.0,  266.0/1084.0, -0.5, 0.0, 0.0, 0.0)}
    };
    // Case4 的配置
    const CaseConfig CASE_4 = {
        {41, BoxPose( 532.0/1616.0, -266.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {42, BoxPose(   0.0/1616.0, -266.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {43, BoxPose(-532.0/1616.0, -266.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {44, BoxPose( 532.0/1616.0,  266.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {45, BoxPose(   0.0/1616.0,  266.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {46, BoxPose(-532.0/1616.0,  266.0/1616.0, -0.5, 0.0, 0.0, 0.0)}
    };
    // Case5 的配置
    const CaseConfig CASE_5 = {
        {51, BoxPose( 532.0/1616.0, -532.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {52, BoxPose(   0.0/1616.0, -532.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {53, BoxPose(-532.0/1616.0, -532.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {54, BoxPose( 532.0/1616.0,           0.0, -0.5, 0.0, 0.0, 0.0)},
        {55, BoxPose(   0.0/1616.0,           0.0, -0.5, 0.0, 0.0, 0.0)},
        {56, BoxPose(-532.0/1616.0,           0.0, -0.5, 0.0, 0.0, 0.0)},
        {57, BoxPose( 532.0/1616.0,  532.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {58, BoxPose(   0.0/1616.0,  532.0/1616.0, -0.5, 0.0, 0.0, 0.0)},
        {59, BoxPose(-532.0/1616.0,  532.0/1616.0, -0.5, 0.0, 0.0, 0.0)}
    };
    // 所有配置的總覽
    enum class BoxConfigType {
        CASE_1,
        CASE_2A,
        CASE_2B,
        CASE_3,
        CASE_4,
        CASE_5
    };
    
    // 獲取指定配置的函數
    inline const CaseConfig& getConfig(BoxConfigType type) {
        switch(type) {
            case BoxConfigType::CASE_1:
                return CASE_1;
            case BoxConfigType::CASE_2A:
                return CASE_2A;
            case BoxConfigType::CASE_2B:
                return CASE_2B;
            case BoxConfigType::CASE_3:
                return CASE_3;
            case BoxConfigType::CASE_4:
                return CASE_4;
            case BoxConfigType::CASE_5:
                return CASE_5;
            default:
                return CASE_1;
        }
    }
    
    // 檢查某個 marker ID 是否在指定配置中
    inline bool isMarkerInConfig(int marker_id, BoxConfigType type) {
        const CaseConfig& config = getConfig(type);
        return config.find(marker_id) != config.end();
    }
    
    // 獲取箱子中心位置（從 marker 座標系轉換）
    inline cv::Vec3d getBoxCenterFromMarker(const cv::Vec3d& marker_tvec, 
                                           const cv::Vec3d& marker_rvec,
                                           int marker_id, 
                                           BoxConfigType type) {
        const CaseConfig& config = getConfig(type);
        auto it = config.find(marker_id);
        
        if (it == config.end()) {
            // 如果找不到該 marker，返回原始位置
            return marker_tvec;
        }
        
        const BoxPose& box_pose = it->second;
        
        // 將 marker 的旋轉轉換為旋轉矩陣
        cv::Mat R_marker;
        cv::Rodrigues(marker_rvec, R_marker);
        
        // 箱子中心相對於 marker 的位置
        cv::Mat box_offset = (cv::Mat_<double>(3, 1) << 
            box_pose.x, box_pose.y, box_pose.z);
        
        // 應用 marker 的旋轉到偏移量
        cv::Mat box_center_camera = R_marker * box_offset + 
            (cv::Mat_<double>(3, 1) << marker_tvec[0], marker_tvec[1], marker_tvec[2]);
        
        return cv::Vec3d(box_center_camera.at<double>(0, 0),
                        box_center_camera.at<double>(1, 0),
                        box_center_camera.at<double>(2, 0));
    }
    
    // 獲取所有檢測到的 markers 對應的箱子中心估計
    inline std::vector<cv::Vec3d> getAllBoxCenterEstimates(
        const std::vector<int>& marker_ids,
        const std::vector<cv::Vec3d>& tvecs,
        const std::vector<cv::Vec3d>& rvecs,
        BoxConfigType type) {
        
        std::vector<cv::Vec3d> box_centers;
        
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            if (isMarkerInConfig(marker_ids[i], type)) {
                cv::Vec3d box_center = getBoxCenterFromMarker(
                    tvecs[i], rvecs[i], marker_ids[i], type);
                box_centers.push_back(box_center);
            }
        }
        
        return box_centers;
    }
}

#endif // ARUCO_BOX_CONFIG_H