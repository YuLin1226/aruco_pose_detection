#include <ros/ros.h>
#include "aruco_detection_manager.h"

int main(int argc, char** argv)
{
    // 初始化 ROS 節點
    ros::init(argc, argv, "aruco_detection_node");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting ArUco Detection Node...");
    
    try 
    {
        // 創建 ArUco 檢測管理器
        ArUcoDetectionManager detector(nh);
        
        // 保持節點運行
        ros::spin();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Error in ArUco Detection Node: %s", e.what());
        return -1;
    }
    
    ROS_INFO("ArUco Detection Node shutting down...");
    return 0;
}