#include "aruco_detection_manager.h"
#include <string>

ArUcoDetectionManager::ArUcoDetectionManager(ros::NodeHandle& nh)
    : nh_(nh), camera_info_received_(false), image_captured_(false), tf_listener_(tf_buffer_)
{
    ROS_INFO("Initializing ArUco Detection Manager...");
    
    // 初始化 ArUco 檢測器
    initializeArUcoDetector();
    
    // 獲取目前測試案例
    initArUcoCaseType();

    // 訂閱相機影像 topic (只接收一次)
    image_subscriber_ = nh_.subscribe("/camera/image_raw", 1, 
        &ArUcoDetectionManager::imageCallback, this);
    
    // 訂閱相機資訊 topic
    camera_info_subscriber_ = nh_.subscribe("/camera/camera_info", 1, 
        &ArUcoDetectionManager::cameraInfoCallback, this);
    
    ROS_INFO("Waiting for camera image and info...");
    ROS_INFO("Image topic: /camera/image_raw");
    ROS_INFO("Camera info topic: /camera/camera_info");
}

ArUcoDetectionManager::~ArUcoDetectionManager()
{
    ROS_INFO("ArUco Detection Manager shutting down...");
}

void ArUcoDetectionManager::initArUcoCaseType()
{
    std::string test_case;
    nh_.getParam("/aruco_detection_case", test_case);
    if (test_case.empty()) {
        test_case = "unknown";  // 預設值
    }
    ROS_INFO("Current TEST CASE: %s", test_case.c_str());
    if (test_case == "case1") {
        box_config_type_ = ArUcoBoxConfig::BoxConfigType::CASE_1;
    } else if (test_case == "case2a") {
        box_config_type_ = ArUcoBoxConfig::BoxConfigType::CASE_2A;
    } else if (test_case == "case2b") {
        box_config_type_ = ArUcoBoxConfig::BoxConfigType::CASE_2B;
    } else if (test_case == "case3") {
        box_config_type_ = ArUcoBoxConfig::BoxConfigType::CASE_3;
    } else if (test_case == "case4") {
        box_config_type_ = ArUcoBoxConfig::BoxConfigType::CASE_4;
    } else if (test_case == "case5") {
        box_config_type_ = ArUcoBoxConfig::BoxConfigType::CASE_5;
    } else {
        ROS_WARN("Unknown test case: %s, using default case1", test_case.c_str());
        box_config_type_ = ArUcoBoxConfig::BoxConfigType::CASE_1;
    }

    nh_.getParam("/total_image_pixels", total_image_pixels_);
    nh_.getParam("/total_image_meters", total_image_meters_);
    nh_.getParam("/marker_pixels", marker_pixels_);

    ROS_INFO("TOTAL IMAGE PIXEL: %d", total_image_pixels_);
    ROS_INFO("TOTAL IMAGE METER: %f", total_image_meters_);
    ROS_INFO("MARKER PIXEL: %d", marker_pixels_);

}

void ArUcoDetectionManager::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // 如果已經接收過影像，則忽略後續的影像
    if (image_captured_)
    {
        return;
    }
    
    try
    {
        ROS_INFO("Received camera image: %dx%d, encoding: %s", 
                 msg->width, msg->height, msg->encoding.c_str());
        
        // 將 ROS 影像訊息轉換為 OpenCV 格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image_ = cv_ptr->image.clone();
        
        // 標記已接收影像
        image_captured_ = true;
        
        // 取消訂閱，確保只接收一次影像
        image_subscriber_.shutdown();
        ROS_INFO("Image captured successfully! Unsubscribed from image topic.");
        
        // 如果相機資訊也已接收，則開始處理
        if (camera_info_received_)
        {
            processImage();
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error processing image: %s", e.what());
    }
}

void ArUcoDetectionManager::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (!camera_info_received_)
    {
        camera_info_ = *msg;
        camera_info_received_ = true;
        
        ROS_INFO("Camera info received:");
        ROS_INFO("  - Image size: %dx%d", camera_info_.width, camera_info_.height);
        ROS_INFO("  - Camera matrix: [%.2f, %.2f, %.2f, %.2f]", 
                 camera_info_.K[0], camera_info_.K[2], camera_info_.K[4], camera_info_.K[5]);
        
        // 取消訂閱相機資訊
        camera_info_subscriber_.shutdown();
        
        // 如果影像也已接收，則開始處理
        if (image_captured_)
        {
            processImage();
        }
    }
}


void ArUcoDetectionManager::initializeArUcoDetector()
{
    // 初始化 ArUco 字典 (使用 6x6_250 字典)
    aruco_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
    // 初始化檢測參數
    detector_params_ = cv::aruco::DetectorParameters::create();
    
    // 可以在這裡調整檢測參數
    detector_params_->adaptiveThreshWinSizeMin = 3;
    detector_params_->adaptiveThreshWinSizeMax = 23;
    detector_params_->adaptiveThreshWinSizeStep = 10;
    
    ROS_INFO("ArUco detector initialized with DICT_6X6_250 dictionary");
}

void ArUcoDetectionManager::processImage()
{
    if (!image_captured_ || !camera_info_received_)
    {
        ROS_WARN("Cannot process image: image_captured=%d, camera_info_received=%d", 
                 image_captured_, camera_info_received_);
        return;
    }
    
    ROS_INFO("Starting ArUco detection...");
    ROS_INFO("Image size: %dx%d", current_image_.cols, current_image_.rows);
    
    try
    {
        // 1. 檢測 ArUco 標記
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        
        cv::aruco::detectMarkers(current_image_, aruco_dictionary_, 
                               marker_corners, marker_ids, detector_params_);
        
        ROS_INFO("Detected %zu ArUco markers", marker_ids.size());
        
        if (!marker_ids.empty())
        {
            // 2. 準備相機內參矩陣
            cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 
                camera_info_.K[0], 0, camera_info_.K[2],
                0, camera_info_.K[4], camera_info_.K[5], 
                0, 0, 1);
            
            // 3. 畸變係數（Gazebo 仿真中通常為零）
            cv::Mat distortion_coeffs = cv::Mat::zeros(1, 5, CV_64F);
            if (camera_info_.D.size() >= 5)
            {
                for (int i = 0; i < 5; ++i)
                {
                    distortion_coeffs.at<double>(0, i) = camera_info_.D[i];
                }
            }
            
            // 4. ArUco 標記實際尺寸計算
            double marker_size = ((double)marker_pixels_ / (double)total_image_pixels_) * total_image_meters_;
            // = (512 / 1084) * 1.0 = 0.4723 米
            
            ROS_INFO("Calculated marker size: %.4f meters (%.2f cm)", 
                     marker_size, marker_size * 100.0);
            
            // 5. 估計每個標記的姿態
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size,
                                               camera_matrix, distortion_coeffs,
                                               rvecs, tvecs);
            
            // 6. 輸出檢測結果並進行座標變換
            for (size_t i = 0; i < marker_ids.size(); ++i)
            {
                ROS_INFO("Marker ID: %d", marker_ids[i]);
                ROS_INFO("  Camera optical frame - Position (x,y,z): (%.3f, %.3f, %.3f)", 
                         tvecs[i][0], tvecs[i][1], tvecs[i][2]);
                ROS_INFO("  Camera optical frame - Rotation (rx,ry,rz): (%.3f, %.3f, %.3f)", 
                         rvecs[i][0], rvecs[i][1], rvecs[i][2]);
                
                // 計算距離
                double distance = cv::norm(tvecs[i]);
                ROS_INFO("  Distance: %.3f meters", distance);
                
                // 將姿態變換到 cam_base_link 座標系
                geometry_msgs::PoseStamped transformed_pose;
                if (transformPose(rvecs[i], tvecs[i], transformed_pose))
                {
                    ROS_INFO("  Cam base link frame - Position (x,y,z): (%.3f, %.3f, %.3f)",
                             transformed_pose.pose.position.x,
                             transformed_pose.pose.position.y,
                             transformed_pose.pose.position.z);
                    ROS_INFO("  Cam base link frame - Orientation (x,y,z,w): (%.3f, %.3f, %.3f, %.3f)",
                             transformed_pose.pose.orientation.x,
                             transformed_pose.pose.orientation.y,
                             transformed_pose.pose.orientation.z,
                             transformed_pose.pose.orientation.w);
                }
                else
                {
                    ROS_WARN("  Failed to transform pose to cam_base_link frame");
                }
                    
                // 權重計算
                double weight = calculateWeight(marker_ids[i], tvecs[i], rvecs[i]);
                ROS_INFO("  Weight: %.4f", weight);
            }
            
            // 7. 可選：繪製檢測結果
            cv::Mat output_image = current_image_.clone();
            cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
            
            // 繪製座標軸
            for (size_t i = 0; i < marker_ids.size(); ++i)
            {
                cv::aruco::drawAxis(output_image, camera_matrix, distortion_coeffs,
                                  rvecs[i], tvecs[i], marker_size * 0.5);
            }

            // 儲存檢測結果圖片
            std::string filename = "aruco_detection_" + std::to_string(ros::Time::now().toSec()) + ".jpg";
            cv::imwrite(filename, output_image);
            ROS_INFO("Saved detection result to: %s", filename.c_str());
            

            // 8. 估計個別箱子中心點
            std::vector<cv::Vec3d> box_estimates = ArUcoBoxConfig::getAllBoxCenterEstimates(
                marker_ids, tvecs, rvecs, box_config_type_);

            if(box_estimates.size() != marker_ids.size()) {
                ROS_WARN("Size match error, box_estimates.size: %zu, marker_ids.size: %zu",
                box_estimates.size(), marker_ids.size());
            } else {
                ROS_INFO("Individual box center estimates (optical frame):");
                for(int i=0; i < box_estimates.size(); ++i){
                    const auto& box = box_estimates[i];
                    ROS_INFO("  Marker ID %d: (%.4f, %.4f, %.4f)",
                        marker_ids[i], box[0], box[1], box[2]);
                }
            }

            // 9. 新增：箱子姿態融合
            if (marker_ids.size() > 0) {
                BoxPose6D fused_pose = fuseBoxPosesOpticalFrame(marker_ids, tvecs, rvecs);
                
                ROS_INFO("=== FUSED BOX POSE (optical frame) ===");
                ROS_INFO("Position: (%.4f, %.4f, %.4f)", 
                         fused_pose.position[0], fused_pose.position[1], fused_pose.position[2]);
                ROS_INFO("Quaternion (w,x,y,z): (%.4f, %.4f, %.4f, %.4f)", 
                         fused_pose.orientation_quat[0], fused_pose.orientation_quat[1], 
                         fused_pose.orientation_quat[2], fused_pose.orientation_quat[3]);
                
                // 轉換為歐拉角便於理解
                tf2::Quaternion tf_quat(fused_pose.orientation_quat[1], fused_pose.orientation_quat[2], 
                                       fused_pose.orientation_quat[3], fused_pose.orientation_quat[0]);
                tf2::Matrix3x3 m(tf_quat);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                ROS_INFO("Euler angles (r,p,y): (%.4f, %.4f, %.4f) rad", roll, pitch, yaw);
                ROS_INFO("Euler angles (r,p,y): (%.2f, %.2f, %.2f) deg", 
                         roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI);
            }
            
            ROS_INFO("ArUco detection completed successfully!");
        }
        else
        {
            ROS_WARN("No ArUco markers detected in the image");
        }
    }
    catch (const cv::Exception& e)
    {
        ROS_ERROR("OpenCV error during ArUco detection: %s", e.what());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error during ArUco processing: %s", e.what());
    }
}

bool ArUcoDetectionManager::getTransformFromOpticalToBase(geometry_msgs::TransformStamped& transform)
{
    try
    {
        transform = tf_buffer_.lookupTransform("cam_base_link", "camera_optical_link", 
                                             ros::Time(0), ros::Duration(3.0));
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Could not get transform from camera_optical_link to cam_base_link: %s", ex.what());
        return false;
    }
}

bool ArUcoDetectionManager::transformPose(const cv::Vec3d& rvec, const cv::Vec3d& tvec, 
                                         geometry_msgs::PoseStamped& transformed_pose)
{
    geometry_msgs::TransformStamped transform;
    if (!getTransformFromOpticalToBase(transform))
    {
        return false;
    }
    
    try
    {
        // 將 OpenCV 旋轉向量轉換為旋轉矩陣
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        
        // 轉換為四元數
        tf2::Matrix3x3 tf_rotation(
            rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
            rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
            rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2)
        );
        
        tf2::Quaternion tf_quat;
        tf_rotation.getRotation(tf_quat);
        
        // 創建在相機光學座標系中的姿態
        geometry_msgs::PoseStamped optical_pose;
        optical_pose.header.frame_id = "camera_optical_link";
        optical_pose.header.stamp = ros::Time::now();
        
        optical_pose.pose.position.x = tvec[0];
        optical_pose.pose.position.y = tvec[1];
        optical_pose.pose.position.z = tvec[2];
        
        optical_pose.pose.orientation.x = tf_quat.x();
        optical_pose.pose.orientation.y = tf_quat.y();
        optical_pose.pose.orientation.z = tf_quat.z();
        optical_pose.pose.orientation.w = tf_quat.w();
        
        // 使用 TF2 進行座標變換
        tf2::doTransform(optical_pose, transformed_pose, transform);
        
        return true;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error transforming pose: %s", e.what());
        return false;
    }
}

double ArUcoDetectionManager::calculateWeight(int marker_id, const cv::Vec3d& tvec, const cv::Vec3d& rvec)
{
    // 1. 距離因子 (距離越近權重越高)
    double distance = cv::norm(tvec);
    double alpha_distance = 0.1;  // 可調參數
    double w_distance = 1.0 / (1.0 + alpha_distance * distance * distance);
    
    // 2. 視角因子 (正對 marker 比側面觀看更準確)
    // marker 的 Z 軸 (法向量) 在相機座標系中的方向
    cv::Mat R_marker;
    cv::Rodrigues(rvec, R_marker);
    
    // marker 的法向量 (Z軸方向，指向相機) 在相機座標系中的方向
    cv::Mat marker_normal = R_marker * (cv::Mat_<double>(3,1) << 0, 0, 1);
    
    // 相機到 marker 的視線方向 (正規化)
    cv::Vec3d view_direction = tvec / cv::norm(tvec);
    cv::Mat view_dir_mat = (cv::Mat_<double>(3,1) << view_direction[0], view_direction[1], view_direction[2]);
    
    // 計算視角 (法向量與視線方向的夾角)
    double dot_product = marker_normal.dot(-view_dir_mat);  // 負號因為是相反方向
    dot_product = std::max(-1.0, std::min(1.0, dot_product));  // 限制在 [-1,1] 避免數值錯誤
    double viewing_angle = acos(dot_product);
    
    double w_angle = cos(viewing_angle) * cos(viewing_angle);
    
    // 3. 綜合權重
    double total_weight = w_distance * w_angle;
    
    // Debug 輸出
    ROS_DEBUG("Marker ID %d: distance=%.3fm, viewing_angle=%.2f°, w_distance=%.3f, w_angle=%.3f, total_weight=%.4f",
              marker_id, distance, viewing_angle * 180.0 / M_PI, w_distance, w_angle, total_weight);
    
    return total_weight;
}

// 四元數加權平均
cv::Vec4d ArUcoDetectionManager::fuseQuaternions(const std::vector<cv::Vec4d>& quaternions,
                                                const std::vector<double>& weights)
{
    cv::Vec4d weighted_sum(0, 0, 0, 0);
    double total_weight = 0.0;
    
    for (size_t i = 0; i < quaternions.size(); ++i) {
        weighted_sum += quaternions[i] * weights[i];
        total_weight += weights[i];
    }
    
    cv::Vec4d result = weighted_sum / total_weight;
    
    // 正規化
    double norm = cv::norm(result);
    return result / norm;
}

// 箱子姿態融合函數
BoxPose6D ArUcoDetectionManager::fuseBoxPosesOpticalFrame(const std::vector<int>& marker_ids,
                                                         const std::vector<cv::Vec3d>& tvecs,
                                                         const std::vector<cv::Vec3d>& rvecs)
{
    BoxPose6D result;
    
    // 1. 獲取箱子中心位置估計 (optical frame)
    std::vector<cv::Vec3d> box_estimates = ArUcoBoxConfig::getAllBoxCenterEstimates(
        marker_ids, tvecs, rvecs, box_config_type_);
    
    // 2. 計算權重
    std::vector<double> weights;
    for (size_t i = 0; i < marker_ids.size(); ++i) {
        double weight = calculateWeight(marker_ids[i], tvecs[i], rvecs[i]);
        weights.push_back(weight);
    }
    
    // 3. 位置加權平均
    cv::Vec3d weighted_pos_sum(0, 0, 0);
    double total_weight = 0.0;
    
    for (size_t i = 0; i < box_estimates.size(); ++i) {
        weighted_pos_sum += box_estimates[i] * weights[i];
        total_weight += weights[i];
    }
    result.position = weighted_pos_sum / total_weight;
    
    // 4. 旋轉加權平均 (假設箱子的旋轉 = marker 的旋轉)
    std::vector<cv::Vec4d> quaternions;
    for (size_t i = 0; i < rvecs.size(); ++i) {
        cv::Mat R;
        cv::Rodrigues(rvecs[i], R);
        
        // 轉換為四元數 [w, x, y, z]
        tf2::Matrix3x3 tf_rotation(
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
        );
        
        tf2::Quaternion tf_quat;
        tf_rotation.getRotation(tf_quat);
        
        quaternions.push_back(cv::Vec4d(tf_quat.w(), tf_quat.x(), tf_quat.y(), tf_quat.z()));
    }
    
    result.orientation_quat = fuseQuaternions(quaternions, weights);
    
    // 輸出權重資訊
    ROS_INFO("Box pose fusion weights:");
    for (size_t i = 0; i < marker_ids.size(); ++i) {
        ROS_INFO("  Marker ID %d: weight = %.4f", marker_ids[i], weights[i]);
    }
    
    return result;
}