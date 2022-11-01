#include <sstream>
#include <string>
#include <iomanip>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "print_info/utils.hpp"
#include "apriltag_ros/AprilTagDetectionArray.h"

class PrintInfo
{
public:
    PrintInfo(std::string tag)
    {
        detectSub = n.subscribe("/tag_detections", 10, &PrintInfo::printDetectInfo, this);
        markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("/detect_info", 10);
        

        // 文本marker
        text_view_facing.header.frame_id=tag;
        text_view_facing.header.stamp = ros::Time::now();
        text_view_facing.ns = "basic_shapes";
        text_view_facing.action = visualization_msgs::Marker::ADD;
        text_view_facing.pose.orientation.w = 1.0;
        text_view_facing.id =0;
        text_view_facing.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_view_facing.scale.z = 0.02;
        text_view_facing.color.b = 1.0;
        text_view_facing.color.g = 1.0;
        text_view_facing.color.r = 1.0;
        text_view_facing.color.a = 1.0;
        marker_array.markers.push_back(text_view_facing);

        // 线段marker
        line_strip.header.frame_id=tag;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "basic_shapes";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id =1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.005;
        line_strip.color.b = 1.0;
        line_strip.color.g = 1.0;
        line_strip.color.r = 1.0;
        line_strip.color.a = 0.5;
        line_strip.lifetime = ros::Duration();
        marker_array.markers.push_back(line_strip);
    }

    void printDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr & detectInfo)
    {
        if (detectInfo->detections.size() == 0)
            return;

        double tx, ty, tz, qx, qy, qz, qw;
        tx = detectInfo->detections[0].pose.pose.pose.position.x;
        ty = detectInfo->detections[0].pose.pose.pose.position.y;
        tz = detectInfo->detections[0].pose.pose.pose.position.z;

        qx = detectInfo->detections[0].pose.pose.pose.orientation.x;
        qy = detectInfo->detections[0].pose.pose.pose.orientation.y;
        qz = detectInfo->detections[0].pose.pose.pose.orientation.z;
        qw = detectInfo->detections[0].pose.pose.pose.orientation.w;
        // std::cout << "qx, qy, qz, qw: " << qx << " " << qy << " " << qz << " " << qw << std::endl;

        // 修改：内旋（测试可行，参考马岳），绕相机自身的坐标系（cam坐标系的x，y，z轴）依次旋转一定角度
        // Eigen::Quaterniond q (qw, qx, qy, qz);
        // Eigen::Matrix3d rot = utils::Quaternion2Matrix(q);
        // Eigen::Vector3d euler = utils::Matrix2EulerInternal(rot);

        // 原始：外旋（测试可行），绕固定坐标系（tag坐标系的x，y，z轴）依次旋转一定的角度
        // Eigen::Quaterniond q (qx, qy, qz, qw);   // NOTE: 构造四元数参数顺序错了😶
        Eigen::Quaterniond q (qw, qx, qy, qz);
        Eigen::Matrix3d rot = q.toRotationMatrix();
        Eigen::Vector3d euler = utils::Matrix2EulerExternal(rot);


        geometry_msgs::Pose pose;
        pose.position.x = tx + 0.03;
        pose.position.y = ty + 0.0;
        pose.position.z = tz + 0.0;
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;

        // 可视化打印控制：
        // 1、设置换行打印，区分平移与旋转
        // 2、设置小数点后位置
        // 3、设置总位数，并用空格填补
        Eigen::Vector3f info_trans, info_euler;
        info_trans << tx * 1000, ty * 1000, tz * 1000;
        info_euler << euler[0]/M_PI * 180, euler[1]/M_PI * 180, euler[2]/M_PI * 180;
        std::ostringstream str;
        str.precision(1);
        str << std::fixed << std::setw(6) << std::setfill(' ') << info_trans[0] << std::endl 
            << std::fixed << std::setw(6) << std::setfill(' ') << info_trans[1] << std::endl 
            << std::fixed << std::setw(6) << std::setfill(' ') << info_trans[2] << std::endl 
            << std::fixed << std::setw(6) << std::setfill(' ') << info_euler[0] << std::endl 
            << std::fixed << std::setw(6) << std::setfill(' ') << info_euler[1] << std::endl 
            << std::fixed << std::setw(6) << std::setfill(' ') << info_euler[2] << std::endl;
        
        text_view_facing.text = str.str();
        text_view_facing.pose = pose;

        // 添加点到line_strip
        line_strip.points.clear();
        geometry_msgs::Point p1, p2;
        p1.x = 0.0, p1.y = 0.0, p1.z = 0.0;
        p2.x = tx, p2.y = ty, p2.z = tz;
        line_strip.points.push_back(p1);
        line_strip.points.push_back(p2);


        marker_array.markers[0] = text_view_facing;
        marker_array.markers[1] = line_strip;
        markerArrayPub.publish(marker_array);
    }

private:
    ros::NodeHandle n; 
    ros::Publisher  markerArrayPub;
    ros::Subscriber detectSub;

    visualization_msgs::Marker text_view_facing;
    visualization_msgs::Marker line_strip;
    visualization_msgs::MarkerArray marker_array;
};

 
int main(int argc, char **argv)
{
    // Initiate ROS
    ros::init(argc, argv, "GetDetectInfo");

    std::string tag;    // tag坐标系名称，是信息打印的参考坐标系

    ROS_INFO("Start Print Info Node. ");
    if (ros::param::get("Tag", tag)) 
        ROS_INFO("  Tag Name exists: %s", tag.c_str()); 
    else 
        ROS_INFO("  Please set Tag Name!");
    
    PrintInfo print_info(tag);
    
    ros::spin();
    
    return 0;
}
