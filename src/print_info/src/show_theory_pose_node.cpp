#include <vector>
#include <string>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> //静态坐标消息类型
#include "print_info/utils.hpp"

using std::cout;
using std::endl;
using std::string;

class ShowTheoryPose
{
public:
    ShowTheoryPose(double A, double R, int N, string Tag) : R(R), N(N), Tag(Tag), angle_with_z(N), positions(N), quaters(N),
                                                            text_view_facing(N), line_strip(N)
    {
        
        // 输入为角度，转化为弧度
        this->A = A / 180.0 * M_PI;
        // cout << "A: " << this->A << " R: " << this->R << " N: " << this->N << endl;

         computeInitPose();

        staticPubTF();

        markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("/theory_info", 10);

        showTheoryNumberAndLine();
        
        // boost::bind(&mapcallback, _1, &test)
    }

    void showTheoryNumberAndLine()
    {
        ros::Rate r(1); 
        while( ros::ok() )
        {
            marker_array.markers.clear();

            // 文本marker
            for (int i=0; i<N; ++i)
            {
                visualization_msgs::Marker temp;
                temp.header.frame_id=Tag;
                temp.header.stamp = ros::Time::now();
                temp.ns = "basic_shapes";
                temp.action = visualization_msgs::Marker::ADD;
                temp.pose.orientation.w = 1.0;
                temp.id =i;
                temp.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                temp.scale.z = 0.02;
                temp.color.b = 0.0;
                temp.color.g = 1.0;
                temp.color.r = 1.0;
                temp.color.a = 1.0;
                text_view_facing[i] = temp;
                marker_array.markers.push_back(text_view_facing[i]);
            }

            // 线段marker
            for (int i=0; i<N; ++i)
            {
                visualization_msgs::Marker temp;
                temp.header.frame_id = Tag;
                temp.header.stamp = ros::Time::now();
                temp.ns = "basic_shapes";
                temp.action = visualization_msgs::Marker::ADD;
                temp.pose.orientation.w = 1.0;
                temp.id = i + N;
                temp.type = visualization_msgs::Marker::LINE_STRIP;
                temp.scale.x = 0.005;
                temp.color.b = 0.0;
                temp.color.g = 1.0;
                temp.color.r = 1.0;
                temp.color.a = 0.3;
                temp.lifetime = ros::Duration();
                line_strip[i] = temp;
                marker_array.markers.push_back(line_strip[i]);
            }

            // 显示理论数值
            Eigen::Vector3f info_trans, info_euler;
            for (int i=0; i<N; ++i)
            {
                // 构造打印信息:平移向量，旋转欧拉角
                Eigen::Vector3d trans = Eigen::Vector3d(positions[i][0], positions[i][1], positions[i][2]);
                Eigen::Vector3d euler = utils::Matrix2EulerExternal(quaters[i].toRotationMatrix());
                info_trans << trans[0] * 1000, trans[1] * 1000, trans[2] * 1000;
                info_euler << euler[0]/M_PI * 180, euler[1]/M_PI * 180, euler[2]/M_PI * 180;

                // 格式化打印信息
                std::ostringstream str;
                str.precision(1);
                str << std::fixed << std::setw(6) << std::setfill(' ') << info_trans[0] << std::endl 
                    << std::fixed << std::setw(6) << std::setfill(' ') << info_trans[1] << std::endl 
                    << std::fixed << std::setw(6) << std::setfill(' ') << info_trans[2] << std::endl 
                    << std::fixed << std::setw(6) << std::setfill(' ') << info_euler[0] << std::endl 
                    << std::fixed << std::setw(6) << std::setfill(' ') << info_euler[1] << std::endl 
                    << std::fixed << std::setw(6) << std::setfill(' ') << info_euler[2] << std::endl;

                // 构造文本显示位姿
                geometry_msgs::Pose pose;
                pose.position.x = trans[0] - 0.03;
                pose.position.y = trans[1] + 0.0;
                pose.position.z = trans[2] + 0.0;
                pose.orientation.x = quaters[i].x();
                pose.orientation.y = quaters[i].y();
                pose.orientation.z = quaters[i].z();
                pose.orientation.w = quaters[i].w();
                
                text_view_facing[i].text = str.str();
                text_view_facing[i].pose = pose;
                marker_array.markers[i] = text_view_facing[i];
            }

            // 显示理论线
            geometry_msgs::Point p1, p2;
            p1.x = 0.0, p1.y = 0.0, p1.z = 0.0;
            for (int i=0; i<N; ++i)
            {
                p2.x = positions[i][0];
                p2.y = positions[i][1];
                p2.z = positions[i][2];
                line_strip[i].points.push_back(p1);
                line_strip[i].points.push_back(p2);
                marker_array.markers[i+N] = line_strip[i];
            }

            // 发布显示
            markerArrayPub.publish(marker_array);
            r.sleep();
        }

        return;
    }


    void computeInitPose()
    {
        // 计算理论位置与原点连线与z轴正方向的夹角
        double delte_angle = A / (N-1);
        double left_boundary_anlge;
        if(N % 2 == 0)
            left_boundary_anlge = (N-1)/2.0 * delte_angle;
        else
            left_boundary_anlge = (N/2) * delte_angle;

        for (int i=0; i<N; ++i)
        {
            // 计算理论位置
            // 1、与tag坐标系原点连线和z轴正方向的夹角
            angle_with_z[i] = left_boundary_anlge - i * delte_angle;

            // 2、相对tag坐标系的坐标位置
            positions[i] << -R * sin(angle_with_z[i]), -0.2 , R * cos(angle_with_z[i]);

            // 3、相对tag坐标系的欧拉角与四元数，欧拉角可以直观获取，但tf需要四元数
            Eigen::Vector3d euler (M_PI, angle_with_z[i], 0.0);
            Eigen::AngleAxisd x(euler[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd y(euler[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd z(euler[2], Eigen::Vector3d::UnitZ());
            quaters[i] = z * x * y;
        }
    }

    void staticPubTF()
    {
        for(int i = 0; i < N; i ++)
        {
            geometry_msgs::TransformStamped ts;                         //创建geometry_msgs命名空间下TransformStamped类型的变量,名为ts
            ts.header.seq = 100;                                        // consecutively increasing ID 
            ts.header.stamp = ros::Time::now();                         // 时间戳
            ts.header.frame_id = Tag;                                   // 父坐标系名称(数据类型string)
            ts.child_frame_id = "Fix_" + std::to_string(i+1);           // 子坐标系名称(数据类型string)
            ts.transform.translation.x = positions[i][0];               // 将计算出的坐标封装在ts中
            ts.transform.translation.y = positions[i][1];
            ts.transform.translation.z = positions[i][2];

            ts.transform.rotation.x = quaters[i].x();
            ts.transform.rotation.y = quaters[i].y();
            ts.transform.rotation.z = quaters[i].z();
            ts.transform.rotation.w = quaters[i].w();
        
            tf_pub.sendTransform(ts);                                   //发送静态坐标信息
        }
    }


private:
    double A;   // 圆弧角度, 输入单位：角度
    double R;   // 圆弧半径，输入单位：米
    int    N;   // 相机数量
    string Tag;
    std::vector<double>                 angle_with_z;   // 理论位置与原点连线和z轴夹角
    std::vector<Eigen::Vector3d>        positions;      // 理论位置坐标
    std::vector<Eigen::Quaterniond>     quaters;        // 理论位置位姿(四元数)
    tf2_ros::StaticTransformBroadcaster tf_pub;         // tf发布对象

    ros::NodeHandle                         n; 
    std::vector<visualization_msgs::Marker> text_view_facing;
    std::vector<visualization_msgs::Marker> line_strip;
    visualization_msgs::MarkerArray         marker_array;
    ros::Publisher                          markerArrayPub;

};

 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ShowTheoryPose");
    double A;
    double R;
    int    N;
    string Tag;

    ROS_INFO("Start Show Theory Pose Node. "); 
    if (ros::param::get("A", A)) 
        ROS_INFO("  Arc Angle exists: %f degree.", A); 
    else 
        ROS_INFO("  Please set Arc angle!");

    if (ros::param::get("R", R)) 
        ROS_INFO("  Arc Radius exists: %f meters.", R);
    else 
        ROS_INFO("  Please set Arc Radius!");

    if (ros::param::get("N", N)) 
        ROS_INFO("  Camera Numbers exists: %d.", N); 
    else 
        ROS_INFO("  Please set Camera Numbers!");

    if (ros::param::get("Tag", Tag)) 
        ROS_INFO("  Tag Name exists: %s.", Tag.c_str()); 
    else 
        ROS_INFO("  Please set Tag Name!");
    
    
    ShowTheoryPose show_theory_pose(A, R, N, Tag);
    
    ros::spin();

    return 0;
}
