#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include <vector>
#include <iostream>
using std::cout;
using std::endl;

class PollingDisplay
{
public:
    PollingDisplay():it(n), index(-1), poll_id(0)
    {
        index_vec = std::vector<bool>(topic_number + 1, false);
        topic_vec = std::vector<std::string>(topic_number);

        for (int i=0; i<topic_number; ++i)
            topic_vec[i] = "/cam_" + std::to_string(i+1) + "/color/tag_detections_image";

        cv::namedWindow("view");
        cv::startWindowThread();

        // 构造图像话题订阅对象
        std::vector<image_transport::Subscriber> ImageSub(topic_number);

        // 构造对应的回调函数指针数组
        void(PollingDisplay::*pImageCallback[])(const sensor_msgs::ImageConstPtr&) = { &PollingDisplay::imageCallback1, 
                                                                                       &PollingDisplay::imageCallback2, 
                                                                                       &PollingDisplay::imageCallback3, 
                                                                                       &PollingDisplay::imageCallback4,
                                                                                       &PollingDisplay::imageCallback5 };



        // 订阅话题，并调用回调函数
        for (int i=0; i<topic_number; ++i)
        {
            ImageSub[i] = it.subscribe(topic_vec[i], 20, pImageCallback[i], this);
        }



        // image_transport::Subscriber ImageSub_0 = it.subscribe(msgs_vec[0], 20, &PollingDisplay::imageCallback1, this);
        // image_transport::Subscriber ImageSub_1 = it.subscribe(msgs_vec[1], 20, &PollingDisplay::imageCallback2, this);
        keysTimer = n.createTimer(ros::Duration(0.5), &PollingDisplay::keysTimerCallback, this);
        showTimer = n.createTimer(ros::Duration(0.5), &PollingDisplay::showTimerCallback, this);
        while (ros::ok())
        {
            ros::spinOnce();
            if(!img2show.empty())
            {
                cv::imshow("view", img2show);
            }
        }
    }

    // TODO：

    void imageCallback1(const sensor_msgs::ImageConstPtr& img_msg)
    {
        if (index == 0)
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(img2show);
    }

    void imageCallback2(const sensor_msgs::ImageConstPtr& img_msg)
    {
        if (index == 1)
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(img2show);
    }

    void imageCallback3(const sensor_msgs::ImageConstPtr& img_msg)
    {
        if (index == 2)
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(img2show);
    }

    void imageCallback4(const sensor_msgs::ImageConstPtr& img_msg)
    {
        if (index == 3)
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(img2show);
    }

    void imageCallback5(const sensor_msgs::ImageConstPtr& img_msg)
    {
        if (index == 4)
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(img2show);
    }


    void keysTimerCallback(const ros::TimerEvent&)
    {
        int k_value = cv::waitKey(500);
        
        if (k_value != -1)
        {
            // 按键 r ： 清除所有显示
            if (k_value == 'r')
            {
                index_vec = std::vector<bool>(topic_number, false);
                show_index_vec = {};
            }


            if (k_value >= '0' && k_value  <= '0' + topic_number)
            {
                for (int i=0; i<=topic_number; ++i)
                {
                    if (k_value == '0' + i)
                    {
                        // 第一次按下，代表确认当前视角显示
                        if (index_vec[i] == false)
                        {
                            index_vec[i] = true;
                            cout << "Press " << i << endl;
                        }
                        // 第二次按下，代表取消当前视角显示
                        else
                        {
                            index_vec[i] = false;
                            cout << "Remove " << i << endl;
                        }
                    }
                }
            }
            else
            {
                cout << "Please press keys from 0 - " << topic_number << ". " << endl;
            }
        }
    }

    void showTimerCallback(const ros::TimerEvent&)
    {
        // 相机状态为 true， 即代表要显示，加入显示索引数组中
        // 相机状态为 false，即代表不显示，从显示索引数组中剔除
        for (int i=0; i<=topic_number; ++i)
        {
            if (index_vec[i] == true)
                show_index_vec.push_back(i-1);
            else
                std::remove(show_index_vec.begin(), show_index_vec.end(), i-1);
        }

        // 为防止重复加入，进行排序去重操作
        std::sort(show_index_vec.begin(), show_index_vec.end());
        show_index_vec.erase(unique(show_index_vec.begin(), show_index_vec.end()), show_index_vec.end());


        if (!show_index_vec.empty())
        {
            // 打印提示 显示列表
            cout << "Show Camera list: ";
            for (auto id : show_index_vec)
                cout << id + 1 << ", ";
            cout << endl;

            poll_id = (poll_id+1) % show_index_vec.size();
            index = show_index_vec[poll_id];
            cout << " Polling Show camera " << index + 1 << endl;
        }
        else
        {
            cout << "Show Camera list: empty ." << endl;
        }
    }

    ~PollingDisplay()
    {
        cv::destroyWindow("view");
    }

private:
    static int topic_number;
    int index;
    int poll_id;
    std::vector<std::string> topic_vec;         // 需要订阅的话题数组
    std::vector<bool>        index_vec;         // 相机的显示状态数组
    std::vector<int>         show_index_vec;    // 轮询显示的相机列表
    cv::Mat img2show;

    ros::NodeHandle n; 
    image_transport::ImageTransport it;
    ros::Timer showTimer;
    ros::Timer keysTimer;
};

int PollingDisplay::topic_number = 5;
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "polling_show");

    PollingDisplay polling_display;

    return 0;
}