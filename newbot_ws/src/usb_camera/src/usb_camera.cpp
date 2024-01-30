#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <iostream>

#include <sys/time.h>
#include "v4l2.h"


//#include <opencv2/opencv.hpp>
//using namespace cv;

using namespace std;

int main(int argc, char** argv)
{

    unsigned int frame_cnt=0;
    double t=0,last_t=0;
    double fps;
    struct timeval tv;

    unsigned char *data_ptr;
    FrameBuf frame_buf;
    
    ros::init(argc, argv, "usb_camera");
    ros::NodeHandle nh("~");

    string pub_image_topic,dev_name,frame_id;
    int width,height,div;

    nh.param<string>("pub_image_topic", pub_image_topic, "/mjpeg_image");

    nh.param<string>("dev_name", dev_name, "/dev/video1");

    nh.param<string>("frame_id", frame_id, "camera");

    nh.param<int>("width", width, 1280);

    nh.param<int>("height", height, 720);
    
    nh.param<int>("div", div, 1);

    //cv::Mat img(height, width, CV_8UC3);

    ros::Publisher pub = nh.advertise<sensor_msgs::CompressedImage>(pub_image_topic, 1);
    
    sensor_msgs::CompressedImage msg;

    ros::Rate loop_rate(30);

    while(nh.ok())
    {
        V4l2 v4l2;

        if(v4l2.init_video(dev_name.c_str(),width,height)<0)//打开视频设备
        {
            ROS_WARN("v4l2 init video error!");
            v4l2.release_video();//释放设备
            sleep(3);
            continue;
        }

        while(nh.ok())
        {
            if(v4l2.get_data(&frame_buf)<0)//获取视频数据
            {
                ROS_WARN("v4l2 get data error!");
                v4l2.release_video();//释放设备
                sleep(3);
                break;
            }
            
            frame_cnt++;
            if(frame_cnt % 30 == 0)
            {
               gettimeofday(&tv, NULL);
               t = tv.tv_sec + tv.tv_usec/1000000.0;

               fps = 30.0/div/(t-last_t);
               ROS_DEBUG("mjpeg read fps %.2f",fps);
               last_t = t;
            }
            
            if(frame_cnt % div != 0)
                continue;
            
            //printf("frame_buf 0x%X %d\n",frame_buf.start,frame_buf.length);

            //解码显示
            //img = cv::imdecode(jpeg, IMREAD_COLOR);
            //imshow("img", img);
            //waitKey(1);

            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame_id; 
            msg.format = "jpeg";
            msg.data.assign(frame_buf.start, frame_buf.start+frame_buf.length);

            pub.publish(msg); //发布压缩图像

            loop_rate.sleep();//如果比30fps还有空余时间会sleep
        }
    }
}


