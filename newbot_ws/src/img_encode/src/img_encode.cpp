#include "img_encode.h"

ImgEncode::ImgEncode() : nh("~")
{
    string sub_image_topic;
    int width,height,jpeg_quality;
    nh.param<string>("sub_image_topic", sub_image_topic, "/camera/image_raw");

    nh.param<int>("width", width, 640);
    nh.param<int>("height", height, 360);
    nh.param<int>("jpeg_quality", jpeg_quality, 80);
    this->jpeg_quality = jpeg_quality;

    image_sub = nh.subscribe(sub_image_topic, 10, &ImgEncode::image_callback,this);
    jpeg_pub = nh.advertise<sensor_msgs::CompressedImage>(sub_image_topic+"/compressed", 10);

#if(USE_ARM_LIB==1)
    mpp_encode.init(width,height,jpeg_quality);
#endif

}


void ImgEncode::image_callback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg,"rgb8");//ROS消息转OPENCV

//auto t1 = std::chrono::system_clock::now();

#if(USE_ARM_LIB==1)
    //硬转换RGB->YUV420P
    cv::Mat image_yuv(cv_ptr->image.rows * 3/2, cv_ptr->image.cols, CV_8UC1);
    rga_cvtcolor(cv_ptr->image, image_yuv);//cv_ptr-->image_yuv420p
#endif

//auto t2 = std::chrono::system_clock::now();

#if(USE_ARM_LIB==1)
    //硬编码YUV420P->JPEG
    mpp_encode.encode(image_yuv.data, cv_ptr->image.cols * cv_ptr->image.rows * 3/2, msg_pub.data);//image_yuv-->msg_pub.data
#else
    //软编码RGB->JPEG
    cv::Mat image_bgr;
    cv::cvtColor(cv_ptr->image, image_bgr, cv::COLOR_RGB2BGR);
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(jpeg_quality);  // JPEG压缩质量
    cv::imencode(".jpg", image_bgr, msg_pub.data, compression_params);
#endif

//auto t3 = std::chrono::system_clock::now();

    msg_pub.header = msg->header;//使用原有时间戳
    msg_pub.format = "jpeg";
    jpeg_pub.publish(msg_pub); //发布压缩图像


// auto t4 = std::chrono::system_clock::now();

// ROS_WARN_THROTTLE(1,"yuv=%d encode=%d pub=%d ms",
// std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count(),
// std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count(),
// std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
// );

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "img_encode");
    ImgEncode img_encode;
    ros::spin();
    return 0;
}


