#include "asr_process.h"

//enable_wakeup订阅回调函数
void AsrProcess::enable_wakeup_callback(const std_msgs::Bool::ConstPtr& msg)
{
  // 在回调函数中处理接收到的消息
  if (msg->data) 
  {
    enable_wakeup = 1;
    ROS_INFO("enable_wakeup_callback true");
  }
  else 
  {
    enable_wakeup = 0;
    ROS_INFO("enable_wakeup_callback false");
  }
}

AsrProcess::~AsrProcess()
{

}

AsrProcess::AsrProcess() : nh("~")
{
  string asr_cfg;
  string sub_asr_id_topic;

  nh.param<string>("asr_cfg", asr_cfg, "asr.cfg");

  ROS_INFO("asr_cfg=%s",asr_cfg.c_str());

  parse_config_file(asr_cfg, asr_cmds);

  asr_id_sub = nh.subscribe("/asr_id", 10, &AsrProcess::asr_id_callback, this);

  enable_wakeup_sub = nh.subscribe("/enable_wakeup", 10, &AsrProcess::enable_wakeup_callback, this);



  tts_pub = nh.advertise<std_msgs::String>("/tts",10);//语音命令字符串发布

  action_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/action_cmd", 10);

  enable_tracking_pub = nh.advertise<std_msgs::Bool>("/enable_tracking", 10);
}

void AsrProcess::parse_asr_pub_tts(int asr_id)
{
    //获取回复语
    string reply_str = get_asr_reply(asr_cmds,asr_id);
    std_msgs::String msg;
    msg.data = reply_str;
    tts_pub.publish(msg);

    //处理和获取速度命令
    float turn_angle,distance;
    int new_action_mode = process_asr_cmd(asr_cmds,asr_id,turn_angle,distance,enable_tracking_pub);
    if(new_action_mode)
    {
        //发布数组数据
        vector<float> array = {new_action_mode,turn_angle,distance};
        std_msgs::Float32MultiArray array_msg;
        array_msg.data = array;
        action_cmd_pub.publish(array_msg);
    }
}


void AsrProcess::asr_id_callback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("asr_id_callback=%d",msg->data);
  if(enable_wakeup)//并且此时enable_wakeup是打开状态
    parse_asr_pub_tts(msg->data);
}

int main(int argc,char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "asr_process");

    ROS_INFO("main");

    AsrProcess asr_process;
    ros::spin();
    ros::shutdown();
    return 0;
}
