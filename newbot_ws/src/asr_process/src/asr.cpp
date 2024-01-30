#include "asr.h"


#include <iostream>
#include <fstream>
#include <map>
#include <string>


using namespace std;

void parse_config_file(string fileName,vector<AsrCmd> &asr_cmds) 
{
    ifstream fin(fileName);
    if (!fin.is_open()) {
        cerr << "Failed to open file: " << fileName << endl;
        return;
    }

    string line;
    while (getline(fin, line)) 
    {
        AsrCmd asr_cmd;
        asr_cmd.key = line.substr(0, line.find("="));
        asr_cmd.cmd = line.substr(line.find("=") + 1, line.find("@") - line.find("=") - 1);
        asr_cmd.reply = line.substr(line.find("@") + 1);

        asr_cmds.push_back(asr_cmd);
    }

    fin.close();
}

int hex2dec(int hex)
{
    char hex_str[10];
    sprintf(hex_str,"%02x",hex);
    int num1 = hex_str[0] - '0';
    int num2 = hex_str[1] - '0';
    return num1 * 10 + num2;
}

string get_asr_reply(vector<AsrCmd> &asr_cmds,int asr_id)
{
    int id_index = hex2dec(asr_id)-1;//asr_id是从1开始记的

    if(id_index >= asr_cmds.size())
        return "ASR ID "+to_string(asr_id)+" 错误";
    
    return asr_cmds[id_index].reply + "#" + asr_cmds[id_index].key;
}

int process_asr_cmd(vector<AsrCmd> &asr_cmds,int asr_id,float &turn_angle,float &distance,ros::Publisher &enable_tracking_pub)
{
    int id_index = hex2dec(asr_id)-1;//asr_id是从1开始记的

    if(id_index >= asr_cmds.size())
        return 0;

    string key = asr_cmds[id_index].key;

    if(key=="turn_on_lidar")
    {
        system("gpio mode 20 out && gpio write 20 1");
        return 0;
    }
    else if(key=="trun_off_lidar")
    {
        system("gpio mode 20 out && gpio write 20 0");
        return 0;
    }
    else if(key=="reduce_lidar")
    {
        system("gpio mode 20 in");
        return 0;
    }
    else if(key=="reboot")
    {
        system("sleep 3 && reboot");
        return 0;
    }
    else if(key=="poweroff")
    {
        system("sleep 3 && poweroff");
        return 0;
    }
    else if(key=="tracking_person")
    {
        std_msgs::Bool msg;
        msg.data = true;
        enable_tracking_pub.publish(msg);
        return 0;
    }
    else if(key=="cancel_tracking")
    {
        std_msgs::Bool msg;
        msg.data = false;
        enable_tracking_pub.publish(msg);
        return 0;
    }

    else if(key=="stop")
    {
        std_msgs::Bool msg;
        msg.data = false;
        enable_tracking_pub.publish(msg);//停止命令也可以取消跟踪

        turn_angle = 0;
        distance = 0;
        return 1;
    }
    else if(key=="dance")
    {
        turn_angle = 0;
        distance = 0;
        return 2;//2表示跳舞
    }


    int value = 0;
    // 查找下划线的位置
    size_t underscorePos = key.find("_");
    if (underscorePos != std::string::npos)
    {
        // 解析指令部分
        string command = key.substr(0, underscorePos);
        
        // 解析数字部分
        std::istringstream iss(key.substr(underscorePos + 1));
        iss >> value;
    }

    if(value==0)
        return 0;

    if (key.find("forward") != string::npos)
    {
        turn_angle = 0;
        distance = value*0.01;//厘米转米
        return 1;
    }
    else if (key.find("backward") != string::npos) 
    {
        turn_angle = 0;
        distance = -value*0.01;//厘米转米
        return 1;
    }
    else if (key.find("left") != string::npos) 
    {
        turn_angle = value;
        distance = 0;
        return 1;
    }
    else if (key.find("right") != string::npos) 
    {
        turn_angle = -value;
        distance = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}
