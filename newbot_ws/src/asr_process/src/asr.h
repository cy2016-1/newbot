#include<vector>
#include<string>
#include<map>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

using namespace std;

typedef struct AsrCmd
{
    string key;
    string cmd;
    string reply;
};


void parse_config_file(string fileName,vector<AsrCmd> &asr_cmds);

string get_asr_reply(vector<AsrCmd> &asr_cmds,int asr_id);

int process_asr_cmd(vector<AsrCmd> &asr_cmds,int asr_id,float &turn_angle,float &distance,ros::Publisher &enable_tracking_pub);