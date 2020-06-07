/**
 * @file SeqGoal.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 设置一系列的的机器人位姿, 机器人逐步走过所有路径点
 * @version 0.1
 * @date 2020-06-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include "ExperNodeBase.hpp"


#include <signal.h>

// TODO 
#define MACRO_RESULT_TOPIC    "/move_base/result"
#define MACRO_GOAL_POSE_TOPIC "/move_base/goal"

#define CONST_PI    3.141592654f


class SeqGoalNode : public ExperNodeBase
{
public:
    SeqGoalNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {

        mPubNextGoal = mupNodeHandle->advertise<move_base_msgs::MoveBaseActionGoal>(MACRO_GOAL_POSE_TOPIC, 1);
        mSubNavRes   = mupNodeHandle->subscribe(MACRO_RESULT_TOPIC, 1, &SeqGoalNode::NavResCB);


        mifsLandMarks.open(ppcArgv[1]);
        mstrLandmarksFile = std::string(ppcArgv[1]);
        
        // mupLoopRate.reset(new ros::Rate(0.2f));

        // 确保初始化完成, 不然可能存在第一条消息发送不出去的情况
        ros::Duration(0.1).sleep();
    }

    ~SeqGoalNode()
    {
        if(mifsLandMarks.is_open())
        {
            mifsLandMarks.close();
        }
    };

    void Run(void) override
    {
        // double pdX[] = {-6.5f,-1.0f,6.0f,1.0f,-1.0f};
        // double pdY[] = {-3.0f, 4.0f,-4.5f,3.0f,1.0f};
        // double pdYaw[] = {0.0f, 90.0f,180.0f,-90.0f,0.0f};

        if(mifsLandMarks.is_open() == false)
        {
            ROS_FATAL_STREAM("Can not open landmarks file \"" << mstrLandmarksFile << "\"!");
            return ;
        }
        else
        {
            ROS_INFO_STREAM("Landmarks file \"" << mstrLandmarksFile << "\" opened.");
            // 预先读取前三行的注释
            std::string strDummy;
            getline(mifsLandMarks,strDummy);
            getline(mifsLandMarks,strDummy);
            getline(mifsLandMarks,strDummy);
        }

        ReadLandMarkNum();

        // 没有错误产生 
        // TODO this vars should be inited in construction function
        mbErr = false;
        mbQuit = false;

        for(size_t nId = 0; nId < mnMaxLandMarkId && ros::ok() && !mbErr && !mbQuit; ++nId)
        {
            // 延时两秒后开始
            ros::Duration(2).sleep();
            // 尚且没有完成
            mbOK  = false;
            double dX, dY, dYAW;
            ReadNextLandMark(dX, dY, dYAW);
            ROS_INFO_STREAM("New Goal Position [" << dX << ", " << dY << ", " << dYAW << "], moving ...");
            SetCurrGoal(dX, dY, dYAW);
            mPubNextGoal.publish(mMsgCurrGoal);
            while(mbOK == false && mbErr == false && mbQuit == false) 
            {
                ros::spinOnce();
                // 成功还是失败都不知道, 等; 每0.5s检查一次
                ros::Duration(0.5).sleep();
            }
            // 退出, 可能是成功, 也可能是失败
        }

        if(mbOK)
        {
            ROS_INFO("Done!");
        }
        else if(mbErr)
        {
            ROS_ERROR("Failed.");
        }

    }

    static void NavResCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& pMsgNavRes)
    {
        // ROS_INFO_STREAM("ON result:");

        uint8_t nState = pMsgNavRes->status.status;
        auto&   strGoalId = pMsgNavRes->status.goal_id.id;
        auto&   strDesc = pMsgNavRes->status.text;

        if(nState == 4)
        {
            // 出现错误
            ROS_ERROR_STREAM("Error on goal \"" << strGoalId << "\": " << strDesc);
            mbOK = false;
            mbErr = true;
        }
        else
        {
            ROS_INFO_STREAM("State on goal \"" << strGoalId << "\": " << strDesc);
            mbOK = true;
            mbErr = false;
        }
    }

    static void OnSignalInterrupt(int nSigId)
    {
        ROS_WARN("Ctrl+C Pressed, program terminated.");
        mbQuit = true;
    }

public:
    // 指示某次路径规划是否执行成功
    static bool            mbOK;
    static bool            mbErr;
    static bool            mbQuit;

private:

    void SetCurrGoal(double dX, double dY, double dHeadingDeg)
    {
        static size_t nCnt = 0;

        auto& msgHeader         = mMsgCurrGoal.header;
        auto& msgGoalID         = mMsgCurrGoal.goal_id;
        auto& msgTargetHeader   = mMsgCurrGoal.goal.target_pose.header;
        auto& msgPt             = mMsgCurrGoal.goal.target_pose.pose.position;
        auto& msgQt             = mMsgCurrGoal.goal.target_pose.pose.orientation;

        ros::Time timeStamp = ros::Time::now();

        msgHeader.seq = nCnt;
        msgHeader.stamp.sec  = timeStamp.sec;
        msgHeader.stamp.nsec = timeStamp.nsec;
        // NOTICE 抓到的数据显示这里没有赋予任何值
        msgHeader.frame_id   = "map";

        
        msgGoalID.stamp.sec  = timeStamp.sec;
        msgGoalID.stamp.nsec = timeStamp.nsec;


        std::stringstream ss;
        ss << "my_goal_" << nCnt << "_" << timeStamp.sec << "." << timeStamp.nsec;
        msgGoalID.id = ss.str().c_str();
        ROS_INFO_STREAM("Goal Id: " << ss.str());

        msgTargetHeader = msgHeader;

        msgPt.x = dX;
        msgPt.y = dY;
        msgPt.z = 0.0f;
        
        // 转换
        Heading2Quat(dHeadingDeg, msgQt);

        nCnt++;
    }

    void Heading2Quat(double dHeadingDeg, geometry_msgs::Quaternion& quat)
    {
        // 实现偏航向四元数的换算
        double dFaiDiv2_rad = 0.5f * dHeadingDeg / CONST_PI;
        quat.x = 0.0f;
        quat.y = 0.0f;
        quat.z = sin(dFaiDiv2_rad);
        quat.w = cos(dFaiDiv2_rad);
    }
    
    void ReadLandMarkNum(void)
    {
        std::string strDummy;
        std::stringstream ss;
        getline(mifsLandMarks,strDummy);
        ss << strDummy;
        ss >> mnMaxLandMarkId;
        ROS_INFO_STREAM("We have " << mnMaxLandMarkId << " landmark(s).");
    }

    void ReadNextLandMark(double& dX, double& dY, double& dYAW)
    {
        std::string strDummy;
        std::stringstream ss;
        getline(mifsLandMarks,strDummy);
        ss << strDummy;
        ss >> dX >> dY >> dYAW;
    }

private:

    ros::Publisher  mPubNextGoal;
    ros::Subscriber mSubNavRes;

    move_base_msgs::MoveBaseActionGoal    mMsgCurrGoal;

    std::ifstream   mifsLandMarks;
    std::string     mstrLandmarksFile;

    size_t          mnMaxLandMarkId;
};


bool SeqGoalNode::mbErr;
bool SeqGoalNode::mbOK;
bool SeqGoalNode::mbQuit;


int main(int argc, char **argv)
{
    // 对于按照位置解析的参数, 必须要这样写, 不然的话 roslaunch 运行的时候会额外送很多参数, 这里会认为给的参数不正确
    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " landmarks_file" << std::endl;
        return 0;
    }

    SeqGoalNode node(argc, argv, "seq_goal");

    signal(SIGINT, &SeqGoalNode::OnSignalInterrupt);


    node.Run();

  return 0;
}

