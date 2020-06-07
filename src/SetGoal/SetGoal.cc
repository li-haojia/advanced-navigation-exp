/**
 * @file SetGoal.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 设置单次导航过程的路标点
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

#include "ExperNodeBase.hpp"

// TODO 
#define MACRO_RESULT_TOPIC    "/move_base/result"
#define MACRO_GOAL_POSE_TOPIC "/move_base/goal"




class SetGoalNode : public ExperNodeBase
{
public:
    SetGoalNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {

        mPubNextGoal = mupNodeHandle->advertise<move_base_msgs::MoveBaseActionGoal>(MACRO_GOAL_POSE_TOPIC, 1);
        mSubNavRes   = mupNodeHandle->subscribe(MACRO_RESULT_TOPIC, 1, &SetGoalNode::NavResCB);

        // mupLoopRate.reset(new ros::Rate(0.2f));

        // 确保初始化完成, 不然可能存在第一条消息发送不出去的情况
        ros::Duration(0.1).sleep();
    }

    ~SetGoalNode(){};

    void Run(void) override
    {
        double pdX[] = {-6.5f,-1.0f,6.0f,1.0f,-2.0f};
        double pdY[] = {-3.0f, 4.0f,-4.5f,3.0f,1.0f};

        // 没有错误产生
        mbErr = false;

        for(size_t nId = 0; nId < 6 && ros::ok() && !mbErr; ++nId)
        {
            // 延时两秒后开始
            ros::Duration(2).sleep();
            // 尚且没有完成
            mbOK  = false;
            ROS_INFO_STREAM("New Goal Position [" << pdX[nId] << ", " << pdY[nId] << "], moving ...");
            SetCurrGoal(pdX[nId], pdY[nId], 0.0f);
            while(mbOK == false && mbErr == false)
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
        else
        {
            ROS_ERROR("Failed.");
        }


        // double dX, dY;

        //  while(ros::ok())
        //  {
        //     std::cout << "Input goal position (x, y):" << std::endl;
        //     std::cin >> dX >> dY;
        //     if(dX == 0 && dY ==0)  break;

        //     ROS_INFO_STREAM("Goal Position [" << dX << ", " << dY << "], moving ...");
        //     SetCurrGoal(dX, dY, 0.0f);
        //     mPubNextGoal.publish(mMsgCurrGoal);
        //     ros::spinOnce();
        //     // ros::spin();
        //     mupLoopRate->sleep();
        //  }

        // ROS_INFO("OK.");
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

public:
    // 指示某次路径规划是否执行成功
    static bool            mbOK;
    static bool            mbErr;

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
        // TODO 实现角度向四元数的换算
        quat.x = 0.0f;
        quat.y = 0.0f;
        quat.z = 0.0f;
        quat.w = 1.0f;
    }
    
private:

    ros::Publisher  mPubNextGoal;

    ros::Subscriber mSubNavRes;

    move_base_msgs::MoveBaseActionGoal    mMsgCurrGoal;

    // std::unique_ptr<ros::Rate>       mupLoopRate;

    
};

class SetGoalNode;

bool SetGoalNode::mbErr;
bool SetGoalNode::mbOK;


int main(int argc, char **argv)
{
    SetGoalNode node(argc, argv, "set_goal");
    node.Run();

  return 0;
}

