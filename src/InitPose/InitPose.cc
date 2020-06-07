/**
 * @file InitPose.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 设置初始的机器人位姿, 使 move_base 导航包获取的机器人初始位姿和 gazebo 中一致
 * @version 0.1
 * @date 2020-06-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <std_srvs/Empty.h>
// #include <sensor_msgs/LaserScan.h>
// TODO fill cmakelists.txt and package.yml
#include <gazebo_msgs/GetModelState.h>

#include <string>
#include <sstream>
#include <iostream>

#include "ExperNodeBase.hpp"

// TODO 
#define MACRO_INIT_POSE_TOPIC "/initialpose"
#define MACRO_CLEAR_COST_MAP_SRV "/move_base/clear_costmaps"
#define MACRO_GAZEBO_MODEL_STATE_SRV "/gazebo/get_model_state"


class InitPoseNode : public ExperNodeBase
{
public:
    InitPoseNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {
        mPubInitPose  = mupNodeHandle->advertise<geometry_msgs::PoseWithCovarianceStamped>(MACRO_INIT_POSE_TOPIC, 1);

        mClientClrMap = mupNodeHandle->serviceClient<std_srvs::Empty>(MACRO_CLEAR_COST_MAP_SRV);

        mClientGzbPose = mupNodeHandle->serviceClient<gazebo_msgs::GetModelState>(MACRO_GAZEBO_MODEL_STATE_SRV);
        
        // 确保初始化完成
        ros::Duration(0.1).sleep();
    }

    ~InitPoseNode(){};

    InitPoseNode(InitPoseNode& node) = delete;
    InitPoseNode(const InitPoseNode& node) = delete;

    void Run(void) override
    {
        mSrvGzbModelState.request.model_name = "turtlebot3";
        mSrvGzbModelState.request.relative_entity_name = "";

        // 等待, 直到准备好
        while(mClientGzbPose.exists() == false)
        {
            ros::Duration(0.1).sleep();
        }

        if(mClientGzbPose.call(mSrvGzbModelState))
        {
            auto& msgPostion = mSrvGzbModelState.response.pose.position;
            auto& msgQuat    = mSrvGzbModelState.response.pose.orientation;
            ROS_INFO_STREAM("==> position: [" << msgPostion.x << ", " << msgPostion.y << ", " << msgPostion.z << "]");
            ROS_INFO_STREAM("==> quat: [" << msgQuat.x << ", " << msgQuat.y << ", " << msgQuat.z << ", " << msgQuat.w << "]");
        }
        else
        {
            ROS_ERROR("mClientGzbPose.call failed.");
        }

        // 设置初始的位姿
        SetInitPoseMsg();


        // size_t nWaitSeconds = 40;
        // for(size_t nSecs = 0; nSecs < nWaitSeconds; ++nSecs)
        // {
        //     ROS_INFO_STREAM("Waiting " << nSecs << "/" << nWaitSeconds << "seconds for rviz ready, subscribers: " << mPubInitPose.getNumSubscribers());
        //     ros::Duration(1).sleep();
        // }

        while(ros::ok() && (mPubInitPose.getNumSubscribers() == 0 || mClientClrMap.exists() == false))
        {
            // TODO 添加对服务有效性的判断
            static size_t nCntSeconds = 0;
            // 如果还没有订阅者, 那么等等
            // TODO 修改输出内容
            ROS_INFO_STREAM("Waiting for " << nCntSeconds++ << " seconds for rviz ready, subscribers: " << mPubInitPose.getNumSubscribers());
            ros::Duration(1).sleep();
        }

        mPubInitPose.publish(mMsgInitPos);
        ROS_INFO("Initilize pose OK.");

        // 延时, 否则会出现清除失效的情况
        ros::Duration(2).sleep();
        while(!mClientClrMap.call(mSrvClrMap))
        {
            ROS_ERROR("Clear cost map failed. Retry after 2 seconds ...");
            ros::Duration(2).sleep();
        }
        ROS_INFO("Cost map cleared!");
    }

private:

    void SetInitPoseMsg(void)
    {
        ros::Time timeStamp = ros::Time::now();

        auto& msgHeader = mMsgInitPos.header;
        // auto& msgPt     = mMsgInitPos.pose.pose.position;
        // auto& msgQt     = mMsgInitPos.pose.pose.orientation;
        auto& msgCov    = mMsgInitPos.pose.covariance;

        // 注意这个并不需要我们设置
        // msgHeader.seq = 0;
        msgHeader.stamp.sec = timeStamp.sec;
        msgHeader.stamp.nsec = timeStamp.nsec;
        msgHeader.frame_id = "map";

        mMsgInitPos.pose.pose.position    = mSrvGzbModelState.response.pose.position;
        mMsgInitPos.pose.pose.orientation = mSrvGzbModelState.response.pose.orientation;
        
        // msgPt.x = -3.0f;
        // msgPt.y = 1.0f;
        // msgPt.z = 0.0f;

        // msgQt.x = 0.0f;
        // msgQt.y = 0.0f;
        // msgQt.z = 0.0f;
        // msgQt.w = 1.0f;

        // pt.x            pt.y               pt.z               axis.x             axis.y              axis.z                  
        msgCov[ 0] = 0.0f; msgCov[ 6] = 0.0f; msgCov[12] = 0.0f; msgCov[18] = 0.0f; msgCov[24] = 0.0f; msgCov[30] = 0.0f;   // pt.x
        msgCov[ 1] = 0.0f; msgCov[ 7] = 0.0f; msgCov[13] = 0.0f; msgCov[19] = 0.0f; msgCov[25] = 0.0f; msgCov[31] = 0.0f;   // pt.y
        msgCov[ 2] = 0.0f; msgCov[ 8] = 0.0f; msgCov[14] = 0.0f; msgCov[20] = 0.0f; msgCov[26] = 0.0f; msgCov[32] = 0.0f;   // pt.z
        msgCov[ 3] = 0.0f; msgCov[ 9] = 0.0f; msgCov[15] = 0.0f; msgCov[21] = 0.0f; msgCov[27] = 0.0f; msgCov[33] = 0.0f;   // axis.x
        msgCov[ 4] = 0.0f; msgCov[10] = 0.0f; msgCov[16] = 0.0f; msgCov[22] = 0.0f; msgCov[28] = 0.0f; msgCov[34] = 0.0f;   // axis.y
        msgCov[ 5] = 0.0f; msgCov[11] = 0.0f; msgCov[17] = 0.0f; msgCov[23] = 0.0f; msgCov[29] = 0.0f; msgCov[35] = 0.0f;   // axis.z
    }
       
private:
    ros::Publisher                            mPubInitPose;

    ros::ServiceClient                        mClientGzbPose;
    ros::ServiceClient                        mClientClrMap;

    
    geometry_msgs::PoseWithCovarianceStamped  mMsgInitPos;
    std_srvs::Empty                           mSrvClrMap;
    gazebo_msgs::GetModelState                mSrvGzbModelState;
};

int main(int argc, char **argv)
{
    InitPoseNode node(argc, argv, "init_pose");
    node.Run();
    return 0;
}
