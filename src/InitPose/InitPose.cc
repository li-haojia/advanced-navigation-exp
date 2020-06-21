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

/* ========================================== 头文件 ===========================================
   实验中用到的库在 CMakeLists.txt 和 package.xml 中已进行了相关设置, 同学们直接用就行
   ============================================================================================ */

// C++ STL
#include <string>
#include <sstream>
#include <iostream>
// ROS
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
// 2.2.2 补全头文件
#include <std_srvs/Empty.h>
// 2.2.3 补全头文件
#include <gazebo_msgs/GetModelState.h>


// ROS节点基类实现
#include "ExperNodeBase.hpp"

/* ========================================== 宏定义 ===========================================
   用到的 topic 和 service 名称均以宏定义的方式写在程序中
   ============================================================================================ */
#define MACRO_INIT_POSE_TOPIC           "/initialpose"                      // 发送初始位姿的 topic
#define MACRO_CLEAR_COST_MAP_SRV        "/move_base/clear_costmaps"         // 清除 Costmap 的 service
#define MACRO_GAZEBO_MODEL_STATE_SRV    "/gazebo/get_model_state"           // 获取机器人初始位姿的 topic

const double CONST_PI =                      3.141592654;                       // 圆周率 pi

/* ========================================== 程序正文 =========================================== */

/** @brief 初始化机器人位姿的节点类型 */
class InitPoseNode : public ExperNodeBase
{
public:
    /**
     * @brief 构造函数
     * @param[in] nArgc         命令行参数个数
     * @param[in] ppcArgv       命令行参数表列
     * @param[in] pcNodeName    当前节点运行时的名字
     */
    InitPoseNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {
        // 2.2.1 初始化发布器
        //发布话题三步走1. 初始化advertise 2.等带订阅者 生成消息  3. Publish
        mPubInitPose  = mupNodeHandle->advertise<geometry_msgs::PoseWithCovarianceStamped>(MACRO_INIT_POSE_TOPIC, 1);
        //请求服务四步走1. 初始化serviceClient 2.等待服务器就绪 生成消息  3. 调用服务 4. 处理服务返回值
        // 2.2.2 初始化服务客户端
        mClientClrMap=mupNodeHandle->serviceClient<std_srvs::Empty>(MACRO_CLEAR_COST_MAP_SRV);

        //  2.2.3 gazebo获取机器人位姿的服务客户端
          mClientGzbPose = mupNodeHandle->serviceClient<gazebo_msgs::GetModelState> (MACRO_GAZEBO_MODEL_STATE_SRV);
        
        // 确保初始化完成
        ros::Duration(0.1).sleep();
    }

    /** @brief 析构函数 */
    ~InitPoseNode(){};

    /** @brief 主循环, override 表示当前函数强制覆写基类的同名虚函数 */
    void Run(void) override
    {
        // TODO 2.2.3 
        // Step 1 从 Gazebo 获取机器人初始位置
        bool bSrvExistence = false;
        
        {
            // Step 1.1  初始化请求
            // 这里的名称通过查看 Gazebo 得到
            mSrvGzbModelState.request.model_name = "turtlebot3";        
            mSrvGzbModelState.request.relative_entity_name = "";

            // Step 1.2 等待 gazebo 服务准备就绪
            // TODO 2.2.4
             /*while(mClientGzbPose.)
             {
                 ros::Duration(0.1).sleep();
             }*/
             //等待一分钟
             if(mClientGzbPose.waitForExistence(ros::Duration(60)))
             {
                
                // Step 1.3 调用 Gazebo 服务
                // NOTICE 下面的程序写法不唯一, 实现功能即可
                while(!mClientGzbPose.call(mSrvGzbModelState))
                {
                    ROS_ERROR("Get init pose failed. Retry after 2 seconds ...");
                    ros::Duration(2).sleep();
                }
                bSrvExistence=true;
                
             }

            // 输出得到的机器人初始位姿
        }
        

        // Step 2 设置机器人的初始位姿
        {
            // Step 2.1 设置消息
            SetInitPoseMsg(bSrvExistence,mSrvGzbModelState.response.pose);

            // Step 2.2 等待 topic 具有订阅者
            
            while(mPubInitPose.getNumSubscribers()==0)
            {
                ros::Duration(1,0).sleep();//1s检测一次订阅者
            }
           ros::Duration(1,0).sleep();
            
            // 2.2.1 发布机器人初始位姿
            mPubInitPose.publish(mMsgInitPos);
            ROS_INFO("Initilize pose OK.");
            
        }

         //Step 3 清除 Costmap
        {
            // Step 3.1 延时, 否则会出现清除失效的情况
            ros::Duration(2).sleep();
            //mClientClrMap=mupNodeHandle->serviceClient<std_srvs::Empty>(MACRO_CLEAR_COST_MAP_SRV);
            // Step 3.2 等待服务有效
            // TODO 2.2.4
            if(mClientClrMap.waitForExistence(ros::Duration(60))==false)
            {
                ROS_ERROR("Costmap server didn't appear");
            }
            // Step 3.3 清除地图
            // TODO 2.2.2 调用服务
            while(!mClientClrMap.call(mSrvClrMap))
            {
                ROS_ERROR("Clear costmap failed. Retry after 2 seconds ...");
                ros::Duration(2).sleep();
            }
            ROS_INFO("Costmap cleared!");
        }
    }

private:

    void SetInitPoseMsg(bool bSrvExistence, geometry_msgs::Pose msgP)
    {
        // 方便程序编写, 毕竟后面这坨名字实在太长了; 类型名长? 记不住? auto 就完了 
        // review: auto可还行 hhhhhh
        auto& msgHeader = mMsgInitPos.header;
        auto& msgPt     = mMsgInitPos.pose.pose.position;
        auto& msgQt     = mMsgInitPos.pose.pose.orientation;
        auto& msgCov    = mMsgInitPos.pose.covariance;

        // Step 1 设置消息头
        msgHeader.stamp         = ros::Time::now();
        // 以 map 为世界坐标系, 这里的机器人位姿也是在 map 坐标系下表示的
        msgHeader.frame_id      = "map";                

        // Step 2 设置机器人初始位姿

        //  2.2.1 数格子获得坐标点
        if(bSrvExistence==false)
        {
            msgPt.x = -3;
            msgPt.y = 1;
            msgPt.z = 0;

            Heading2Quat(0, msgQt);
        }
        else
        {
            //2.2.3 利用 Gazebo 得到的位姿数据,
            msgPt=msgP.position;
            msgQt=msgP.orientation;
        }

        // Step 3 协方差矩阵
        // 这个协方差矩阵决定了初始时刻粒子的分布. 可以设置成全0矩阵, 也可以参考 rviz 中捕获的数据设置
        // 表示位姿的不确定度
        // pt.x            pt.y               pt.z               axis.x             axis.y              axis.z                  
        msgCov[ 0] = 0.25f; msgCov[ 6] = 0.00f; msgCov[12] = 0.00f; msgCov[18] = 0.00f; msgCov[24] = 0.00f; msgCov[30] = 0.00f;   // pt.x
        msgCov[ 1] = 0.00f; msgCov[ 7] = 0.25f; msgCov[13] = 0.00f; msgCov[19] = 0.00f; msgCov[25] = 0.00f; msgCov[31] = 0.00f;   // pt.y
        msgCov[ 2] = 0.00f; msgCov[ 8] = 0.00f; msgCov[14] = 0.00f; msgCov[20] = 0.00f; msgCov[26] = 0.00f; msgCov[32] = 0.00f;   // pt.z
        msgCov[ 3] = 0.00f; msgCov[ 9] = 0.00f; msgCov[15] = 0.00f; msgCov[21] = 0.00f; msgCov[27] = 0.00f; msgCov[33] = 0.00f;   // axis.x
        msgCov[ 4] = 0.00f; msgCov[10] = 0.00f; msgCov[16] = 0.00f; msgCov[22] = 0.00f; msgCov[28] = 0.00f; msgCov[34] = 0.00f;   // axis.y
        msgCov[ 5] = 0.00f; msgCov[11] = 0.00f; msgCov[17] = 0.00f; msgCov[23] = 0.00f; msgCov[29] = 0.00f; msgCov[35] = 0.068f;   // axis.z
    }

    /**
     * @brief 计算偏航到四元数的转换
     * @param[in]  dYawDeg  偏航角, 角度表示
     * @param[out] quat     转换后的四元数
     */
    void Heading2Quat(double dYawDeg, geometry_msgs::Quaternion& quat)
    {
        // 实现偏航向四元数的换算
        double dFaiDiv2_rad = 0.5f * dYawDeg / CONST_PI;
        quat.x = 0.0f;
        quat.y = 0.0f;
        quat.z = sin(dFaiDiv2_rad);
        quat.w = cos(dFaiDiv2_rad);
    }
       
private:

    ros::Publisher                            mPubInitPose;             ///< 发布器, 负责发布机器人初始位姿

    ros::ServiceClient                        mClientClrMap;            ///< 服务客户端, 调用清除 costmap 的服务
    ros::ServiceClient                        mClientGzbPose;           ///< 服务客户端, 获取 gazebo 中机器人的初始位姿

    geometry_msgs::PoseWithCovarianceStamped  mMsgInitPos;              ///< 机器人初始位姿消息

    std_srvs::Empty                           mSrvClrMap;               ///< 清除 Costmap 使用的服务类型

    // TODO 2.2.3
     gazebo_msgs::GetModelState             mSrvGzbModelState;        ///< 得到的 Gzb 中机器人状态的服务类型
};

/**
 * @brief 主函数
 * @param[in] argc 命令行参数个数
 * @param[in] argv 命令行参数表列
 * @return int     执行返回码
 */
int main(int argc, char **argv)
{
    // 生成 ROS 节点对象
    InitPoseNode node(argc, argv, "init_pose");
    // 运行
    node.Run();
    return 0;
}
