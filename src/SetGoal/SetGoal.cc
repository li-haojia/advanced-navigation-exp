/**
 * @file SetGoal.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 设置单次导航过程的路标点并获取导航状态
 * @version 0.1
 * @date 2020-06-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/* ========================================== 头文件 =========================================== */
// C++ STL
#include <string>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
// TODO 补充消息头文件
// #include <____>
// ...

// 节点基类
#include "ExperNodeBase.hpp"

/* ========================================== 宏定义 =========================================== */
#define MACRO_GOAL_POSE_TOPIC   "/move_base/goal"       // 发送导航目标点的 topic
// TODO 2.3.2 填写你选择的 topic
// #define MACRO_RESULT_TOPIC      "______"             // 获取导航结果的 topic

#define CONST_PI                3.141592654f            // 圆周率

/* ========================================== 程序正文 =========================================== */

/** @brief 设置机器人导航路标点的节点 */
class SetGoalNode : public ExperNodeBase
{
public:
    /**
     * @brief 构造函数
     * @param[in] nArgc         命令行参数个数
     * @param[in] ppcArgv       命令行参数表列
     * @param[in] pcNodeName    当前节点运行时的名字
     */
    SetGoalNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {

        // TODO 2.3.1 设置发布器
        // mPubNextGoal = _______________(MACRO_GOAL_POSE_TOPIC, 1);
        // TODO 2.3.2 设置导航状态订阅器. 注意回调函数是类的成员函数, 或者是类的静态函数时, 后面应该怎么写; 方式不唯一
        // mSubNavRes   = _______________(MACRO_RESULT_TOPIC, 1, _____________________);

        // 确保初始化完成, 不然可能存在第一条消息发送不出去的情况
        ros::Duration(0.1).sleep();
    }

    /** @brief 析构函数 */
    ~SetGoalNode(){};

    /** @brief 主循环 */
    void Run(void) override
    {
        // TODO 2.3.1 发布导航目标点
        // 如果需要自己添加类成员变量或成员函数, 请随意添加
        // <YOUR CODE>

        // TODO 2.3.2 获取导航执行结果, 并显示在屏幕上
        // <YOUR CODE>
    }

    // TODO 2.3.2 导航执行结果的回调函数
    /* _______________
    {
       <YOUR CODE>
    }
    */

private:

    /**
     * @brief 根据传入坐标生成路径点
     * @param[in] dX            坐标X
     * @param[in] dY            坐标Y
     * @param[in] dYawDeg       偏航角, 角度制表示
     */
    void SetCurrGoal(double dX, double dY, double dYawDeg)
    {
        auto& msgHeader         = mMsgCurrGoal.header;
        auto& msgGoalID         = mMsgCurrGoal.goal_id;
        auto& msgTargetHeader   = mMsgCurrGoal.goal.target_pose.header;
        auto& msgPt             = mMsgCurrGoal.goal.target_pose.pose.position;
        auto& msgQt             = mMsgCurrGoal.goal.target_pose.pose.orientation;

        // Step 1 初始化消息头
        msgHeader.seq = nCnt;
        msgHeader.stamp = ros::Time::now();
        msgHeader.frame_id   = "map";
        
        msgGoalID.stamp = msgHeader.stamp;
        msgTargetHeader = msgHeader;

        /* NOTICE 这里的 id 设置有点意思, 有时间的同学可以进行下面的几个小测试:
         *      - 如果设置的id为空, 通过 rostopic echo 查看到发送的消息中, 这个字段是什么内容? 
         *      - 同样查看 rviz 发送的消息, 这个字段是什么内容? 多发送几次, 你能找到什么规律?
         *      - 将这里的id改为任何一个固定的非空字符串如"my_goal", 通过 rostopic echo 查看到这个字段是什么?
         *      - 修改坐标多发送几次呢?
         *      - 你发现 rviz 中机器人执行导航的过程的不正常情况了吗?
        */
        std::stringstream ss;
        ss << "my_goal_" << nCnt << "_" << timeStamp.sec << "." << timeStamp.nsec;
        msgGoalID.id = ss.str().c_str();
        ROS_DEBUG_STREAM("Goal Id: " << ss.str());

        // Step 2 设置目标点
        // TODO 2.3.1 设置目标点
        // <YOUR CODE>
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

    // TODO 2.3.2 补全类型
    ros::Publisher  mPubNextGoal;           // 路标点发布器
    ros::Subscriber mSubNavRes;             // 导航状态订阅器
    // _______________    mMsgCurrGoal;
    
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
    SetGoalNode node(argc, argv, "set_goal");
    // 运行
    node.Run();
    return 0;
}

