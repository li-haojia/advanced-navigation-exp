/**
 * @file OctoMap.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 根据 Global Point Cloud Map 创建八叉树地图
 * @version 0.1
 * @date 2020-06-04
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/* ========================================== 头文件 =========================================== */
// C++ STL
#include <string>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/point_cloud.h>

// Octomap
#include <octomap/OcTree.h>

// ROS 节点基类
#include "ExperNodeBase.hpp"

/* ========================================== 宏定义 =========================================== */
// 八叉树地图的分辨率, 也就是最小的小方格的大小
#define CONST_OCTOMAP_RESOLUTION_DEFAULT 0.05f
#define MACRO_GLOABAL_POINT_CLOUD_TOPIC          "/point_cloud_map/global_map"
/* ========================================== 程序正文 =========================================== */
/** @brief 八叉树建图的节点类 */
class OctoMapNode : public ExperNodeBase
{
public:
    /**
     * @brief 构造函数
     * @param[in] nArgc         命令行参数个数
     * @param[in] ppcArgv       命令行参数表列
     * @param[in] pcNodeName    当前节点名称
     */
    OctoMapNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {
        // TODO 补充完善从参数服务器获取节点数据的操作, 参数名称为 "/octo_map/resolution", 类型 float, 
        // 获取后保存在成员变量 mfOctoResolution 中, 如果参数服务器中不存在该参数则使用宏 CONST_OCTOMAP_RESOLUTION_DEFAULT 作为默认值
        // YOUR CODE
        mupNodeHandle->param<float>("/octo_map/resolution", 
                                    mfOctoResolution,
                                    CONST_OCTOMAP_RESOLUTION_DEFAULT);
        // TODO 初始化点云订阅器, 订阅 point_cloud_map 节点的全局点云
        mSubPCL = mupNodeHandle->subscribe(MACRO_GLOABAL_POINT_CLOUD_TOPIC,1,&OctoMapNode::GlobalPCMapCallBack,this);

        // 八叉树地图发布器
        mPubOctoMap = mupNodeHandle->advertise<octomap_msgs::Octomap>("/octomap", 1);

        // 生成八叉树地图对象
        mupOctTree.reset(new octomap::OcTree(mfOctoResolution));
    }

    /** @brief 析构函数 */
    ~OctoMapNode() {}  

    /** @brief 主循环 */
    void Run(void) override
    {
        ros::spin();
    }

public:

    /**
     * @brief 全局点云到来时的回调函数
     * @param[in] msgPCL 消息
     */
    void GlobalPCMapCallBack(const sensor_msgs::PointCloud2 &msgPCL)
    {
        ROS_DEBUG("Global Map Get!");

        // Step 1 点云消息转换成为 PCL 格式
        // TODO 
        pcl::PointCloud<pcl::PointXYZRGB>  mGlobalPCMap;
        pcl::fromROSMsg(msgPCL,mGlobalPCMap);
        // Step 2 清除八叉树地图
        mupOctTree->clear();

        // Step 3 向八叉树地图中添加点
        for(const auto& pt : mGlobalPCMap)
        {
            mupOctTree->updateNode(octomap::point3d(pt.x, pt.y, pt.z), true);
        }

        // Step 4 剪枝, 发挥八叉树特性
        mupOctTree->prune();
        
        // Step 5 发布八叉树消息
        //设置header
        mMsgOctoMap.header.frame_id = "map";
        mMsgOctoMap.header.stamp = ros::Time::now();

        if (octomap_msgs::binaryMapToMsg(*mupOctTree, mMsgOctoMap))
            //转换成功，可以发布了
            mPubOctoMap.publish(mMsgOctoMap);
        else
            ROS_ERROR("Error serializing OctoMap");
    }

private:

    ros::Subscriber                         mSubPCL;            ///< 点云订阅器
    ros::Publisher                          mPubOctoMap;        ///< 八叉树地图发布器
    float                                   mfOctoResolution;   ///< 八叉树地图的分辨率
    std::unique_ptr<octomap::OcTree>        mupOctTree;         ///< 八叉树地图对象的智能指针

    octomap_msgs::Octomap                   mMsgOctoMap;        ///< 八叉树地图的 ROS 消息

};

/**
 * @brief 主函数
 * 
 * @param[in] argc 命令行参数个数
 * @param[in] argv 命令行参数表列
 * @return int     运行返回值
 */
int main(int argc, char **argv)
{
    OctoMapNode node(argc, argv, "octo_map");
    node.Run();
    return 0;
}
