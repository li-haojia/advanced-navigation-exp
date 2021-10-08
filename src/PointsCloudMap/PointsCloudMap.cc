/**
 * @file ImageBase.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 基本图像的获取, 处理和发布
 * @version 0.1
 * @date 2020-06-04
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/* ========================================== 头文件 =========================================== */
// C++ STL
#include <string>
#include <sstream>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
// TODO 补充需要用到的头文件
#include <tf/transform_listener.h>
#include "tf/tf.h"
// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// ROS节点基类
#include "ExperNodeBase.hpp"
// 相机内参
#include "CameraIntrinsic.hpp"

/* ========================================== 宏定义 =========================================== */

#define CONST_PI 3.141592654f

// 彩色图像和深度图像 topic
#define MACRO_SRC_RGB_IMG_TOPIC                 "/camera/rgb/image_raw"
#define MACRO_SRC_DEPTH_IMG_TOPIC               "/camera/depth/image_raw"

// 彩色相机和深度相机参数 topic
#define MACRO_SRC_RGB_IMG_INFO_TOPIC            "/camera/rgb/camera_info"
#define MACRO_SRC_DEPTH_IMG_INFO_TOPIC          "/camera/depth/camera_info"

#define MACRO_LOCAL_POINT_CLOUD_TOPIC          "/point_cloud_map/local_map"
#define MACRO_GLOABAL_POINT_CLOUD_TOPIC          "/point_cloud_map/global_map"
// 坐标系名称
// TODO 3.3.3
#define MACRO_CAMERA_OPTICAL_NAME            "camera_rgb_optical_frame"
#define MACRO_MAP_LINK_NAME                  "map"

// 判断全局地图需要更新的阈值
#define CONST_DISTANCE_THRES_DEFAULT             0.2f
#define CONST_ANGLE_DEG_THRES_DEFAULT           10.0f

// 点云降采样和滤波使用参数
#define CONST_POINT_CLOUD_LEAF_SIZE_DEFAULT      0.05f
// 这个是整数
#define CONST_POINT_CLOUD_MEAN_K_DEFAULT        10
#define CONST_POINT_CLOUD_FILTER_THRES_DEFAULT   0.2f


// 图像缩小倍数的逆
#define CONST_IMAGE_SCALE_FACTOR_INV_DEFAULT     0.25f
// 图像更新频率
#define CONST_IMAGE_UPDATE_FREQUENCY_DEFAULT     1.00f
/* ========================================== 程序正文 =========================================== */

// PCMap = Point Cloud Map
class PCMapNode : public ExperNodeBase
{
public:
    // 构造函数
    PCMapNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {
        // Step 1 初始化 ImageTransport
        mupImgTrans.reset(new image_transport::ImageTransport(*mupNodeHandle));

        // Step 2 订阅彩色图像和深度图像, 注意队列长度为 1 最合适
        // Why?
        // TODO 3.3.1
        mImgSubRGB = mupImgTrans->subscribe(MACRO_SRC_RGB_IMG_TOPIC,1,&PCMapNode::ImageRGBCallback,this);
        mImgSubDepth = mupImgTrans->subscribe(MACRO_SRC_DEPTH_IMG_TOPIC,1,&PCMapNode::ImageDepthCallback,this);

        // Step 3 订阅深度相机内参
        // TODO 3.3.1
        mSubDepthCamInfo = mupNodeHandle->subscribe(MACRO_SRC_DEPTH_IMG_INFO_TOPIC,1,&PCMapNode::DepthCamInfoCallBack,this);
        
        // Step 4 设置局部点云(单帧点云)和全局点云发布器
        mPubLocalPCMap  = mupNodeHandle->advertise<sensor_msgs::PointCloud2>(MACRO_LOCAL_POINT_CLOUD_TOPIC,1);
        mPubGlobalPCMap = mupNodeHandle->advertise<sensor_msgs::PointCloud2>(MACRO_GLOABAL_POINT_CLOUD_TOPIC,1);;

        // Step 5 生成 TF 坐标变换树监听器
        //tf写法，首先要监听然后查询变换关系或者直接发布也行
        mupTFListener.reset(new tf::TransformListener());

        // Step 6 从参数服务器中获取参数
        mupNodeHandle->param<float>("/point_cloud_map/scale_factor_inv", 
                                    mfSCaleFactorInv,
                                    CONST_IMAGE_SCALE_FACTOR_INV_DEFAULT);

        mupNodeHandle->param<float>("/point_cloud_map/update_frequency", 
                                    mfUpdateFrequency,
                                    CONST_IMAGE_UPDATE_FREQUENCY_DEFAULT);

        mupNodeHandle->param<float>("/point_cloud_map/distance_threshold", 
                                    mfDistanceThres,
                                    CONST_DISTANCE_THRES_DEFAULT);
    
        mupNodeHandle->param<float>("/point_cloud_map/angle_deg_threshold", 
                                    mfAngleDegThres,
                                    CONST_ANGLE_DEG_THRES_DEFAULT);

        mupNodeHandle->param<float>("/point_cloud_map/point_cloud_leaf_size", 
                                    mfPCLeafSize,
                                    CONST_POINT_CLOUD_LEAF_SIZE_DEFAULT);

        mupNodeHandle->param<int>("/point_cloud_map/point_cloud_mean_K", 
                                    mnPCMeanK,
                                    CONST_POINT_CLOUD_MEAN_K_DEFAULT);

        mupNodeHandle->param<float>("/point_cloud_map/point_cloud_filter_thres", 
                                    mfPCFilterThres,
                                    CONST_POINT_CLOUD_FILTER_THRES_DEFAULT);

        // Step 7 帧频控制
        mupLoopRate.reset(new ros::Rate(mfUpdateFrequency));
        
        // Step 8 帧频控制
        // 确保初始化完成, 避免意外情况
        ros::Duration(1.0).sleep();
        ROS_INFO("Initialized.");

        // 初始化位姿状态
        GetCurrTransformInfo();
        mLastTransform = mCurrTransform;
    }

    /** @brief 析构函数 */
    ~PCMapNode(){}

    /** @brief 主循环 */
    void Run(void) override
    {
        while(ros::ok())
        {
            ros::spinOnce();
            mupLoopRate->sleep();
        }
    }

public:

    /**
     * @brief 彩色图像回调函数
     * @param[in] pMsg 图像指针
     */
    void ImageRGBCallback(const sensor_msgs::ImageConstPtr& pMsg)
    {
        // TODO 3.3.1
        // Step 1 转换成为 cv::Mat 格式图像
        cv::Mat bgrimg;
        try
        {
           bgrimg=cv_bridge::toCvShare(pMsg,"bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", pMsg->encoding.c_str());
        }
        // Step 2 改变大小, 更新成员变量 
        cv::resize(bgrimg, mimgScaledLastRGB, cv::Size(),
                   mfSCaleFactorInv, mfSCaleFactorInv);

    }

    /**
     * @brief 深度图像回调函数
     * @param[in] pMsg 图像指针
     */
    void ImageDepthCallback(const sensor_msgs::ImageConstPtr& pMsg)
    {

        // Step 1  获取 tf 变换信息
        GetCurrTransformInfo();

        // Step 2 获取深度图像
        // TODO 3.3.1
        cv::Mat imgRecv;
        try
        {
            // 这里会自动重载
            imgRecv = cv_bridge::toCvShare(pMsg)->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert depth image.");
        }

        // 缩放图像, 并更新 mimgScaledLastDepth
        // TODO 3.3.1 
        cv::resize(imgRecv, mimgScaledLastDepth, cv::Size(),
                   mfSCaleFactorInv, mfSCaleFactorInv);


        // Step 3 计算局部点云
        mnRows = mimgScaledLastDepth.rows;
        mnCols = mimgScaledLastDepth.cols;

        // 转换点云的工作
        static size_t nCnt = 0;
        // 隔10张处理一张
        if(nCnt % 10 != 0)       return;
        nCnt = 0;

        // 只是使用深度相机的内参, 内参要是还没有准备好, 那么也先跳过去了
        if(!mDepthCameraIntrinsic.isOK)  return;

        // 处理静态点云
        CreateLocalPointCloudMap();

        // 如果需要更新全局点云
        if(NeedUpdateGlobalMap())
        {
            // 那么就更新全局点云
            UpdateGlobalPointCloudMap();
        }
       
    }

    /**
     * @brief 深度相机参数到来时的回调函数, 要求只需要获取一次就足够了
     * @param[in] pMsg 消息指针
     */
    void DepthCamInfoCallBack(const sensor_msgs::CameraInfo::ConstPtr& pMsg)
    {
        // 注意乘 mfSCaleFactorInv, 通过 topic 直接获取的是原始图像对应的相机参数, 我们需要计算等效的相机参数
        // TODO 3.3.1 处理相机内参, 实现
        // mDepthCameraIntrinsic.fx = ....
                // 注意内参的变化, 这里需要计算缩小后图像的等效相机内参
        mDepthCameraIntrinsic.fx = pMsg->K[0] * mfSCaleFactorInv;
        mDepthCameraIntrinsic.fy = pMsg->K[4] * mfSCaleFactorInv;

        mDepthCameraIntrinsic.cx = pMsg->K[2] * mfSCaleFactorInv;
        mDepthCameraIntrinsic.cy = pMsg->K[5] * mfSCaleFactorInv;

        mDepthCameraIntrinsic.isOK = true;

        ROS_INFO_STREAM("Depth camera intrinsic: fx = " << mDepthCameraIntrinsic.fx << ", fy = " << mDepthCameraIntrinsic.fy << ", cx = " << mDepthCameraIntrinsic.cx << ", cy = " << mDepthCameraIntrinsic.cy);
        mSubDepthCamInfo.shutdown();
    }

    /** @brief 通过TF获取坐标变换 */
    void GetCurrTransformInfo(void)
    {
        tf::StampedTransform transform;
        try
        {
            // TODO 3.3.3 获得两个坐标系之间转换的关系; 注意两个坐标系别写反了
            //查询第二个在第一个的哪里
            mupTFListener->lookupTransform(MACRO_MAP_LINK_NAME,MACRO_CAMERA_OPTICAL_NAME,ros::Time(0),transform);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
        }

        // 格式转换, tf::Transform 支持很多数学计算, 不用我们自己手写了, 主要体现在 PCMapNode::NeedUpdateGlobalMap() 函数中
        mCurrTransform = tf::Transform(transform.getRotation(), transform.getOrigin());

        // 如果需要访问位姿变换信息, 可以按照下面的方式操作:
        // geometry_msgs::Point pt;
        // pt.x = mCurrTransform.getOrigin().x();
        // pt.y = mCurrTransform.getOrigin().y();
        // pt.z = mCurrTransform.getOrigin().z();
        
        // geometry_msgs::Quaternion qt;
        // qt.x = mCurrTransform.getRotation().x();
        // qt.y = mCurrTransform.getRotation().y();
        // qt.z = mCurrTransform.getRotation().z();
        // qt.w = mCurrTransform.getRotation().w();

        // ROS_INFO_STREAM("tf: [" << pt.x << ", " <<
        //                            pt.y << ", " <<
        //                            pt.z << "], [" <<
        //                            qt.x << ", " <<
        //                            qt.y << ", " <<
        //                            qt.z << ", " <<
        //                            qt.w << "]");   
    }

    /** @brief 创建局部点云 */
    void CreateLocalPointCloudMap(void)
    {
        // 清除局部点云
        mLocalPCMap.clear();

        // 遍历图像上的每一个点, 生成当前帧的点云数据
        for(size_t nIdY = 0; nIdY < mnRows; ++nIdY)
        {
            for(size_t nIdX = 0; nIdX < mnCols; ++nIdX)
            {
                float z = mimgScaledLastDepth.at<float>(nIdY, nIdX);
                // 如果深度值不合法, 就跳过去
                if(std::isnan(z))  continue;

                // TODO 3.3.2 计算空间点坐标
                //利用相机模型计算空间位置
                float u = nIdX;
                float v = nIdY;

                float x = z*(u-mDepthCameraIntrinsic.cx)/mDepthCameraIntrinsic.fx;
                float y = z*(v-mDepthCameraIntrinsic.cy)/mDepthCameraIntrinsic.fy;

                // b: color[0] g: color[1] r: color[2]
                //位置坑  由于光心重合，直接一一对应
                cv::Vec3b color = mimgScaledLastRGB.at<cv::Vec3b>(v,u);
                // TODO 3.3.2 新建点, 设置坐标和颜色, 并添加到局部点云地图中
                // 颜色坑rgb
                pcl::PointXYZRGB pt(color[2],color[1],color[0]);
                pt.x=x;pt.y=y;pt.z=z;
                mLocalPCMap.push_back(pt);
                
            }
        }

        // TODO 3.3.2 转换成为 ROS 的消息格式
        pcl::toROSMsg(mLocalPCMap,mMsgLocalPCMap); 
        // NOTICE 注意这里的 MACRO_CAMERA_OPTICAL_NAME 需要自己设置
        mMsgLocalPCMap.header.frame_id = MACRO_CAMERA_OPTICAL_NAME;
        mMsgLocalPCMap.header.stamp = ros::Time::now();
        // TODO 3.3.2 发布
        mPubLocalPCMap.publish(mMsgLocalPCMap);
    }

    /**
     * @brief 判断是否需要更新全局点云
     * @return true 
     * @return false 
     */
    bool NeedUpdateGlobalMap(void)
    {
        // 计算相对位移
        tf::Transform Tcl = mCurrTransform * mLastTransform.inverse();
        bool bDistance = (Tcl.getOrigin().length() > CONST_DISTANCE_THRES_DEFAULT);

        // 计算相对角度
        // 0 ~ 180 
        double dAngleDeg = Tcl.getRotation().getAngleShortestPath() / CONST_PI * 180.0;
        bool bANgle    = (dAngleDeg > CONST_ANGLE_DEG_THRES_DEFAULT);
        
        // 只要有一个条件满足, 就需要更新全局地图
        if(bDistance || bANgle)
        {
            // 更新
            mLastTransform = mCurrTransform;
            // DEBUG
            ROS_DEBUG_STREAM("dAngleDeg = " << dAngleDeg << ", distance = " << Tcl.getOrigin().length() << ", Update global map!");
            return true;
        }
        else
        {
            // 不满足
            return false;
        }
    }

    void UpdateGlobalPointCloudMap(void)
    {
        
        // Step 1 局部点云降采样, 滤波
        // TODO 3.3.4 注意相关参数 mfPCLeafSize mnPCMeanK mfPCFilterThres均已提供
       // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(mLocalPCMap.makeShared());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(new pcl::PointCloud<pcl::PointXYZRGB>);
        //降采样
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setInputCloud(mLocalPCMap.makeShared());
        filter.setLeafSize(mfPCLeafSize,mfPCLeafSize,mfPCLeafSize);
        filter.filter(*downsample);
        //移除外点 交换顺序是不是更好？
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(downsample);
        sor.setMeanK(mnPCMeanK);
        sor.setStddevMulThresh(mfPCFilterThres);
        sor.filter(*filtered);
        // Step 2 将点云中点的坐标转换到 map 坐标系下, 添加到全局点云中
        for(auto& oldPt: *filtered)//tmpPCMapFiltered)
        {
            // TODO 3.3.3 局部点云转换到 map 坐标系下
            tf::Vector3 tvBefore3d(oldPt.x, oldPt.y, oldPt.z);
            tf::Vector3 tvAfter3d = mCurrTransform * tvBefore3d;
            // 访问元素: tvAfter3d.x() 
            pcl::PointXYZRGB pt(oldPt);
            pt.x=tvAfter3d.x();
            pt.y=tvAfter3d.y();
            pt.z=tvAfter3d.z();
            mGlobalPCMap.push_back(pt);
            // TODO 3.3.3 将新点的坐标添加到全局点云中
        }

        // Step 3 对全局点云进行降采样
        // TODO 3.3.4
       // YOUR CODE
        filter.setInputCloud(mGlobalPCMap.makeShared());
        filter.filter(mGlobalPCMap);
        // Step 4 发布全局点云地图
        pcl::toROSMsg(mGlobalPCMap, mMsgGlobalPCMap);
        mMsgGlobalPCMap.header.frame_id = "map";
        // 发布
        mPubGlobalPCMap.publish(mMsgGlobalPCMap);
    }

private:

    // 这个节点的程序稍微麻烦些, 类的成员变量下面的基本上不需要我们添加其他的即可实现实验效果;
    // 当然你如果觉得不够也可以自己添加或修改

    std::unique_ptr<image_transport::ImageTransport> 
                                                 mupImgTrans;                   ///< Image Transporter

    ros::Subscriber                              mSubDepthCamInfo;              ///< 深度相机内参数订阅器
    ros::Publisher                               mPubLocalPCMap;                ///< 局部点云地图发布器
    ros::Publisher                               mPubGlobalPCMap;               ///< 全局点云地图发布器

    image_transport::Subscriber                  mImgSubRGB;                    ///< 彩色图像订阅器
    image_transport::Subscriber                  mImgSubDepth;                  ///< 深度图像订阅器

    pcl::PointCloud<pcl::PointXYZRGB>            mLocalPCMap;                   ///< 局部点云对象
    pcl::PointCloud<pcl::PointXYZRGB>            mGlobalPCMap;                  ///< 全局点云对象

    sensor_msgs::PointCloud2                     mMsgLocalPCMap;                ///< 局部点云消息
    sensor_msgs::PointCloud2                     mMsgGlobalPCMap;               ///< 全局点云消息

    CameraIntrinsic                              mDepthCameraIntrinsic;         ///< 深度相机内参

    cv::Mat                                      mimgScaledLastRGB;             ///< 上次获得的缩放后的彩色图像
    cv::Mat                                      mimgScaledLastDepth;           ///< 上次获得的缩放后的深度图像

    size_t                                       mnRows;                        ///< 缩放后图像的行数
    size_t                                       mnCols;                        ///< 缩放后图像的列数

    float                                        mfSCaleFactorInv;              ///< 图像缩放倍数
    float                                        mfUpdateFrequency;             ///< 点云地图更新频率
    float                                        mfDistanceThres;               ///< 判断是否需要更新全局地图的距离阈值
    float                                        mfAngleDegThres;               ///< 判断是否需要更新全局地图的角度阈值
    
    float                                        mfPCLeafSize;                  ///< 点云降采样用到的参数
    int                                          mnPCMeanK;                     ///< 点云滤波用到的参数
    float                                        mfPCFilterThres;               ///< 点云滤波用到的参数

    std::unique_ptr<tf::TransformListener>       mupTFListener;                 ///< TF 坐标变换监听器

    // 注意是深度相机光心在 map 坐标系下的坐标
    tf::Transform                                mCurrTransform;                ///< 当前次回调发生时的位姿
    tf::Transform                                mLastTransform;                ///< 上次全局地图更新时的位姿

    std::unique_ptr<ros::Rate>                   mupLoopRate;                   ///< 用于控制处理频率, 减少计算机算力负担
};

/**
 * @brief 主函数
 * @param[in] argc 命令行参数个数
 * @param[in] argv 命令行参数表列
 * @return int     运行返回值
 */
int main(int argc, char **argv)
{
    // 生成节点对象
    PCMapNode node(argc, argv, "point_cloud_map");
    // 运行
    node.Run();
    return 0;
}
