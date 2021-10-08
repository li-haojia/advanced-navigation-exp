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
#include <iostream>

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// TODO 补充
#include "sensor_msgs/CameraInfo.h"
// OpenCV
#include <opencv2/opencv.hpp>

// 节点基类
#include "ExperNodeBase.hpp"
// 相机内参
#include "CameraIntrinsic.hpp"

/* ========================================== 宏定义 =========================================== */
// 原图像 topic
#define MACRO_SRC_RGB_IMG_TOPIC "/camera/rgb/image_raw"
#define MACRO_SRC_DEPTH_IMG_TOPIC "/camera/depth/image_raw"

// 相机参数 topic
#define MACRO_SRC_RGB_IMG_INFO_TOPIC "/camera/rgb/camera_info"
#define MACRO_SRC_DEPTH_IMG_INFO_TOPIC "/camera/depth/camera_info"
#define MACRO_SRC_GRAY_IMG_RES_INFO_TOPIC "/camera/gray/camera_info"
// #define MACRO_SRC_DEPTH_IMG_INFO_TOPIC          "/camera/depth/camera_info"

// 图像缩小倍数的逆
#define CONST_IMAGE_DEFAULT_SCALE_FACTOR_INV 0.25

#define MACRO_SRC_COLORED_DEPTH_RES_TOPIC "/camera/color_depth"
#define MACRO_SRC_GRAY_RES_TOPIC "/camera/gray/image_raw"
/* ========================================== 程序正文 =========================================== */
class ImageBaseNode : public ExperNodeBase
{
public:
    /**
     * @brief 构造函数
     * @param[in] nArgc         命令行参数个数
     * @param[in] ppcArgv       命令行参数表列
     * @param[in] pcNodeName    当前节点名称
     */
    ImageBaseNode(int nArgc, char **ppcArgv, const char *pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {
        // 生成
        mupImgTrans.reset(new image_transport::ImageTransport(*mupNodeHandle));

        // 订阅彩色图像, 队列长度最好为1
        mImgSubRGB = mupImgTrans->subscribe(MACRO_SRC_RGB_IMG_TOPIC, 1,
                                            boost::bind(&ImageBaseNode::ImageRGBCallback, this, _1));

        // 这种写法也是可以的, 注意设置类的成员函数作为回调函数时, 需要传递 this 指针
        // mImgSubRGB = mupImgTrans->subscribe(MACRO_SRC_RGB_IMG_TOPIC, 1, this);

        // TODO 3.2.1 订阅深度图像
        mImgSubDepth = mupImgTrans->subscribe(MACRO_SRC_DEPTH_IMG_TOPIC, 1, &ImageBaseNode::ImageDepthCallback, this);

        // 发布
        mImgPubColoredDepth = mupImgTrans->advertise(MACRO_SRC_COLORED_DEPTH_RES_TOPIC, 1);

        mImgPubgray = mupImgTrans->advertise(MACRO_SRC_GRAY_RES_TOPIC, 1);
        // 订阅彩色相机内参
        mSubRGBCamInfo = mupNodeHandle->subscribe<sensor_msgs::CameraInfo>(MACRO_SRC_RGB_IMG_INFO_TOPIC, 1,
                                                                           boost::bind(&ImageBaseNode::RGBCamInfoCallBack, this, _1));

        // TODO 订阅深度相机内参
        mSubDepthCamInfo = mupNodeHandle->subscribe<sensor_msgs::CameraInfo>(MACRO_SRC_DEPTH_IMG_INFO_TOPIC, 1,
                                                                             boost::bind(&ImageBaseNode::DepthCamInfoCallBack, this, _1));

        mGrayCamera = mupNodeHandle->advertise<sensor_msgs::CameraInfo>(MACRO_SRC_GRAY_IMG_RES_INFO_TOPIC, 1);

        // 从参数服务器获取参数
        mupNodeHandle->param<float>("/image_base/scale_factor_inv",        // 名
                                    mfSCaleFactorInv,                      // 接收变量
                                    CONST_IMAGE_DEFAULT_SCALE_FACTOR_INV); // 默认值

        // 延时, 避免出现幺蛾子
        ros::Duration(0.1).sleep();
        ROS_INFO("Initialized, scaled factor inverse = %.4f", mfSCaleFactorInv);
    }

    /** @brief 析构函数 */
    ~ImageBaseNode() {}

    /** @brief 主循环 */
    void Run(void) override
    {
        // while (ros::ok())
        // {
        //     /* code */
        // }

        ros::spin();
    }

public:
    /**
     * @brief 彩色图像到来时的回调函数
     * @param[in] msg 消息指针
     */
    void ImageRGBCallback(const sensor_msgs::ImageConstPtr &pMsg)
    {
        // 生成图像
        cv::Mat imgRecv;
        try
        {
            // 利用 cv_bridge 转换, OpenCV彩色图像格式为 BGR 通道顺序, 每通道8bit
            imgRecv = cv_bridge::toCvShare(pMsg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            // 如果出现问题
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", pMsg->encoding.c_str());
        }

        // 大小缩放
        cv::resize(imgRecv, mimgScaledLastRGB, cv::Size(),
                   mfSCaleFactorInv, mfSCaleFactorInv);

        cv_bridge::CvImage grayimg;
        // TODO 3.2.2 彩色图像灰度化并发布图像
        cv::cvtColor(mimgScaledLastRGB, grayimg.image, CV_BGR2GRAY);
        grayimg.header = pMsg->header;
        grayimg.encoding = "mono8";
        mImgPubgray.publish(grayimg.toImageMsg());
        if (mRGBCameraIntrinsic.isOK)
        {
            sensor_msgs::CameraInfo c;
            c.width = grayimg.image.cols;
            c.height = grayimg.image.rows;
            c.K[0] = mRGBCameraIntrinsic.fx;
            c.K[4] = mRGBCameraIntrinsic.fy;
            c.K[2] = mRGBCameraIntrinsic.cx;
            c.K[5] = mRGBCameraIntrinsic.cy;
            mGrayCamera.publish(c);
        }

        // 显示图像
        cv::imshow("RGB Image", mimgScaledLastRGB);
        // 注意必须调用它,否则显示图像的 OpenCV 窗口不会刷新
        cv::waitKey(1);
    }

    /**
     * @brief 深度图像到来时的回调函数
     * @param[in] pMsg 
     */
    void ImageDepthCallback(const sensor_msgs::ImageConstPtr &pMsg)
    {
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

        // TODO 3.2.1 缩放图像
        cv::Mat mimgScaledLastDepth;
        cv::resize(imgRecv, mimgScaledLastDepth, cv::Size(),
                   mfSCaleFactorInv, mfSCaleFactorInv);

        // TODO 3.2.2 调用伪彩色化函数
        // 根据需要自由补充代码
        // 调用如下函数实现深度度向彩色图的转换
        cv::Mat imgcoloredDepth;
        ColoredDepth(/*深度图*/ mimgScaledLastDepth, /*转换后图像, 是彩色图*/ imgcoloredDepth);

        // TODO 3.2.2 发布图像
        // 消息类型转换  记住 cv_bridge::CvImage 本质就是加了header的cv::Mat
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgcoloredDepth).toImageMsg();
        // 发布消息
        mImgPubColoredDepth.publish(msg);

        // TODO 3.2.1 显示图像
        // 显示图像
        cv::imshow("Depth Image", imgcoloredDepth);
        // 注意必须调用它,否则显示图像的 OpenCV 窗口不会刷新
        cv::waitKey(1);
    }

    /**
     * @brief RGB相机参数到来时的回调函数
     * @param[in] pMsg 消息指针
     */
    void RGBCamInfoCallBack(const sensor_msgs::CameraInfo::ConstPtr &pMsg)
    {
        // 注意内参的变化, 这里需要计算缩小后图像的等效相机内参
        mRGBCameraIntrinsic.fx = pMsg->K[0] * mfSCaleFactorInv;
        mRGBCameraIntrinsic.fy = pMsg->K[4] * mfSCaleFactorInv;

        mRGBCameraIntrinsic.cx = pMsg->K[2] * mfSCaleFactorInv;
        mRGBCameraIntrinsic.cy = pMsg->K[5] * mfSCaleFactorInv;

        mRGBCameraIntrinsic.isOK = true;

        ROS_INFO_STREAM("RGB camera intrinsic: fx = " << mRGBCameraIntrinsic.fx << ", fy = " << mRGBCameraIntrinsic.fy << ", cx = " << mRGBCameraIntrinsic.cx << ", cy = " << mRGBCameraIntrinsic.cy);

        // 关闭订阅器
        mSubRGBCamInfo.shutdown();
    }

    /**
     * @brief 深度相机参数到来时的回调函数
     * @param[in] pMsg 
     */
    void DepthCamInfoCallBack(const sensor_msgs::CameraInfo::ConstPtr &pMsg)
    {
        mDepthCameraIntrinsic.fx = pMsg->K[0] * mfSCaleFactorInv;
        mDepthCameraIntrinsic.fy = pMsg->K[4] * mfSCaleFactorInv;

        mDepthCameraIntrinsic.cx = pMsg->K[2] * mfSCaleFactorInv;
        mDepthCameraIntrinsic.cy = pMsg->K[5] * mfSCaleFactorInv;

        mDepthCameraIntrinsic.isOK = true;

        ROS_INFO_STREAM("Depth camera intrinsic: fx = " << mDepthCameraIntrinsic.fx << ", fy = " << mDepthCameraIntrinsic.fy << ", cx = " << mDepthCameraIntrinsic.cx << ", cy = " << mDepthCameraIntrinsic.cy);
        mSubDepthCamInfo.shutdown();
    }

private:
    /**
     * @brief 深度图像转换成伪彩色图像
     * @param[in]  imgSrc 输入的深度图像
     * @param[out] imgDst 输出的伪彩色图像
     */
    void ColoredDepth(const cv::Mat &imgSrc, cv::Mat &imgDst)
    {
        cv::Mat imgAbs;
        // 数值映射
        cv::convertScaleAbs(imgSrc, imgAbs, 51.2, 0.0);
        // 转换为伪彩色图
        cv::applyColorMap(255 - imgAbs, imgDst, cv::COLORMAP_JET);

        size_t nRows = imgSrc.rows;
        size_t nCols = imgSrc.cols;

        cv::Vec3b cvBlack(0, 0, 0);

        // 处理一些特殊的 nan 像素
        for (size_t nIdY = 0; nIdY < nRows; ++nIdY)
        {
            for (size_t nIdX = 0; nIdX < nCols; ++nIdX)
            {
                // 深度值无效的点在深度图中以 nan 表示, 这里特殊处理一下, 作为黑色显示
                if (std::isnan(imgSrc.at<float>(nIdY, nIdX)))
                {
                    imgDst.at<cv::Vec3b>(nIdY, nIdX) = cvBlack;
                }
            }
        }
    }

private:
    std::unique_ptr<image_transport::ImageTransport> mupImgTrans; ///< ImageTransport 对象指针

    ros::Subscriber mSubRGBCamInfo;   ///< RGB相机参数订阅器
    ros::Subscriber mSubDepthCamInfo; ///< 深度相机参数订阅器
    ros::Publisher mGrayCamera;
    // HERE

    image_transport::Subscriber mImgSubRGB; ///< 彩色图像订阅器
    // TODO 3.2.1
    image_transport::Subscriber mImgSubDepth; ///< 深度图像订阅器
    // TODO 3.2.2 添加发布器
    image_transport::Publisher mImgPubgray;
    image_transport::Publisher mImgPubColoredDepth;

    CameraIntrinsic mRGBCameraIntrinsic;   ///< 彩色相机内参
    CameraIntrinsic mDepthCameraIntrinsic; ///< 深度相机内参

    cv::Mat mimgScaledLastRGB;   ///< 缩放后的彩色图像
    cv::Mat mimgScaledLastDepth; ///< 缩放后的深度图像

    float mfSCaleFactorInv; ///< 缩放因子
};

/**
 * @brief 主函数
 * @param[in] argc 命令行参数个数
 * @param[in] argv 命令行参数表列
 * @return int 退出状态
 */
int main(int argc, char **argv)
{
    ImageBaseNode node(argc, argv, "image_base");
    node.Run();
    return 0;
}
