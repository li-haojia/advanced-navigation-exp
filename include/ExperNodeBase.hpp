/**
 * @file ExperNodeBase.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 节点基类, 提供基本的功能实现
 * @version 0.1
 * @date 2020-06-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __EXPER_NODE_BASE_HPP__
#define __EXPER_NODE_BASE_HPP__

// 需要roscpp支持
#include <ros/ros.h>

/** @brief ROS节点基类 */
class ExperNodeBase
{
public:
    /**
     * @brief 构造函数
     * @param[in] nArgc         命令行参数个数
     * @param[in] ppcArgv       命令行参数表列
     * @param[in] pcNodeName    当前节点运行时的名字
     */
    ExperNodeBase(int nArgc, char** ppcArgv, const char* pcNodeName)
    {
        // 初始化当前 ROS 节点
        ros::init(nArgc, ppcArgv, pcNodeName);
        // 生成当前节点句柄
        mupNodeHandle.reset(new ros::NodeHandle());
    }

    /** @brief 析构函数, 没有什么可做的 */
    ~ExperNodeBase(){};

    // 禁用拷贝构造
    ExperNodeBase(ExperNodeBase& node)       = delete;
    ExperNodeBase(const ExperNodeBase& node) = delete;

    // 实现子类任务的主循环
    virtual void Run(void) = 0;

protected:

    std::unique_ptr<ros::NodeHandle> mupNodeHandle;     ///< 节点句柄对象的指针. 

};          // ExperNodeBase

#endif      // __EXPER_NODE_BASE_HPP__


// std::unique_ptr 可以认为是具有自动析构功能的指针类型, 这样可以避免 new 分配空间后忘记 delete 造成内存泄漏的尴尬
