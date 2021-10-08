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

#include <ros/ros.h>

class ExperNodeBase
{
public:
    ExperNodeBase(int nArgc, char** ppcArgv, const char* pcNodeName)
    {
        ros::init(nArgc, ppcArgv, pcNodeName);
        mupNodeHandle.reset(new ros::NodeHandle());
    }

    ~ExperNodeBase(){};

    ExperNodeBase(ExperNodeBase& node)       = delete;
    ExperNodeBase(const ExperNodeBase& node) = delete;

    virtual void Run(void)           = 0;

protected:
    std::unique_ptr<ros::NodeHandle> mupNodeHandle;

};          // ExperNodeBase

#endif      // __EXPER_NODE_BASE_HPP__
