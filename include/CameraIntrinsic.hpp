/**
 * @file CameraIntrinsic.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief  提供相机内参数据类型
 * @version 0.1
 * @date 2020-06-05
 * 
 * @copyright Copyright (c) 2020
 * 
 */


#ifndef __CAMERA_INTRINSIC_HPP__
#define __CAMERA_INTRINSIC_HPP__

// 相机内参数据类型
struct CameraIntrinsic
{
    bool   isOK;
    double fx;
    double fy;
    double cx;
    double cy;

    CameraIntrinsic():isOK(false), fx(0.0f), fy(0.0f), cx(0.0f), cy(0.0f){};
};

#endif      // __CAMERA_INTRINSIC_HPP__