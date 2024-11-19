//
// Created by fish on 2024/11/16.
//

#ifndef APP_CONF_H
#define APP_CONF_H

/*
 *  app_conf.h
 *  用来控制系统的一些变量，主要以 define 的形式存在
 */

// 底盘 - 麦克纳姆轮
// #define COMPILE_CHASSIS_MECANUM

// 底盘 - 全向轮
#define COMPILE_CHASSIS_OMNI

// 云台
// #define COMPILE_GIMBAL

// 双控制器
#define USE_DUAL_CONTROLLERS

// 检查是否有冲突（？）
#if defined(COMPILE_CHASSIS_MECANUM) + defined(COMPILE_CHASSIS_OMNI) > 1
#error Invalid Definition
#endif

#endif //APP_CONF_H
