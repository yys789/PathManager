#ifndef _ROBOT_MOVE_H
#define _ROBOT_MOVE_H

#include <queue>

#include "walgo/quaternion.h"
#include "walgo/rotation.h"
#include "base/data_types.h"
#include "walgo/interppath.h"

using namespace std;

// 根据起点和终点生成最终的相对路徑划分，包含位置和角度的渐变，变化速度和加速度取两者较小运动
void absolute2Eular( RobotPos begin, RobotPos end,
              double maxSpeed, double acc,
              const double maxAngleSpeed, const double angleAcc,
              std::queue<RobotPos> &path ) ;

// 根据相对路徑生成最终的相对路徑划分
void relative2Eular( const RobotPos &begin, const RobotPos &relativePos,
              double maxSpeed, double acc,
              const double maxAngleSpeed, const double angleAcc,
              std::queue<RobotPos> &path ) ;

/// 规划机器轴的旋转
/// @param [in] relativeAxle 轴的旋转角度
/// @param [in] maxSpeed 旋转的速度
/// @param [in] acc 旋转的加速度
/// @param [out] path 计算后的旋转路径
void axleRelative( const RobotAxle &relativeAxle, double maxSpeed, double acc, std::queue<RobotAxle> &path ) ;

/// 路径拟合
/// @param [in] in_vec 原始的路径
/// @out_vec[out] out_vec 计算后的路径
/// @param [in] multiple 速度缩放倍数
void pathFit( const std::vector<RobotPos> &in_vec, std::vector<RobotPos> &out_vec, double multiple );
#endif
