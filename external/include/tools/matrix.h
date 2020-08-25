#ifndef MATRIX_H
#define MATRIX_H

#include "base/data_types.h"
#include "walgo/rotation.h"
#include "base/config.h"
#include <Eigen/Dense>

void pos2Matrix( const RobotPos &pos, Eigen::Matrix4d &m, EULERTYPE eulerType ) ;

void pos2Matrix(const RobotPos &pos, Eigen::Matrix3d &m, EULERTYPE eulerType );

void Matrix2pos(Eigen::Matrix4d &m,  RobotPos &pos, EULERTYPE eulerType );

void Matrix2pos(Eigen::Matrix3d &m,  RobotPos &pos, EULERTYPE eulerType );

void pos2Matrix( const RobotPos &pos, Eigen::Matrix4f &m, EULERTYPE eulerType ) ;

void pos2Matrix(const RobotPos &pos, Eigen::Matrix3f &m, EULERTYPE eulerType );

void Matrix2pos(Eigen::Matrix4f &m,  RobotPos &pos, EULERTYPE eulerType );

void Matrix2pos(Eigen::Matrix3f &m,  RobotPos &pos, EULERTYPE eulerType );

/// 计算空间任意两个TCP点的平移转换
/// pos1.tcp及pos2.tcp都要写入相应的TCP选点
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [out] pos2 pos2在基坐标中的位置
void abcConversion(  const RobotPos &pos1, RobotPos &pos2 ) ;

/// 计算空间任意两点的转换
/// @param [in] pos1InFlange pos1和法兰的坐标关系
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [in] pos2InFlange pos2和法兰的坐标关系
/// @param [out] pos2 pos2在基坐标中的位置
void abcConversion( const RobotTCP &tcp1, const RobotPos &pos1, RobotTCP tcp2, RobotPos &pos2 ) ;

/// 计算空间任意两点的转换
/// pos1.tcp要写入相应的TCP选点
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [out] pos2 pos2在基坐标中的位置
void abcConversion( const RobotPos &pos1, RobotTCP tcp2, RobotPos &pos2 ) ;

/// 计算空间任意两点的转换
/// @param [in] pos1InFlange pos1和法兰的坐标关系
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [in] pos2InFlange pos2和法兰的坐标关系
/// @param [out] pos2 pos2在基坐标中的位置
void posConversion( const RobotPos &pos1InFlange,  const RobotPos &pos1,
                   const RobotPos &pos2InFlange, RobotPos &pos2 ) ;

/// 计算空间任意两点的转换
/// @param [in] pos1InFlange pos1和法兰的坐标关系
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [in] pos2InFlange pos2和法兰的坐标关系
/// @param [out] pos2 pos2在基坐标中的位置
void posConversion( const RobotTCP &tcp1, const RobotPos &pos1, RobotTCP tcp2, RobotPos &pos2 ) ;

/// 计算空间任意两点的转换
/// pos1.tcp要写入相应的TCP选点
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [out] pos2 pos2在基坐标中的位置
void posConversion( const RobotPos &pos1, RobotTCP tcp2, RobotPos &pos2 ) ;

/// 计算空间任意两点的转换
/// pos1.tcp及pos2.tcp都要写入相应的TCP选点
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [out] pos2 pos2在基坐标中的位置
void posConversion( const RobotPos &pos1, RobotPos &pos2 ) ;

/// points 3d位姿点云
/// abs 这些点拟合的平面方程系数a*x+b*y+c*z=1（也是法向量）
/// 返回 拟合成功、失败
bool points2face(std::vector<RobotPos> &points, Eigen::Vector3d &abc);

#endif // MATRIX_H
