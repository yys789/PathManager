#ifndef COORD_H
#define COORD_H

#include <vector>
#include "base/data_types.h"
#include "walgo/point.h"
#include <Eigen/Dense>

using namespace walgo;
using namespace std;

#define SYS "etc/sys.info"

/// 计算某个点绕基坐标每个轴旋转后的欧拉角度
/// @param [in] flangePos   当前法兰坐标
/// @param [in] pointInFlange  要计算的旋转点在法兰中的位置
/// @param [in] rotation_vec  旋转的坐标轴次序[z, y, x]
/// @param [out] newPos 计算出的最终法兰欧拉角
void pointRotate( const RobotPos &flangePos, const RobotPos &pointInFlange, const std::vector<RobotPos> &rotation_vec, RobotPos &newPos );

/// 坐标系变化
/// @param [in] currPos point 在坐标系A中的坐标
/// @param [in] rotatePos 坐标系A旋转, point在空间中的位置不变
/// @param [out] outPos point 在坐标系旋转后的坐标
void coordinateRotate( const RobotPos &currPos, const RobotPos &rotatePos, RobotPos &outPos ) ;

/// 计算两点的旋转矩阵
/// @param [in] posA
/// @param [in] posB
Eigen::Matrix4d  getRotateMatrix( const RobotPos &posA, const RobotPos &posB );

/// 根据两点间的旋转矩阵求另一点的坐标
/// @param [in] posA
/// @param [in] m
/// @param [out] posB
void rotateMatrix2Pos( const RobotPos &posA, const Eigen::Matrix4d &m, RobotPos &posB );

void laser2Base(double TCP[], const RobotPos &Pos, RobotPos &outPos,int mode = 0) ;

/// p 抽象pos
/// tool 夹具相对于变位机的坐标变换
/// t2b 变位机相对于base的变换状态
void posInBase(RobotPos & p, RobotPos tool, RobotPos t2b);
/// p base下的pos
/// tool 夹具相对于变位机的坐标变换
/// t2b 变位机相对于base的变换状态
void posInTool(RobotPos & p, RobotPos tool, RobotPos t2b);

/// 将一串点往一个法平面上投影
void shadow(point3d<double> pabc, vector<RobotPos> & points);

/// cur 实际当前方向
/// keelCur 龙骨当前方向
/// keelNext 龙骨下一方向
/// 返回 实际下一方向
point3dd getKeelDir(point3dd cur, point3dd keelCur, point3dd keelNext);

/// s 回撤pos
/// len 回撤距离
void retractMove(RobotPos & s,int dz,int dy=0,int dx=0);

/// 返回两串点形成的曲线的空间距离
bool checkPath(vector<RobotPos> target, vector<RobotPos> bme,double & sumlen);

/// lr 地轨位置
/// angle1 翻转角度
/// angle2 旋转角度
/// 变位机当前状态下相对于base的坐标变换参数
RobotPos getPosition(MOTORDEGRE degrees);

/// /// lrLen 地轨位置
/// angle1 翻转角度
/// angle2 旋转角度
/// tool 夹具相对于变位机的变换参数
/// path base下的轨迹
/// keel 夹具坐标系下的抽象龙骨序列
void mergePath(vector<RobotPos> & path,int tracing,vector<RobotPos> keel);

/// 判定Pos_set内的点机器人是否可达
bool Pos2Joint2(std::vector<RobotPos> Pos_set);
#endif // COORD_H
