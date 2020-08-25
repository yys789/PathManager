#include "tools/robot_move.h"
#include "base/data_types.h"
#include "base.h"

#include <iostream>

using namespace  walgo;

// 重置机器人移动的角度
void resetAngle( const double &angle1, double &angle2 )
{
    double dAngle = std::abs(angle2 - angle1) ;
    if ( dAngle > 180 ) {
        if ( angle1 < 0 ) angle2 = angle1 - ( 360 - dAngle ) ;
        if ( angle1 > 0 ) angle2 = angle1 + ( 360 - dAngle ) ;
    }
}

// 重置机器人移动的位置
void resetPos( const double &pos1, double &pos2 )
{
    double dPos = std::abs(pos1 - pos2) ;

    if ( equald(dPos, 0.01) ) {
        pos2 = pos1 ;
    }
}

// 生成带有加速，减速的路徑
// 因为欧拉角的旋转需要基于绝对路徑，所以用绝对路徑的方式实现
void absolute2Eular(RobotPos beginPos, RobotPos endPos,
             double maxSpeed, double acc,
             const double maxAngleSpeed, const double angleAcc,
             std::queue<RobotPos> &path)
{
   //if ( beginPos == endPos ) return ;
    // 机器人会抖动，角度是以0和180为原点, 范围是-180~+180
    //resetAngle( beginPos.a, endPos.a ) ;
    //resetAngle( beginPos.b, endPos.b ) ;
    //resetAngle( beginPos.c, endPos.c ) ;

    double lengthX = endPos.x - beginPos.x ;
    double lengthY = endPos.y - beginPos.y ;
    double lengthZ = endPos.z - beginPos.z ;

    std::vector<RobotPos> path_vec ;
    double angleAll = 0 ;
    double lengthAll = sqrt(pow(lengthX, 2) + pow(lengthY, 2) + pow(lengthZ, 2)) ;
    double currSpeed = 0 ;
    // 当前欧拉角百分比
    double percentDone = 0, percentSlowDown, percent ;
    double useLength = 0, useAcc = 0, useSpeed = 0 ;

    // 相对修正
    Rotation startR( beginPos.a, beginPos.b, beginPos.c,
                     false, EULERTYPE::zyx, true ) ;
    Rotation endR( endPos.a, endPos.b, endPos.c,
                   false, EULERTYPE::zyx, true ) ;
    double na = 0 ,
           nb = 0 ,
           nc = 0 ;

    // 计算欧拉角一共要转多少度
    auto s2e = endR.times(startR.inverse()) ;
    s2e.getAngleAxis( angleAll, na, nb, nc ) ;
    // 认为线速度和角速度对等关系为: speed ~ angleSpeed * 10
    if ( equald(maxSpeed, 0) &&
         equald(maxAngleSpeed, 0)) {
       double a, b, c ;
       Rotation dR( angleAll, na, nb, nc ) ;
       dR.getEulerAngles( a, b, c,
                          true, EULERTYPE::zyx, true ) ;
       path.push({lengthX, lengthY, lengthZ, a, b, c}) ;
       return ;
    }
    else {
       if ( greaterd(lengthAll / maxSpeed, angleAll / M_PI * 180 / (maxAngleSpeed))) {
          useLength = lengthAll ;
          useAcc = acc ;
          useSpeed = maxSpeed ;
       }
       else {
          useLength = angleAll / M_PI * 180;
          useAcc = angleAcc ;
          useSpeed = maxAngleSpeed ;
       }
    }
    std::vector<std::pair<double, RobotPos>> queuePercent ;
    // 加速过程
    RobotPos nextPos, relativePos = endPos - beginPos ;
    while ( currSpeed < useSpeed ) {
        currSpeed += useAcc;
        percent = currSpeed / useLength ;
        nextPos = relativePos * percent ;
        queuePercent.emplace_back(percent, nextPos) ;
        percentDone += percent ;

        if ( greaterequald(currSpeed, useSpeed )) {
            currSpeed = useSpeed ;
            break ;
        }
        if ( greaterequald(percentDone, 0.5) )
            break ;
    }

    // 以speed的速度走完匀速部分
    percentSlowDown = percentDone ;
    percent = currSpeed / useLength ;
    nextPos = relativePos * percent ;
    while ( greaterequald(1-percentDone, percentSlowDown)) {
        queuePercent.emplace_back(percent, nextPos) ;
        percentDone += percent ;
    }
    // 减速过程
    while ( lessd(percentDone, 1)) {
        currSpeed -= useAcc;
        if ( currSpeed <= useAcc ) {
            currSpeed = useAcc ;
        }
        percent = currSpeed / useLength ;
        // 已走路程的百分比
        nextPos = relativePos * percent ;
        queuePercent.emplace_back(percent, nextPos) ;
        percentDone += percent ;
    }
    // 增加角度
    for ( auto &value : queuePercent ) {
       Rotation dR( value.first * angleAll , na, nb, nc ) ;
       dR.getEulerAngles(value.second.a, value.second.b, value.second.c,
                         true, EULERTYPE::zyx, true) ;
    }
    // 精度纠正
    while ( greaterd(percentDone, 1)) {
       percentDone -= queuePercent.back().first ;
       queuePercent.pop_back() ;
    }
    // 相对坐标
    double a, b, c ;
    Rotation dR( (1-percentDone) * angleAll, na, nb, nc ) ;
    dR.getEulerAngles( a, b, c,
                          true, EULERTYPE::zyx, true ) ;

    nextPos = relativePos * (1-percentDone) ;
    nextPos.a = a ;
    nextPos.b = b ;
    nextPos.c = c ;
    if ( endPos != RobotPos{} ) {
       queuePercent.emplace_back(1-percent, nextPos) ;
    }
    for ( auto &p : queuePercent ) {
       path.push(p.second) ;
    }
}

void relative2Eular(const RobotPos &begin, const RobotPos &relativePos,
             double maxSpeed, double acc, const double maxAngleSpeed, const double angleAcc,
             std::queue<RobotPos> &path)
{
    // 生成路徑
    absolute2Eular( begin, begin + relativePos,
             maxSpeed, acc, maxAngleSpeed, angleAcc,
             path ) ;
}

/// 规划机器轴的旋转
/// @param [in] relativeAxle 轴的旋转角度
/// @param [in] maxSpeed 旋转的速度
/// @param [in] acc 旋转的加速度
/// @param [out] path 计算后的旋转路径
void axleRelative( const RobotAxle &relativeAxle, double maxSpeed, double acc, std::queue<RobotAxle> &path )
{
   std::initializer_list<double> l{relativeAxle.a1,
      relativeAxle.a2,
      relativeAxle.a3,
      relativeAxle.a4,
      relativeAxle.a5,
      relativeAxle.a6} ;
   double maxAxle = std::abs(*std::max_element(l.begin(), l.end(), [](double d1, double d2){
         return lessd(std::abs(d1), std::abs(d2));})) ;
   RobotAxle axleDone{}, nextAxle{} ;
   if ( acc > maxSpeed ) acc = maxSpeed ;
   double currSpeed = acc ;
   double percentDone = 0, percentSlowDown, percent ;
   std::vector<std::pair<double, RobotAxle>> queuePercent ;
   // 加速
   while ( lessd(currSpeed, maxSpeed ) ) {
      percent = currSpeed / maxAxle ;
      // 按比例计算每个轴的旋转
      nextAxle = relativeAxle * percent ;
      axleDone += nextAxle ;
      queuePercent.emplace_back( percent, nextAxle ) ;
      percentDone += percent ;
      currSpeed += acc ;
      if ( greaterd(currSpeed, maxSpeed)) {
         currSpeed = maxSpeed ;
      }
   }

   // 匀速
   percentSlowDown = percentDone ;
   percent = currSpeed / maxAxle ;
   while ( greaterd((1-percentDone), percentSlowDown) ) {
      // 按比例计算每个轴的旋转
      nextAxle = relativeAxle * percent ;
      axleDone += nextAxle ;
      percentDone += percent ;
      queuePercent.emplace_back( percent, nextAxle ) ;
   }
   // 减速
   while ( lessd(percentDone, 1) ) {
      currSpeed -= acc ;
      if ( lessd(currSpeed, acc) ) currSpeed = acc ;
      percent = currSpeed / maxAxle ;
      nextAxle = relativeAxle * percent ;
      axleDone += nextAxle ;
      percentDone += percent ;
      queuePercent.emplace_back( percent, nextAxle ) ;
   }
   // 最终修正
   while ( greaterd(percentDone, 1) ) {
      percentDone -= queuePercent.back().first ;
      axleDone -= queuePercent.back().second ;
      queuePercent.pop_back() ;
   }

   for ( auto &p : queuePercent ) {
      path.push(p.second) ;
   }
   if ( relativeAxle != axleDone ) {
      path.push(relativeAxle - axleDone) ;
   }
   for ( auto &p : queuePercent) {
      //std::cout << p.second << '\n' ;
   }
   //std::cout << relativeAxle - axleDone << '\n' ;
   std::cout << "axleDone:" << axleDone.toStr() << "\n" ;
   std::cout << std::flush ;
}

/// 路径拟合
/// @param [in] in_vec 原始的路径
/// @out_vec[out] out_vec 计算后的路径
/// @param [in] multiple 速度缩放倍数
void pathFit( const std::vector<RobotPos> &in_vec, std::vector<RobotPos> &out_vec, double multiple )
{
   if ( in_vec.size() == 0 ) return ;
   vector<point3dd> posIn, eulerIn, posOut, eulerOut ;
   for ( auto &pos : in_vec ) {
      posIn.emplace_back( pos.x, pos.y, pos.z ) ;
      eulerIn.emplace_back( pos.a, pos.b, pos.c ) ;
   }
   int sizeIn = in_vec.size() ;
   sizeIn /= 10 ;
   sizeIn = sizeIn >= 2 ? sizeIn : 2 ;
   interpPathScale(posIn, eulerIn, posOut, eulerOut, sizeIn, sizeIn / multiple, 1);
   auto tcp = in_vec[0].tcp ;
   assert( posOut.size() == eulerOut.size() ) ;
   for ( size_t i = 0; i < posOut.size(); ++i ) {
      out_vec.push_back( {posOut[i]._x, posOut[i]._y, posOut[i]._z, eulerOut[i]._x, eulerOut[i]._y, eulerOut[i]._z, tcp} ) ;
   }
}
