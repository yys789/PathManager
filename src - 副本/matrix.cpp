#include "include/matrix.h"
#include <type_traits>

using namespace Eigen;
using namespace std;
using namespace walgo;

void pos2Matrix(const RobotPos &pos, Eigen::Matrix4d &m, EULERTYPE eulerType)
{
    using namespace Eigen;

    /*Matrix3d m3 ;
    pos2Matrix( pos, m3, eulerType ) ;
    for ( long i = 0; i < m.rows(); ++i ) {
       for ( long j = 0; j < m.cols(); ++j ) {
          m(i, j) = 0 ;
       }
    }
    m(3, 3) = 1 ;
    for ( long i = 0; i < m3.rows(); ++i ) {
       for ( long j = 0; j < m3.cols(); ++j ) {
          m(i, j) = m3(i, j) ;
       }
    }
    m(0, 3) = pos.x ;
    m(1, 3) = pos.y ;
    m(2, 3) = pos.z ;
    return ;*/
    double a = pos.a * M_PI / 180 ;
    double b = pos.b * M_PI / 180 ;
    double c = pos.c * M_PI / 180 ;
    if ( eulerType == zyx )
    {
       Matrix4d rx, ry, rz;
       rx << 1, 0, 0, 0,
             0, cos(c), -sin(c), 0,
             0, sin(c), cos(c), 0,
             0, 0, 0, 1;

       ry << cos(b), 0, sin(b), 0,
             0, 1, 0, 0,
             -sin(b), 0, cos(b), 0,
             0, 0, 0, 1;

       rz << cos(a), -sin(a), 0, 0,
             sin(a), cos(a), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
       m = rz*ry*rx;
    }
    else if ( eulerType == zyz)
    {
       Matrix4d rz1, ry, rz2;
       rz1 << cos(a), -sin(a), 0, 0,
              sin(a), cos(a), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

       ry  << cos(b), 0, sin(b), 0,
             0, 1, 0, 0,
             -sin(b), 0, cos(b), 0,
             0, 0, 0, 1;

       rz2 << cos(c), -sin(c), 0, 0,
             sin(c), cos(c), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
       m = rz1*ry*rz2;
    }

    m(0, 3) = pos.x ;
    m(1, 3) = pos.y ;
    m(2, 3) = pos.z ;
}
#if 0
std::vector<int> getAxisdIndex( EULERTYPE eulerType )
{
   using namespace Eigen;
   std::string strEular ;
   std::vector<int> axisd_arr ;
   switch( eulerType ) {
   case EULERTYPE::zyx:
      strEular = "zyx" ;
      break;
   case EULERTYPE::zyz:
      strEular = "zyz" ;
      break;
   case EULERTYPE::zxz:
      strEular = "zxz" ;
      break;
   }
   for ( size_t i = 0; i < strEular.size(); ++i ) {
      switch ( strEular[i] ) {
      case 'X':
         axisd_arr.push_back(0);
         break ;
      case 'Y':
         axisd_arr.push_back(1) ;
         break ;
      case 'Z':
         axisd_arr.push_back(2) ;
         break ;
      }
   }
   return axisd_arr ;
}

std::vector<Eigen::Vector3d::BasisReturnType> getAxisd( EULERTYPE eularType )
{
   using namespace Eigen;
   std::string strEular ;
   std::vector<Eigen::Vector3d::BasisReturnType> axisd_arr ;
   switch( eularType ) {
   case EULERTYPE::zyx:
      strEular = "zyx" ;
      break;
   case EULERTYPE::zyz:
      strEular = "zyz" ;
      break;
   case EULERTYPE::zxz:
      strEular = "zxz" ;
      break;
   }
   for ( size_t i = 0; i < strEular.size(); ++i ) {
      switch ( strEular[i] ) {
      case 'X':
         axisd_arr.push_back(Vector3d::UnitX()) ;
         break ;
      case 'Y':
         axisd_arr.push_back(Vector3d::UnitY()) ;
         break ;
      case 'Z':
         axisd_arr.push_back(Vector3d::UnitZ()) ;
         break ;
      }
   }
   return axisd_arr ;
}
#endif

void pos2Matrix(const RobotPos &pos, Eigen::Matrix3d &m, EULERTYPE eulerType)
{
    using namespace Eigen;

    double a = pos.a * M_PI / 180 ;
    double b = pos.b * M_PI / 180 ;
    double c = pos.c * M_PI / 180 ;
    if ( eulerType == zyx)
    {
        Matrix3d rx, ry, rz;
        rx << 1, 0, 0,
              0, cos(c), -sin(c),
              0, sin(c), cos(c);

        ry << cos(b), 0, sin(b),
              0, 1, 0,
             -sin(b), 0, cos(b);

        rz << cos(a), -sin(a), 0,
              sin(a), cos(a), 0,
              0, 0, 1;
        m = rz*ry*rx;
    }
    else if ( eulerType == zyz)
    {
       Matrix3d rz1, ry, rz2;
       rz1 << cos(a), -sin(a), 0,
              sin(a), cos(a), 0,
              0, 0, 1;

       ry  << cos(b), 0, sin(b),
              0, 1, 0,
              -sin(b), 0, cos(b);


       rz2 << cos(c), -sin(c), 0,
              sin(c), cos(c), 0,
              0, 0, 1;
       m = rz1*ry*rz2;
    }

}

void Matrix2pos(Eigen::Matrix4d &m,  RobotPos &pos, EULERTYPE eulerType)
{
    using namespace Eigen;
    /*Eigen::Matrix3d m3 ;
    for ( int i = 0; i < 3; ++i ) {
       for ( int j = 0; j < 3; ++j ) {
          m3(i, j) = m(i, j) ;
       }
    }
    auto euler_arr = getAxisdIndex(eulerType) ;
    auto v = m3.eulerAngles( euler_arr[0], euler_arr[1], euler_arr[2]) ;
    pos.x = m(0, 3) ;
    pos.y = m(1, 3) ;
    pos.z = m(2, 3) ;
    pos.a = v(0) / M_PI * 180 ;
    pos.b = v(1) / M_PI * 180 ;
    pos.c = v(2) / M_PI * 180 ;
    return ;*/
    double a, b, c;
    if ( eulerType == zyx)
    {
        b = asin(-m(2, 0));
        if (abs(m(2, 0)) != 1)
        {
           c = atan2(m(2, 1) / cos(b), m(2, 2) / cos(b));
           a = atan2(m(1, 0) / cos(b), m(0, 0) / cos(b));
        }
       else
       {
           a = 0;
           if (b < 0)
              c = a + atan2(m(0, 1), m(0, 2));
           else
              c = -a + atan2(-m(0, 1), -m(0, 2));
       }
       pos.x = m(0, 3);
       pos.y = m(1, 3);
       pos.z = m(2, 3);
       pos.a = a * 180 / M_PI;
       pos.b = b * 180 / M_PI;
       pos.c = c * 180 / M_PI;
    }
    else if ( eulerType == zyz)
    {

       b = acos(m(2, 2));
       a = atan2(m(1,2),m(0,2));
       c = atan2((-1*m(0,0)*sin(a)+m(1,0)*cos(a)),(m(1,1)*cos(a)-m(0,1)*sin(a)));//Ax-Ay/-Ox+Oy

       pos.x = m(0, 3);
       pos.y = m(1, 3);
       pos.z = m(2, 3);
       pos.a = a * 180 / M_PI;
       pos.b = b * 180 / M_PI;
       pos.c = c * 180 / M_PI;
    }
}

void Matrix2pos(Eigen::Matrix3d &m,  RobotPos &pos, EULERTYPE eulerType)
{
    using namespace Eigen;
    /*auto index_arr = getAxisdIndex(eulerType) ;
    Vector3d v = m.eulerAngles( index_arr[0], index_arr[1], index_arr[2] ) ;
    pos.a = v(0) / M_PI * 180 ;
    pos.b = v(1) / M_PI * 180 ;
    pos.c = v(2) / M_PI * 180 ;
    return ;*/
    double a, b, c;
    if ( eulerType == zyx)
    {
       b = asin(-m(2, 0));
       if (abs(m(2, 0)) != 1)
       {
          c = atan2(m(2, 1) / cos(b), m(2, 2) / cos(b));
          a = atan2(m(1, 0) / cos(b), m(0, 0) / cos(b));
       }
       else
       {
          a = 0;
          if (b < 0)
              c = a + atan2(m(0, 1), m(0, 2));
          else
              c = -a + atan2(-m(0, 1), -m(0, 2));
       }

       pos.a = a * 180 / M_PI;
       pos.b = b * 180 / M_PI;
       pos.c = c * 180 / M_PI;
    }
    else if ( eulerType == zyz)
    {

       b = acos(m(2, 2));
       a = atan2(m(1,2),m(0,2));
       c = atan2((-1*m(0,0)*sin(a)+m(1,0)*cos(a)),(m(1,1)*cos(a)-m(0,1)*sin(a)));//Ax-Ay/-Ox+Oy

       pos.a = a * 180 / M_PI;
       pos.b = b * 180 / M_PI;
       pos.c = c * 180 / M_PI;
    }
}
//重载float类型的转换
void pos2Matrix(const RobotPos &pos, Eigen::Matrix4f &m, EULERTYPE eulerType)
{
    using namespace Eigen;
    /*Matrix3d m3 ;
    pos2Matrix( pos, m3, eulerType ) ;
    for ( long i = 0; i < m.rows(); ++i ) {
       for ( long j = 0; j < m.cols(); ++j ) {
          m(i, j) = 0 ;
       }
    }
    m(3, 3) = 1 ;
    for ( long i = 0; i < m3.rows(); ++i ) {
       for ( long j = 0; j < m3.cols(); ++j ) {
          m(i, j) = m3(i, j) ;
       }
    }
    m(0, 3) = pos.x ;
    m(1, 3) = pos.y ;
    m(2, 3) = pos.z ;
    return ;*/
    float a = pos.a * M_PI / 180 ;
    float b = pos.b * M_PI / 180 ;
    float c = pos.c * M_PI / 180 ;
    if ( eulerType == zyx)
    {
        Matrix4f rx, ry, rz;
        rx << 1, 0, 0, 0,
              0, cos(c), -sin(c), 0,
              0, sin(c), cos(c), 0,
              0, 0, 0, 1;

        ry << cos(b), 0, sin(b), 0,
              0, 1, 0, 0,
             -sin(b), 0, cos(b), 0,
              0, 0, 0, 1;
        rz << cos(a), -sin(a), 0, 0,
              sin(a), cos(a), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

        m = rz*ry*rx;
    }
    else if ( eulerType == zyz)
    {
       Matrix4f rz1, ry, rz2;
       rz1 << cos(a), -sin(a), 0, 0,
              sin(a), cos(a), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

       ry  << cos(b), 0, sin(b), 0,
             0, 1, 0, 0,
             -sin(b), 0, cos(b), 0,
             0, 0, 0, 1;

       rz2 << cos(c), -sin(c), 0, 0,
             sin(c), cos(c), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
       m = rz1*ry*rz2;
    }

    m(0, 3) = pos.x ;
    m(1, 3) = pos.y ;
    m(2, 3) = pos.z ;

}

void pos2Matrix(const RobotPos &pos, Eigen::Matrix3f &m, EULERTYPE eulerType)
{
    using namespace Eigen;
    /*auto axisd_arr = getAxisd(eulerType) ;
   Matrix3d md ;
   md = AngleAxisd(pos.a/180*M_PI, axisd_arr[0]) *
                AngleAxisd(pos.b/180*M_PI, axisd_arr[1]) *
                AngleAxisd(pos.c/180*M_PI, axisd_arr[2]) ;
   for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) {
         m(i, j) = md(i, j) ;
      }
   }
   return ;*/
    float a = pos.a * M_PI / 180 ;
    float b = pos.b * M_PI / 180 ;
    float c = pos.c * M_PI / 180 ;
    if ( eulerType == zyx)
    {
        Matrix3f rx, ry, rz;
        rx << 1, 0, 0,
              0, cos(c), -sin(c),
              0, sin(c), cos(c);

        ry << cos(b), 0, sin(b),
              0, 1, 0,
             -sin(b), 0, cos(b);

        rz << cos(a), -sin(a), 0,
              sin(a), cos(a), 0,
              0, 0, 1;

        m = rz*ry*rx;
    }
    else if ( eulerType == zyz)
    {
       Matrix3f rz1, ry, rz2;
       rz1 << cos(a), -sin(a), 0,
              sin(a), cos(a), 0,
              0, 0, 1;


       ry  << cos(b), 0, sin(b),
             0, 1, 0,
             -sin(b), 0, cos(b);

       rz2 << cos(c), -sin(c), 0,
             sin(c), cos(c), 0,
             0, 0, 1;

       m = rz1*ry*rz2;
    }

}

void Matrix2pos(Eigen::Matrix4f &m,  RobotPos &pos, EULERTYPE eulerType)
{
    using namespace Eigen;
    /*Eigen::Matrix3d m3 ;
    for ( int i = 0; i < 3; ++i ) {
       for ( int j = 0; j < 3; ++j ) {
          m3(i, j) = m(i, j) ;
       }
    }
    auto euler_arr = getAxisdIndex(eulerType) ;
    auto v = m3.eulerAngles( euler_arr[0], euler_arr[1], euler_arr[2]) ;
    pos.x = m(0, 3) ;
    pos.y = m(1, 3) ;
    pos.z = m(2, 3) ;
    pos.a = v(0) / M_PI * 180 ;
    pos.b = v(1) / M_PI * 180 ;
    pos.c = v(2) / M_PI * 180 ;
    return ;*/
    float a, b, c;
    if ( eulerType == zyx)
    {
         b = asin(-m(2, 0));
         if (abs(m(2, 0)) != 1)
         {
            c = atan2(m(2, 1) / cos(b), m(2, 2) / cos(b));
            a = atan2(m(1, 0) / cos(b), m(0, 0) / cos(b));
         }
         else
         {
            a = 0;
            if (b < 0)
               c = a + atan2(m(0, 1), m(0, 2));
            else
               c = -a + atan2(-m(0, 1), -m(0, 2));
         }
         pos.x = m(0, 3);
         pos.y = m(1, 3);
         pos.z = m(2, 3);
         pos.a = a * 180 / M_PI;
         pos.b = b * 180 / M_PI;
         pos.c = c * 180 / M_PI;
    }
    else if ( eulerType == zyz)
    {
       b = acos(m(2, 2));
       a = atan2(m(1,2),m(0,2));
       c = atan2((-1*m(0,0)*sin(a)+m(1,0)*cos(a)),(m(1,1)*cos(a)-m(0,1)*sin(a)));//Ax-Ay/-Ox+Oy

       pos.x = m(0, 3);
       pos.y = m(1, 3);
       pos.z = m(2, 3);
       pos.a = a * 180 / M_PI;
       pos.b = b * 180 / M_PI;
       pos.c = c * 180 / M_PI;
    }
}

void Matrix2pos(Eigen::Matrix3f &m,  RobotPos &pos, EULERTYPE eulerType)
{
    using namespace Eigen;
    /*auto euler_arr = getAxisdIndex(eulerType) ;
    auto v = m.eulerAngles( euler_arr[0], euler_arr[1], euler_arr[2]) ;
    pos.a = v(0) / M_PI * 180 ;
    pos.b = v(1) / M_PI * 180 ;
    pos.c = v(2) / M_PI * 180 ;
    return ;*/

    float a, b, c;
    if ( eulerType == zyx)
    {
        b = asin(-m(2, 0));
        if (abs(m(2, 0)) != 1)
        {
            c = atan2(m(2, 1) / cos(b), m(2, 2) / cos(b));
            a = atan2(m(1, 0) / cos(b), m(0, 0) / cos(b));
        }
        else
        {
            a = 0;
            if (b < 0)
                c = a + atan2(m(0, 1), m(0, 2));
            else
                c = -a + atan2(-m(0, 1), -m(0, 2));
        }
        pos.a = a * 180 / M_PI;
        pos.b = b * 180 / M_PI;
        pos.c = c * 180 / M_PI;
    }
    else if ( eulerType == zyz)
    {
       b = acos(m(2, 2));
       a = atan2(m(1,2),m(0,2));
       c = atan2((-1*m(0,0)*sin(a)+m(1,0)*cos(a)),(m(1,1)*cos(a)-m(0,1)*sin(a)));//Ax-Ay/-Ox+Oy

       pos.a = a * 180 / M_PI;
       pos.b = b * 180 / M_PI;
       pos.c = c * 180 / M_PI;
    }
}

void posConversion( const RobotPos &pos1InFlange,  const RobotPos &pos1, const RobotPos &pos2InFlange, RobotPos &pos2 )
{
   using namespace Eigen ;
   Config sys_config ;
   Matrix3d Rb1, RKCS, Rb2, Rtemp, R2;
   pos2Matrix(pos1InFlange, Rtemp, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   pos2Matrix(pos1, Rb1, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   RKCS = Rb1*Rtemp.inverse();

   Vector3d Xb1(pos1.x, pos1.y, pos1.z), Xt1(pos1InFlange.x, pos1InFlange.y, pos1InFlange.z), TKCS;
   TKCS = Xb1 - RKCS*Xt1;

   Vector3d Xb2, Xt2(pos2InFlange.x, pos2InFlange.y, pos2InFlange.z);
   pos2Matrix(pos2InFlange, R2, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));

   Rb2 = RKCS*R2;
   Xb2 = RKCS*Xt2 + TKCS;

   Matrix2pos(Rb2, pos2, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   pos2.x = Xb2(0);
   pos2.y = Xb2(1);
   pos2.z = Xb2(2);
}

/// 计算空间任意两个TCP点的平移转换
/// pos1.tcp及pos2.tcp都要写入相应的TCP选点
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [out] pos2 pos2在基坐标中的位置
void abcConversion( const RobotPos &pos1, RobotPos &pos2 )
{
   auto pos = pos1 ;
   posConversion( pos, pos2 ) ;
   pos2.x = pos.x ;
   pos2.y = pos.y ;
   pos2.z = pos.z ;
}

/// 计算空间任意两个TCP点的平移转换
/// @param [in] pos1InFlange pos1和法兰的坐标关系
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [in] pos2InFlange pos2和法兰的坐标关系
/// @param [out] pos2 pos2在基坐标中的位置
void abcConversion( const RobotTCP &tcp1, const RobotPos &pos1, RobotTCP tcp2, RobotPos &pos2 )
{
   auto pos = pos1 ;
   posConversion( tcp1, pos, tcp2, pos2 ) ;
   pos2.x = pos.x ;
   pos2.y = pos.y ;
   pos2.z = pos.z ;
   pos2.tcp = tcp2;
}

/// 计算空间任意两个TCP点的平移转换
/// pos1.tcp要写入相应的TCP选点
/// @param [in] pos1 pos1在基坐标中的位置
/// @param [out] pos2 pos2在基坐标中的位置
void abcConversion( const RobotPos &pos1, RobotTCP tcp2, RobotPos &pos2 )
{
   auto pos = pos1 ;
   posConversion( pos, tcp2, pos2 ) ;
   pos2.x = pos.x ;
   pos2.y = pos.y ;
   pos2.z = pos.z ;
}

void posConversion( const RobotTCP &tcp1, const RobotPos &pos1, RobotTCP tcp2, RobotPos &pos2 )
{
   auto pos1InFlange = getPosInFlange(tcp1) ;
   auto pos2InFlange = getPosInFlange(tcp2) ;
   auto pos = pos1 ;
   pos2.tcp = tcp2 ;
   posConversion( pos1InFlange, pos, pos2InFlange, pos2) ;
}

void posConversion( const RobotPos &pos1, RobotTCP tcp2, RobotPos &pos2 )
{
   auto pos1InFlange = getPosInFlange(pos1.tcp) ;
   auto pos2InFlange = getPosInFlange(tcp2) ;
   auto pos = pos1 ;
   pos2.tcp = tcp2 ;
   posConversion( pos1InFlange, pos, pos2InFlange, pos2) ;
}

void posConversion( const RobotPos &pos1, RobotPos &pos2 )
{
   auto pos1InFlange = getPosInFlange(pos1.tcp) ;
   auto pos2InFlange = getPosInFlange(pos2.tcp) ;
   auto pos = pos1 ;
   posConversion( pos1InFlange, pos, pos2InFlange, pos2) ;
}

/// points 3d位姿点云
/// abs 这些点拟合的平面方程系数a*x+b*y+c*z=1（也是法向量）
/// 返回 拟合成功、失败
bool points2face(vector<RobotPos> &points, Vector3d &abc)
{
    vector<RobotPos> noZero;
    RobotPos zero = RobotPos::instance();
    for(auto p : points)
    {
        if(p.dis(zero) > 0.1)
            noZero.push_back(p);
    }
    int fConts = noZero.size();
    if(fConts == 0)
        return false;
    MatrixXd f(fConts, 3);
    int row = 0;
    for ( auto & fpoint : noZero ) {
        f(row,0) = fpoint.x;
        f(row,1) = fpoint.y;
        f(row,2) = fpoint.z;
        row ++;
    }
    MatrixXd ft(3,fConts);
    ft = f.transpose();
    Matrix3d ftf = ft*f;
    FullPivLU<Matrix3d> fpu(ftf);
    if(fpu.rank() < 3)
        return false;
    Matrix3d ftfi = ftf.inverse();
    MatrixXd fp(fConts,1);
    fp.setOnes(fConts,1);
    abc = ftfi*ft*fp;
    return true;
}
