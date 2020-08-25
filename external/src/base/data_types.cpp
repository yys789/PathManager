#include "base.h"
#include "base/data_types.h"
#include "base/config.h"
#include "base/wzerror.h"
#include "tools/matrix.h"
#include <iostream>
using namespace Eigen;


/*********************************************Begin TypeDef**************************************************/
std::chrono::milliseconds operator"" _ms( unsigned long long t )
{
   return std::chrono::milliseconds(t) ;
}
/*********************************************End TypeDef****************************************************/

/*********************************************Begin RobotPos*************************************************/
bool operator==(const RobotPos &p1, const RobotPos &p2)
{
   // 机器人的坐标会有0.01的抖动
    return (lessequald(std::abs(p1.x-p2.x), 0.01) &&
          lessequald(std::abs(p1.y-p2.y), 0.01) &&
          lessequald(std::abs(p1.z-p2.z), 0.01) &&
          lessequald(std::abs(p1.a-p2.a), 0.01) &&
          lessequald(std::abs(p1.b-p2.b), 0.01) &&
          lessequald(std::abs(p1.c-p2.c), 0.01)) ;
}

bool operator!=(const RobotPos &p1, const RobotPos &p2)
{
    return !(p1 == p2) ;
}

RobotPos operator !(const RobotPos &B)
{
    RobotPos A;
    Matrix4d mb;
    pos2Matrix(B,mb,zyz);
    Matrix4d mc = mb.inverse();
    Matrix2pos(mc,A,zyz);
    return A;
}

RobotPos operator+(const RobotPos &A, const RobotPos &B)
{
    RobotPos C = A;
    C.x += B.x;
    C.y += B.y;
    C.z += B.z;
    C.a += B.a;
    C.b += B.b;
    C.c += B.c;
    return C;
}

RobotPos operator-(const RobotPos &A, const RobotPos &B)
{
    RobotPos C = A;
    C.x -= B.x;
    C.y -= B.y;
    C.z -= B.z;
    C.a -= B.a;
    C.b -= B.b;
    C.c -= B.c;
    return C;
}

RobotPos operator*(const RobotPos &A, const double &b)
{
    RobotPos C = A;
    C.x *= b;
    C.y *= b;
    C.z *= b;
    C.a *= b;
    C.b *= b;
    C.c *= b;
    return C;
}

RobotPos operator/(const RobotPos &A, const double &b)
{
    RobotPos C = A;
    double b1 =abs(b)<0.00001?0.00001:b;
    C.x /= b1;
    C.y /= b1;
    C.z /= b1;
    C.a /= b1;
    C.b /= b1;
    C.c /= b1;
    return C;
}

RobotPos operator<<(const RobotPos &A, const RobotPos &B)
{
    Matrix4d ma;
    pos2Matrix(A,ma,zyz);
    Matrix4d mb;
    pos2Matrix(B,mb,zyz);
    Matrix4d mc = ma*mb;
    RobotPos C = A;
    Matrix2pos(mc,C,zyz);
    return C;
}

RobotPos operator>>(const RobotPos &A, const RobotPos &B)
{
    Matrix4d ma;
    pos2Matrix(A,ma,zyz);
    Matrix4d mb;
    pos2Matrix(B,mb,zyz);
    Matrix4d mc = mb*ma;
    RobotPos C = A;
    Matrix2pos(mc,C,zyz);
    return C;
}

RobotPos operator>(const RobotPos &p1, const RobotTCP & tcp)
{
    RobotPos posInFlange{} ;
    static Config config("etc/calibration.info") ;
    RobotPos rt;
    switch ( tcp ) {
    case RobotTCP::CAMERA:
        posInFlange = config.get<RobotPos>("TINC");
        rt = p1<<posInFlange;
        break ;
    case RobotTCP::UPLOAD:
        rt = p1;
        break ;
    case RobotTCP::FLANGE:
        posInFlange = config.get<RobotPos>("TINF");
        rt = p1<<posInFlange;
        break ;
    case RobotTCP::CAMERA1:
        posInFlange = config.get<RobotPos>("TINC1");
        rt = p1<<posInFlange;
        break ;
    }
    return rt;
}

RobotPos operator>(const RobotTCP & tcp, const RobotPos &p1)
{
    RobotPos posInFlange{} ;
    static Config config("etc/calibration.info") ;
    RobotPos rt = p1;
    switch ( tcp ) {
    case RobotTCP::CAMERA:
        posInFlange = config.get<RobotPos>("TINC");
        rt = p1<<!posInFlange;
        break ;
    case RobotTCP::UPLOAD:
        rt = p1;
        break ;
    case RobotTCP::FLANGE:
        posInFlange = config.get<RobotPos>("TINF");
        rt = p1<<!posInFlange;
        break ;
    case RobotTCP::CAMERA1:
        posInFlange = config.get<RobotPos>("TINC1");
        rt = p1<<!posInFlange;
        break ;
    }
    return rt;
}

/// 支持输入流
/// @param [in] is 输入流
/// @param [out] p 机器人位置
/// @return 输入流的引用
template <typename istream>
istream &operator>>(istream &is, RobotPos &p)
{
    int nTcp ;
    is >> p.x >> p.y >> p.z >> p.a >> p.b >> p.c >> nTcp ;
    p.tcp = static_cast<RobotTCP>(nTcp) ;
    return is ;
}

void file2Path(const std::string &fileName, std::vector<RobotPos> &pos_vec)
{
    std::ifstream ifs(fileName, std::ios::binary ) ;
    RobotPos pos{} ;
    while ( ifs >> pos ) {
        pos_vec.push_back(pos);
    }
    ifs.close() ;
}

RobotPos getPosInFlange( RobotTCP tcp )
{
   RobotPos posInFlange{} ;
   static Config config("etc/calibration.info") ;
   switch ( tcp ) {
   case RobotTCP::CAMERA:
       posInFlange = config.get<RobotPos>("TINC");
       break ;
   case RobotTCP::UPLOAD:
       break ;
   case RobotTCP::FLANGE:
       posInFlange = config.get<RobotPos>("TINF");
       break ;
   case RobotTCP::CAMERA1:
       posInFlange = config.get<RobotPos>("TINC1");
       break ;
   }
   return posInFlange ;
}

RobotAxle RobotAxle::instance(string str)
{
    RobotAxle p = {0,0,0,0,0,0};
    char split[3] = {',',' ','\t'};
    vector<double> ds;
    for(int i=0;i<3;i++)
    {
        ds.resize(0);
        ds.clear();
        stringstream ss(str);
        while(ss.good())
        {
            string substr;
            getline( ss, substr,split[i]);
            stringstream sd(substr);
            double di;
            sd>>di;
            ds.push_back(di);
        }
        if(ds.size() >= 6)
            break;
    }
    if(ds.size() < 6)
        return p;
    p.a1 = ds[0];
    p.a2 = ds[1];
    p.a3 = ds[2];
    p.a4 = ds[3];
    p.a5 = ds[4];
    p.a6 = ds[5];
    return p;
}

RobotPos RobotPos::instance(string str)
{
    RobotPos p = {0,0,0,0,0,0,UPLOAD,0,0,0,1,0,0,0,0,0,0,0,0,0};
    char split[3] = {',',' ','\t'};
    vector<double> ds;
    for(int i=0;i<3;i++)
    {
        ds.resize(0);
        ds.clear();
        stringstream ss(str);
        while(ss.good())
        {
            string substr;
            getline( ss, substr,split[i]);
            stringstream sd(substr);
            double di;
            sd>>di;
            ds.push_back(di);
        }
        if(ds.size() >= 6)
            break;
    }
    if(ds.size() < 6)
        return p;
    p.x = ds[0];
    p.y = ds[1];
    p.z = ds[2];
    p.a = ds[3];
    p.b = ds[4];
    p.c = ds[5];
    p.v = 0;
    p.acc = 80;
    return p;
}

double RobotPos::flux()  /// 通量
{
    point3dd axis_x = {x_x,x_y,x_z};
    point3dd axis_y = {y_x,y_y,y_z};
    point3dd axis_z = {z_x,z_y,z_z};
    if(axis_x.norm() > 0.01 && axis_y.norm() > 0.01 && axis_z.norm() > 0.01)
    {
        return abs(axis_x.cross(axis_y).inner(axis_z));
    }
    else
    {
        return 0;
    }
}

double RobotPos::dis(RobotPos p) /// 间距
{
    return sqrt(pow(x-p.x,2)+pow(y-p.y,2)+pow(z-p.z,2));
}

std::string RobotPos::toStr(){
         char buf[64];
         ::sprintf(buf,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",x,y,z,a,b,c);
         return std::string(buf);
}
/*********************************************begin RobotPos*************************************************/
bool operator==(const RobotAxle &p1, const RobotAxle &p2 )
{
   // 机器人的坐标会有0.01的抖动
   return (lessequald(std::abs(p1.a1-p2.a1),0.01) &&
         lessequald(std::abs(p1.a2-p2.a2), 0.01) &&
         lessequald(std::abs(p1.a3-p2.a3), 0.01) &&
         lessequald(std::abs(p1.a4-p2.a4), 0.01) &&
         lessequald(std::abs(p1.a5-p2.a5), 0.01) &&
         lessequald(std::abs(p1.a6-p2.a6), 0.01)) ;
}

bool operator!=(const RobotAxle &p1, const RobotAxle &p2 )
{
   return !(p1 == p2) ;
}

const RobotAxle operator-(const RobotAxle &p1, const RobotAxle &p2)
{
   return {p1.a1 - p2.a1,
           p1.a2 - p2.a2,
           p1.a3 - p2.a3,
           p1.a4 - p2.a4,
           p1.a5 - p2.a5,
           p1.a6 - p2.a6} ;
}

const RobotAxle operator+(const RobotAxle &p1, const RobotAxle &p2)
{
   return {p1.a1 + p2.a1,
           p1.a2 + p2.a2,
           p1.a3 + p2.a3,
           p1.a4 + p2.a4,
           p1.a5 + p2.a5,
           p1.a6 + p2.a6} ;
}

const RobotAxle operator*(const RobotAxle &p, double percent)
{
   return {p.a1 * percent,
           p.a2 * percent,
           p.a3 * percent,
           p.a4 * percent,
           p.a5 * percent,
           p.a6 * percent} ;
}

RobotAxle &operator+=(RobotAxle &p1, const RobotAxle &p2)
{
   return p1 = p1 + p2 ;
}

RobotAxle &operator-=(RobotAxle &p1, const RobotAxle &p2)
{
   return p1 = p1 - p2 ;
}

/*********************************************End RobotPos*************************************************/

/*********************************************Begin UV*************************************************/
bool operator==( const UV &uv1, const UV &uv2 )
{
   return equald(uv1.u, uv2.u) && equald( uv1.v, uv2.v ) ;
}

UV operator+( const UV &uv1, const UV &uv2 )
{
   return {uv1.u + uv2.u, uv1.v + uv2.v} ;
}

UV operator-( const UV &uv1, const UV &uv2 )
{
   return {uv1.u - uv2.u, uv1.v - uv2.v} ;
}

UV operator/( const UV &uv, double d )
{
   return {uv.u / d, uv.v / d} ;
}

double lenLine( const UV &uv1, const UV &uv2 )
{
   UV diffUV = uv2 - uv1 ;
   return sqrt(pow(diffUV.u, 2) + pow(diffUV.v, 2)) ;
}
/*********************************************End UV*************************************************/
