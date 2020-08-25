#ifndef _DATA_TYPE_H
#define _DATA_TYPE_H

#include <string>
#include <array>
#include <vector>
#include <fstream>
#include <chrono>
#include "walgo/point.h"

#include <boost/any.hpp>

#include <opencv2/opencv.hpp>
#include <sstream>

using namespace  walgo;
using namespace  std;

/*********************************************Begin TypeDef**************************************************/
std::chrono::milliseconds operator"" _ms( unsigned long long t ) ;

// 毫秒
typedef std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<double, std::ratio<1, 1000>>> Millsecond ;

// 毫秒之差
typedef std::chrono::duration<double, std::ratio<1, 1000>> MillDiff ;

/*********************************************End TypeDef*****************************************************/


/*********************************************Begin RobootPos*************************************************/
/// 机器人TCP点的选取
/// UPLOAD 机器人通信上传 CAMERA 默认相机中心点 FLANGE 法兰盘 CAMERA1 跟踪相机中心点 TorchTop 焊枪头
enum RobotTCP {UPLOAD = 0, CAMERA, FLANGE, CAMERA1, TorchTop} ;
/// 机器人的状态
enum RobotStatus{RUNNING = 0, STOPPING, STOPPED, ERROR};
/// 机器人错误信息
enum RobotError{NORMAL = 0, FORCESTOPMOVING, SERVICESTOP, FOUNDTARGET, WRONGDIRECTION,MOVETIMEOUT,HOLDING};

/// 位置类型
enum PosType{PICTURE = 0, TRANS, SCANBEGIN, SCANEND, CALCSEAM, POSITIONER} ;

/// 法兰十字滑台坐标
struct XZ
{
   double x;
   double z;
} ;

bool operator==(const XZ &xz1, const XZ &xz2 ) ;
/// 机器人位置,表示机器人的坐标及欧拉角
struct RobotPos
{
   double x, y, z, a, b, c;
   RobotTCP tcp;
   double dh, dw, v, acc;
   bool weld;         /// 断焊功能开关 false 空走 true 焊接
   double x_x;   /// base下左侧方向（x.cross(y) = 前进方向）
   double x_y;   /// base下左侧方向（x.cross(y) = 前进方向）
   double x_z;   /// base下左侧方向（x.cross(y) = 前进方向）
   double y_x;   /// base下右侧方向（x.cross(y) = 前进方向）
   double y_y;   /// base下右侧方向（x.cross(y) = 前进方向）
   double y_z;   /// base下右侧方向（x.cross(y) = 前进方向）
   double z_x;   /// base下前进方向（x.cross(y) = 前进方向）
   double z_y;   /// base下前进方向（x.cross(y) = 前进方向）
   double z_z;   /// base下前进方向（x.cross(y) = 前进方向）
   static RobotPos instance(string str = "");
   double flux();  /// 通量
   double dis(RobotPos p);
   std::string toStr();
};

struct WELDPARAMS
{
    double dl;  // width
    double dr;  // height
    double v;
    double I;
    double U;
    int vh;     // v or h
    double dy;
};

typedef struct weldOpt
{
    double S_W ;
    double I_W ;
    double V_W ;
    double T_END ;
    double I_END ;
    double V_END ;
    double range;
    double frequency;
    double serialNumber;
    double axis_x;
    double axis_y;
} weldInfo;


struct MOTORDEGRE
{
    double angle1;
    double angle2;
    double angle3;
};

struct WELDANGLEINFO
{
    MOTORDEGRE motors;
    MOTORDEGRE motore;
    double motor1speed;
    double motor2speed;
    double motor3speed;
    double angleSpeed1;        /// 变位机转速1(度/秒)
    double angleSpeed2;        /// 变位机转速2(度/秒)
    double angleSpeed3;        /// 变位机转速3(度/秒)
    double rotateTime;         /// 開始转动时间
};


RobotPos operator+(const RobotPos &A, const RobotPos &B);
RobotPos operator-(const RobotPos &A, const RobotPos &B);
RobotPos operator*(const RobotPos &A, const double &b);
RobotPos operator/(const RobotPos &A, const double &b);
/// 判断两个pos是否相同
bool operator==(const RobotPos &p1, const RobotPos &p2 ) ;
/// 判断两个pos是否不同
bool operator!=(const RobotPos &p1, const RobotPos &p2 ) ;
/// pos B取反
RobotPos operator !(const RobotPos &B);
/// A*B
RobotPos operator<<(const RobotPos &A, const RobotPos &B);
/// B*A
RobotPos operator>>(const RobotPos &A, const RobotPos &B);
/// tcp置于p1时，计算机器人的位置
RobotPos operator>(const RobotPos &p1, const RobotTCP & tcp);
/// 上述运算的逆
RobotPos operator>(const RobotTCP & tcp, const RobotPos &p1);
/// 从文件中读取路徑保存到内存中
/// @param [in] fileName 文件名
/// @param [out] pos_vec 容器
void file2Path( const std::string &fileName, std::vector<RobotPos> &pos_vec ) ;

struct RobotAxle
{
    double a1, a2, a3, a4, a5, a6 ;
    std::string toStr(){
        char buf[64];
        ::sprintf(buf,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",a1,a2,a3,a4,a5,a6);
        return std::string(buf);
    }
    static RobotAxle instance(string str = "");
} ;

bool operator==(const RobotAxle &p1, const RobotAxle &p2 ) ;

bool operator!=(const RobotAxle &p1, const RobotAxle &p2 ) ;

const RobotAxle operator-(const RobotAxle &p1, const RobotAxle &p2) ;

const RobotAxle operator+(const RobotAxle &p1, const RobotAxle &p2) ;

const RobotAxle operator*(const RobotAxle &p, double percent) ;

RobotAxle &operator+=(RobotAxle &p1, const RobotAxle &p2) ;

RobotAxle &operator-=(RobotAxle &p1, const RobotAxle &p2) ;

/// 获取TCP点和法兰的关系
/// @param [in] tcp TCP点
/// @return TCP点和法兰的关系
RobotPos getPosInFlange( RobotTCP tcp ) ;

/*********************************************End RobootPos*************************************************/

/*********************************************Begin PosInfo*************************************************/

struct PosInfo
{
   RobotPos pos ;
   RobotAxle axle ;
   PosType posType ;
   int positionerRotate ;
   std::string strSeamAlgo ;
   bool bEnabled ;
} ;

/*********************************************End PosInfo*************************************************/

/*********************************************Begin UV*************************************************/
struct UV
{
   double u ;
   double v ;
};

bool operator==( const UV &uv1, const UV &uv2 ) ;

UV operator+( const UV &uv1, const UV &uv2 ) ;

UV operator-( const UV &uv1, const UV &uv2 ) ;

UV operator/( const UV &uv, double d ) ;

double lenLine( const UV &uv1, const UV &uv2 ) ;

/// 焊缝信息
//struct SeamInfo
//{
//   UV uv_left;
//   UV uv_right;
//   UV uv_center;
//} ;

/*********************************************End UV*************************************************/

/*********************************************Begin SeamType***********************************************/

enum class SeamType{LINE = 0, FIT} ;

const char * const SeamTypeString[] = {"LINE", "FIT"} ;

/*********************************************End SeamType*************************************************/

/*********************************************Begin DataTypes**********************************************/
using MatVec = std::vector<std::pair<cv::Mat, RobotPos>> ;
using LineSeam = std::pair<RobotPos, RobotPos> ;
using FitSeam = std::vector<RobotPos> ;
using SeamList = std::list<boost::any> ;
/*********************************************End DataTypes**********************************************/

#endif // _DATA_TYPE_H
