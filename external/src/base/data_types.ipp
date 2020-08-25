#include "base.h"
#include "base/config.h"
#include "base/wzerror.h"

/*********************************************Begin TypeDef**************************************************/
std::chrono::milliseconds operator"" _ms( unsigned long long t )
{
   return std::chrono::milliseconds(t) ;
}
/*********************************************End TypeDef****************************************************/

/*********************************************Begin RobotPos*************************************************/
//bool operator==(const XZ &xz1, const XZ &xz2 )
//{
//   return xz1.x == xz2.x && xz1.z == xz2.z ;
//}

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

RobotPos operator-(const RobotPos &p1, const RobotPos &p2)
{
    return {p1.x - p2.x, p1.y - p2.y, p1.z - p2.z,
            p1.a - p2.a, p1.b - p2.b, p1.c - p2.c, p1.tcp} ;
}

RobotPos operator+(const RobotPos &p1, const RobotPos &p2)
{
    return {p1.x + p2.x, p1.y + p2.y, p1.z + p2.z,
            p1.a + p2.a, p1.b + p2.b, p1.c + p2.c, p1.tcp} ;
}

RobotPos operator*(const RobotPos &p1, double percent )
{
   return {p1.x * percent, p1.y * percent, p1.z * percent,
           p1.a * percent, p1.b * percent, p1.z * percent, p1.tcp} ;
}

RobotPos &operator+=(RobotPos &p1, const RobotPos &p2)
{
   return p1 = p1 + p2 ;
}

RobotPos &operator-=(RobotPos &p1, const RobotPos &p2)
{
   return p1 = p1 - p2 ;
}

//RobotPos operator+( const RobotPos &p1, const double relativePos[6] )
//{
//    return {p1.x + relativePos[0],
//            p1.y + relativePos[1],
//            p1.z + relativePos[2],
//            p1.a + relativePos[3],
//            p1.b + relativePos[4],
//            p1.c + relativePos[5], p1.tcp} ;
//}

//void pos2Double( const RobotPos &p, double pos[6] )
//{
//    pos[0] = p.x ;
//    pos[1] = p.y ;
//    pos[2] = p.z ;
//    pos[3] = p.a ;
//    pos[4] = p.b ;
//    pos[5] = p.c ;
//}
//
//void double2Pos(const std::array<double, 6> pos, RobotPos &p)
//{
//    p.x = pos[0] ;
//    p.y = pos[1] ;
//    p.z = pos[2] ;
//    p.a = pos[3] ;
//    p.b = pos[4] ;
//    p.c = pos[5] ;
//}
void file2Path(const std::string &fileName, std::vector<RobotPos> &pos_vec)
{
    std::ifstream ifs(fileName, std::ios::binary ) ;
    RobotPos pos{} ;
    while ( ifs >> pos ) {
        pos_vec.push_back(pos);
    }
    ifs.close() ;
}

//void double2Pos(const double pos[6], RobotPos &p)
//{
//    p.x = pos[0] ;
//    p.y = pos[1] ;
//    p.z = pos[2] ;
//    p.a = pos[3] ;
//    p.b = pos[4] ;
//    p.c = pos[5] ;
//}

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

RobotPos getPosInFlange( RobotTCP tcp )
{
   RobotPos posInFlange{} ;
   static Config config("etc/calibration.info") ;
   switch ( tcp ) {
   case RobotTCP::CAMERA:
       posInFlange = config.get<RobotPos>("TINC");
       break ;
   case RobotTCP::VIRTUAL:
       posInFlange = config.get<RobotPos>("virtualInFlange");
       break;
   case RobotTCP::WELDING:
       posInFlange = config.get<RobotPos>("weldingInFlange");
       break ;
   case RobotTCP::FLANGE:
       break ;
   case RobotTCP::CURRENT:
       BOOST_THROW_EXCEPTION(WZError() << err_pos("TCP点无CURRENT") ) ;
      break ;
   }
   return posInFlange ;
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

/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/
/*********************************************Begin RobotPos*************************************************/

