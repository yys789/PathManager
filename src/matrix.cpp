#include "include/matrix.h"
#include "include/defines.h"


void Matrix2pos(Eigen::Matrix4d &m,  RobotPos &pos)
{
    double a, b, c;

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


void pos2Matrix(const RobotPos &pos, Eigen::Matrix4d &m)
{
    double a = pos.a * M_PI / 180 ;
    double b = pos.b * M_PI / 180 ;
    double c = pos.c * M_PI / 180 ;

   Eigen::Matrix4d rz1, ry, rz2;
   rz1 << cos(a), -sin(a), 0, 0,
          sin(a),  cos(a), 0, 0,
               0,       0, 1, 0,
               0,       0, 0, 1;

   ry  << cos(b), 0, sin(b), 0,
               0, 1,      0, 0,
         -sin(b), 0, cos(b), 0,
               0, 0,      0, 1;

   rz2 << cos(c), -sin(c), 0, 0,
          sin(c),  cos(c), 0, 0,
               0,       0, 1, 0,
               0,       0, 0, 1;
   m = rz1*ry*rz2;

    m(0, 3) = pos.x;
    m(1, 3) = pos.y;
    m(2, 3) = pos.z;
}


void PosRotate(RobotPos cpos, RobotPos &pos, double agl)
{
    Eigen::Matrix4d Mbf, Maf, Mp, Mr, pInb, pIna;
    agl = agl/(180/M_PI);
    Mr << cos(agl), -sin(agl), 0, 0,
          sin(agl),  cos(agl), 0, 0,
                 0,         0, 1, 0,
                 0,         0, 0, 1;
    pos2Matrix(cpos, Mbf);
    pos2Matrix(pos, Mp);
    Maf = Mbf*Mr;
    pInb = Mbf.inverse()*Mp;
    pIna = pInb;
    Mp =  Maf*pIna;
    Matrix2pos(Mp, pos);
}









