#include "include/circleFit.h"


void circleFit::build()
{
    // 拟合平面z=ax+by+c，ax+by-z+c=0
    int rows = pointLst.size();
    Eigen::MatrixXd A(rows, 3);
    Eigen::MatrixXd X(3, 1);
    Eigen::MatrixXd B(rows, 1);
    for(int i=0; i<rows; i++)
    {
        A(i,0) = pointLst[i].x;
        A(i,1) = pointLst[i].y;
        A(i,2) = 1;
        B(i,0) = pointLst[i].z;
    }
    X = (A.transpose()*A).inverse()*A.transpose()*B;  // X = {a,b,c}

    // 取平面上一点作为平面坐标系(0,0,c,...)
    Eigen::Vector3d x,y,z;
    Eigen::Matrix4d Mc, Mi, MPInC;
    z << -X(0), -X(1), 1;
    y << 1, 1, 1;
    x = y.cross(z);
    y = z.cross(x);
    x = x/sqrt(x.dot(x));
    y = y/sqrt(y.dot(y));
    z = z/sqrt(z.dot(z));
    Mc << x(0), y(0), z(0),    0,
          x(1), y(1), z(1),    0,
          x(2), y(2), z(2), X(2),
             0,    0,    0,    1;
    RobotPos pos, PInC;
    vector<RobotPos> posLst;
    for(int i=0; i<pointLst.size(); i++)
    {
        pos = {pointLst[i].x,pointLst[i].y,pointLst[i].z,0,0,0};
        pos2Matrix(pos, Mi);
        MPInC = Mc.inverse()*Mi;
        Matrix2pos(MPInC, pos);
        posLst.push_back(pos);
    }

    // 拟合圆(xi-x0)^2+(yi-y0)^2 = r^2
    Eigen::MatrixXd A1(rows-1, 2);
    Eigen::MatrixXd X1(2, 1);
    Eigen::MatrixXd B1(rows-1, 1);
    for(int i=0; i<rows-1; i++)
    {
        A1(i,0) = 2*(posLst[i+1].x-posLst[i].x);
        A1(i,1) = 2*(posLst[i+1].y-posLst[i].y);
        B1(i,0) = pow(posLst[i+1].x,2)+pow(posLst[i+1].y,2)-pow(posLst[i].x,2)-pow(posLst[i].y,2);
    }
    X1 = (A1.transpose()*A1).inverse()*A1.transpose()*B1; // X1 = (x0,y0)

    // 计算半径与rms
    radius = 0;
    rms = 0;
    for(int i=0; i<posLst.size(); i++)
    {
        pos = posLst[i];
        radius += sqrt(pow(pos.x-X1(0),2)+pow(pos.y-X1(1),2));
    }
    radius = radius/posLst.size();
    for(int i=0; i<posLst.size(); i++)
    {
        pos = posLst[i];
        rms += pow(sqrt(pow(pos.x-X1(0),2)+pow(pos.y-X1(1),2))-radius, 2);
    }
    rms = rms/posLst.size();
    rms = sqrt(rms);

    z << 0,0,1;
    x << posLst[0].x-X1(0), posLst[0].y-X1(1), 0;
    y = z.cross(x);
    x = x/sqrt(x.dot(x));
    y = y/sqrt(y.dot(y));
    z = z/sqrt(z.dot(z));
    MPInC << x(0), y(0), z(0), X1(0),
             x(1), y(1), z(1), X1(1),
             x(2), y(2), z(2),     0,
               0,    0,    0,      1;
    Mc = Mc*MPInC;
    Matrix2pos(Mc, cpos);
    qDebug() << cpos.x << "," << cpos.y << "," << cpos.z << ","
             << cpos.a << "," << cpos.b << "," << cpos.c <<  "\n";
}


void circleFit::interpolation(double aglS, double interval, vector<RobotPos> &path)
{
    double agl = 360*(interval/(2*M_PI*radius));
    int posNum = 360/agl + 1;
    RobotPos pos, pInC;
    pInC = {radius,0,0,0,0,0};
    Eigen::Matrix4d MPInC, MP;
    pos2Matrix(pInC, MPInC);
    for(int i=0; i<posNum; i++)
    {
        pos = cpos;
        PosRotate(cpos, pos, i*agl);
        pos2Matrix(pos, MP);
        MP = MP*MPInC;
        Matrix2pos(MP, pos);
        path.push_back(pos);
    }
}





void circleFit::getRms()
{

}




























