#include "base/adjustui.h"
#include "ui_adjustui.h"
#include "base/data_types.h"
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;

AdjustUI::AdjustUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AdjustUI)
{
    ui->setupUi(this);
    setAttribute(Qt::WA_AlwaysStackOnTop);
}

AdjustUI::~AdjustUI()
{
    delete ui;
}

void Rotate_around_V(Eigen::Vector3d &v, Eigen::Vector3d V, double agl)
{
    Eigen::Matrix3d ri, rj, rx, r1, r2;
    Eigen::Vector3d z1, z2, z, zi, xi, yi, x1, x2;
    V << V(0)/sqrt(V.dot(V)), V(1)/sqrt(V.dot(V)), V(2)/sqrt(V.dot(V));
    zi << 0, 0, 1;
    xi << V(0), V(1), 0;
    xi << xi(0)/sqrt(xi.dot(xi)), xi(1)/sqrt(xi.dot(xi)), 0;
    yi = zi.cross(xi);
    xi = V;
    zi = xi.cross(yi);
    ri << xi(0), yi(0), zi(0),
          xi(1), yi(1), zi(1),
          xi(2), yi(2), zi(2);

    rx << 1,        0,         0,
          0, cos(agl), -sin(agl),
          0, sin(agl),  cos(agl);

    v = ri*rx*ri.inverse()*v;
}



void interpolation2(vector<RobotPos> &seam, vector<RobotPos> pos_set, int Ltype)
{
    Eigen::Matrix4d r, Rz, Rz1, Ry, Rz2;
    vector<Eigen::Matrix4d> R, R1;
    double pi = M_PI;

    for(int j=0; j<(int)pos_set.size()-1; j++){
        for(int i=j; i<j+2; i++){
            Rz1 << cos(pos_set[i].a/(180/pi)), -sin(pos_set[i].a/(180/pi)), 0, 0,
                   sin(pos_set[i].a/(180/pi)),  cos(pos_set[i].a/(180/pi)), 0, 0,
                                            0,                           0, 1, 0,
                                            0,                           0, 0, 1;

            Ry <<  cos(pos_set[i].b/(180/pi)),  0, sin(pos_set[i].b/(180/pi)), 0,
                                            0,  1,                          0, 0,
                  -sin(pos_set[i].b/(180/pi)),  0, cos(pos_set[i].b/(180/pi)), 0,
                                            0,  0,                          0, 1;

            Rz2 << cos(pos_set[i].c/(180/pi)), -sin(pos_set[i].c/(180/pi)), 0, 0,
                   sin(pos_set[i].c/(180/pi)),  cos(pos_set[i].c/(180/pi)), 0, 0,
                                            0,                           0, 1, 0,
                                            0,                           0, 0, 1;
            r = Rz1*Ry*Rz2;
            r(0, 3) = pos_set[i].x ;
            r(1, 3) = pos_set[i].y ;
            r(2, 3) = pos_set[i].z ;
            R.push_back(r);
        }

        Eigen::Matrix4d ri, rj, rx, r1, r2;
        Eigen::Vector3d z1, z2, z, zi, xi, yi, x1, x2;
        double n1, n2;
        z1 << R[0](0,2), R[0](1,2), R[0](2,2);
        z2 << R[1](0,2), R[1](1,2), R[1](2,2);
        z = z1.cross(z2);
        z << z(0)/sqrt(z.dot(z)), z(1)/sqrt(z.dot(z)), z(2)/sqrt(z.dot(z));
        double t1 = z1.dot(z2)/(sqrt(z1.dot(z1))*sqrt(z2.dot(z2)));
        if(t1 >= 1){
            t1 = 1;
            z = {0, 0, 1};   //z與z1不能相等
            z = z.cross(z1);
        }
        double a = acos(t1);
        zi << 0, 0, 1;
        xi << z(0), z(1), 0;
        xi << xi(0)/sqrt(xi.dot(xi)), xi(1)/sqrt(xi.dot(xi)), 0;
        yi = zi.cross(xi);
        xi = z;
        zi = xi.cross(yi);
        ri << xi(0), yi(0), zi(0), 0,
              xi(1), yi(1), zi(1), 0,
              xi(2), yi(2), zi(2), 0,
                  0,     0,     0, 1;

        rx << 1,      0,       0,  0,
              0, cos(a), -sin(a),  0,
              0, sin(a),  cos(a),  0,
              0,      0,       0,  1;

        r1 = R[0];
        r2 = R[1];
        r1 = ri*rx*ri.inverse()*r1;
        x1 << r1(0,0), r1(1,0), r1(2,0);
        x2 << r2(0,0), r2(1,0), r2(2,0);
        z2 << r2(0,2), r2(1,2), r2(2,2);

        double t2 = x1.dot(x2)/(sqrt(x1.dot(x1))*sqrt(x2.dot(x2)));
        if(t2 > 1)
            t2 = 1;
        double a2 = acos(t2);
        z = x1.cross(x2);
        z << z(0)/sqrt(z.dot(z)), z(1)/sqrt(z.dot(z)), z(2)/sqrt(z.dot(z));
        if(z.dot(z2) < 0)
            a2 = - a2;

        if(Ltype == 2){
            if(a2 >= 0)
                a2 = a2 - 2*pi;
            else
                a2 = a2 + 2*pi;
        }
        if(Ltype == 3){
            a = a - pi;
        }
        if(Ltype == 4){
            a = a - pi;
            if(a2 >= 0)
                a2 = a2 - 2*pi;
            else
                a2 = a2 + 2*pi;
        }
        R.clear();

        for(int i=0; i<(int)seam.size(); i++){
            if(pos_set[j].x == seam[i].x && pos_set[j].y == seam[i].y && pos_set[j].z == seam[i].z)
                n1 = i;
            if(pos_set[j+1].x == seam[i].x && pos_set[j+1].y == seam[i].y && pos_set[j+1].z == seam[i].z){
                n2 = i;
                break;
            }
        }

        int N = n2 - n1;
        for(int i=n1; i<n2; i++){
            rx << 1,               0,                0,  0,
                  0, cos(a*(i-n1)/N), -sin(a*(i-n1)/N),  0,
                  0, sin(a*(i-n1)/N),  cos(a*(i-n1)/N),  0,
                  0,               0,                0,  1;

            Rz <<  cos(a2*(i-n1)/N), -sin(a2*(i-n1)/N), 0, 0,
                   sin(a2*(i-n1)/N),  cos(a2*(i-n1)/N), 0, 0,
                                  0,                 0, 1, 0,
                                  0,                 0, 0, 1;

            Rz1 << cos(pos_set[j].a/(180/pi)), -sin(pos_set[j].a/(180/pi)), 0, 0,
                   sin(pos_set[j].a/(180/pi)),  cos(pos_set[j].a/(180/pi)), 0, 0,
                                            0,                           0, 1, 0,
                                            0,                           0, 0, 1;

            Ry <<  cos(pos_set[j].b/(180/pi)),  0, sin(pos_set[j].b/(180/pi)), 0,
                                            0,  1,                          0, 0,
                  -sin(pos_set[j].b/(180/pi)),  0, cos(pos_set[j].b/(180/pi)), 0,
                                            0,  0,                          0, 1;

            Rz2 << cos(pos_set[j].c/(180/pi)), -sin(pos_set[j].c/(180/pi)), 0, 0,
                   sin(pos_set[j].c/(180/pi)),  cos(pos_set[j].c/(180/pi)), 0, 0,
                                            0,                           0, 1, 0,
                                            0,                           0, 0, 1;
            r = Rz1*Ry*Rz2;
            r = ri*rx*ri.inverse()*r*Rz;
            r(0, 3) = seam[i].x ;
            r(1, 3) = seam[i].y ;
            r(2, 3) = seam[i].z ;
            R1.push_back(r);
        }
    }

    double a, b, c;
    RobotPos pos, pos_end;
    pos_end = pos_set[pos_set.size()-1];
    seam.clear();
    for(int i=0; i<(int)R1.size(); i++){
        b = acos(R1[i](2, 2));
        a = atan2(R1[i](1,2),R1[i](0,2));
        c = atan2((-1*R1[i](0,0)*sin(a)+R1[i](1,0)*cos(a)),(R1[i](1,1)*cos(a)-R1[i](0,1)*sin(a)));
        pos.x = R1[i](0, 3);
        pos.y = R1[i](1, 3);
        pos.z = R1[i](2, 3);
        pos.a = a * 180 / M_PI;
        pos.b = b * 180 / M_PI;
        pos.c = c * 180 / M_PI;
        seam.push_back(pos);
    }
    seam.push_back(pos_end);
    R1.clear();
}

void Insertion_point(RobotPos p0, RobotPos p1, RobotPos p2, RobotPos p3, double len, vector<RobotPos> & pOut)
{
    Eigen::Vector3d V01 = {p1.x-p0.x, p1.y-p0.y, p1.z-p0.z};
    Eigen::Vector3d V23 = {p3.x-p2.x, p3.y-p2.y, p3.z-p2.z};
    Eigen::Vector3d Vz = V01.cross(V23);
    Eigen::Vector3d V2O = Vz.cross(V23);
    V2O = len * V2O / sqrt(V2O.dot(V2O));
    Eigen::Vector3d pO = {p2.x+V2O(0), p2.y+V2O(1), p2.z+V2O(2)};
    Eigen::Vector3d VO1 = {p1.x-pO(0), p1.y-pO(1), p1.z-pO(2)};
    pOut.push_back(p1);
    for(int i=0; i<5; i++){
        Rotate_around_V(VO1, Vz, M_PI/12);
        Eigen::Vector3d pi = pO + VO1;
        RobotPos p = RobotPos::instance();
        p.x = pi(0);
        p.y = pi(1);
        p.z = pi(2);
        pOut.push_back(p);
    }
     pOut.push_back(p2);
     vector<RobotPos> pOut2;
     pOut2.push_back(p1);
     pOut2.push_back(p2);
     interpolation2(pOut, pOut2, 1);
     pOut2.clear();
     for(int i=1; i<6; i++)
         pOut2.push_back(pOut[i]);
     pOut.clear();
     pOut = pOut2;
}
