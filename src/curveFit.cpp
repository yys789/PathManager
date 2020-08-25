#include "include/curveFit.h"


curveFit::curveFit()
{
    Sx = 0;
    Ex = 0;
}

void curveFit::build(int degree)
{
    int rows = curveFitX.size();
    int columns = degree+1;
    Eigen::MatrixXd A(rows, columns);
    Fy(columns, 1);
    Fz(columns, 1);
    Eigen::MatrixXd By(rows, 1);
    Eigen::MatrixXd Bz(rows, 1);

    // 拟合Y(x)
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<columns; j++)
        {
            A(i,j) = pow(curveFitX[i], columns-j-1);
        }
        By(i,0) = curveFitY[i];
    }
    Fy = (A.transpose()*A).inverse()*A.transpose()*By;

    // 拟合Z(x)
    for(int i=0; i<rows; i++)
    {
        Bz(i,0) = curveFitZ[i];
    }
    Fz = (A.transpose()*A).inverse()*A.transpose()*Bz;

    bool arriveEnd;
    Eigen::MatrixXd Xi(1, columns);
    vector<double> point;
    Eigen::MatrixXd x, y, z;
    double dx = 0.001;
    Sx = curveFitX[0];
    Ex = curveFitX[curveFitX.size()-1];
    for(int i=0; i<1e10; i++)
    {
        x(0,0) = Sx + i*dx;
        if(x(0,0)>Ex)
            break;
        for(int j=0; j<columns; j++)
            Xi(1,j) = pow(x(0,0), columns-j-1);
        y = Xi*Fy;
        z = Xi*Fz;
        point = {x(0,0), y(0,0), z(0,0)};
        pointLst.push_back(point);
    }

    double len = 0;
    vector<double> point1, point2;
    lenLst.push_back(0);
    for(int i=0; i<pointLst.size()-1; i++)
    {
        point1 = pointLst[i];
        point2 = pointLst[i+1];
        len += sqrt(pow(point2[0]-point1[0],2)+pow(point2[1]-point1[1],2)+pow(point2[2]-point1[2],2));
        lenLst.push_back(len);
    }
}


void curveFit::getPoint(double t, vector<double> &point)
{
    for(int i=0; i<lenLst.size(); i++)
    {
        if(t<lenLst[i])
        {
            point = pointLst[i];
        }
    }
}



























