#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
//    configPath = "PumpDown.info";
//    filename = "PumpDown.txt";
//    configPath = "PumpUp.info";
//    filename = "PumpUp.txt";
//    configPath = "VertebralBodyUp.info";
//    filename = "VertebralBodyUp.txt";
//    configPath = "VertebralBodyDown.info";
//    filename = "VertebralBodyDown.txt";
//    configPath = "configuration/ImpellerUp.info";
//    filename = "pathData/ImpellerUp.txt";
//    configPath = "configuration/vertebralbodyAUp.info";
//    filename = "pathData/vertebralbodyAUp.txt";
//    configPath = "configuration/vertebralbodyADown.info";
//    filename = "pathData/vertebralbodyADown.txt";
    configPath = "configuration/vertebralbodyDDown.info";
    filename = "pathData/vertebralbodyDDown.txt";
    aglBest = {0,0,-90,0,-60,0};
    agl_acl = {-15, -36, -132, 44, -62, -66};
    backPosNum = 0;
    backDistance = 100;
    readConfig_RobotPos(configPath, "cposa", cposa);
    readConfig_RobotPos(configPath, "cposb", cposb);
    readConfig_RobotPos(configPath, "TInF", TInF);
    readConfig_RobotPos(configPath, "CInF", CInF);
    readConfig_RobotPos(configPath, "CInT", CInT);
    readConfig_RobotPos(configPath, "posW1", posW1);
    readConfig_RobotPos(configPath, "posW2", posW2);
    readConfig_agl(configPath, "parameterAgl", aglWeld, aglInclude, aglHead, aglEnd);
    readConfig_Str(configPath, "GParameter", "CadName", CadName);
}

MainWindow::~MainWindow()
{
    delete ui;
}


// 读配置文件
void MainWindow::readConfig2(string configPath,
                            vector<vector<int>> RelateIndex,
                            vector<vector<int>> &pathDirection,
                            vector<vector<int>> &laserDirection,
                            vector<vector<int>> &rotateYEnd,
                            vector<vector<int>> &rotateYHead,
                            vector<vector<double>> &region,
                            vector<vector<string>> &nameLsts,
                             vector<vector<int>> &orderIDLsts,
                             vector<vector<int>> &rankLsts,
                             vector<vector<int>> &TracingLsts)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    vector<int> vp, vl, vre, vrh, vo, vra, vtr;
    vector<string> vn;
    for(int i=0; i<RelateIndex.size(); i++)
    {
        for(int j=0; j<RelateIndex[i].size(); j++)
        {
            child = pt.get_child(std::to_string(RelateIndex[i][j]));
            vp.push_back(child.get<int>("pathDirection"));
            vl.push_back(child.get<int>("laserDirection"));
            vre.push_back(child.get<int>("rotateYEnd"));
            vrh.push_back(child.get<int>("rotateYHead"));
            vn.push_back(child.get<string>("seamName"));
            vo.push_back(child.get<int>("orderID"));
            vra.push_back(child.get<int>("rank"));
            vtr.push_back(child.get<int>("Tracing"));
        }
        pathDirection.push_back(vp);
        laserDirection.push_back(vl);
        rotateYEnd.push_back(vre);
        rotateYHead.push_back(vrh);
        nameLsts.push_back(vn);
        orderIDLsts.push_back(vo);
        rankLsts.push_back(vra);
        TracingLsts.push_back(vtr);
        vp.clear();
        vl.clear();
        vre.clear();
        vrh.clear();
        vn.clear();
        vo.clear();
        vra.clear();
        vtr.clear();
    }
    child = pt.get_child(std::to_string(RelateIndex[0][0]));
    region.push_back({child.get<int>("Xnum"),child.get<int>("robotX1"),child.get<int>("robotX2")});
    region.push_back({child.get<int>("Ynum"),child.get<int>("robotY1"),child.get<int>("robotY2")});
    region.push_back({child.get<int>("Znum"),child.get<int>("robotZ1"),child.get<int>("robotZ2")});
    region.push_back({child.get<int>("rotateNum"),child.get<int>("rotatePhase1"),child.get<int>("rotatePhase2")});
    region.push_back({child.get<int>("flipNum"),child.get<int>("flipPhase1"),child.get<int>("flipPhase2")});
    region.push_back({child.get<int>("VXnum"),child.get<double>("VX1"),child.get<double>("VX2")});
    region.push_back({child.get<int>("VYnum"),child.get<double>("VY1"),child.get<double>("VY2")});
    region.push_back({child.get<int>("VZnum"),child.get<double>("VZ1"),child.get<double>("VZ2")});
    region.push_back({child.get<int>("rotateVNum"),child.get<double>("rotateV1"),child.get<double>("rotateV2")});
    region.push_back({child.get<int>("flipVNum"),child.get<double>("flipV1"),child.get<double>("flipV2")});
}


// 读配置文件
void MainWindow::readConfig(string configPath,
                            vector<vector<int>> RelateIndex,
                            vector<vector<int>> &pathDirection,
                            vector<vector<int>> &laserDirection,
                            vector<vector<int>> &rotateYEnd,
                            vector<vector<int>> &rotateYHead,
                            vector<vector<double>> &region,
                            vector<vector<string>> &nameLsts)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    vector<int> vp, vl, vre, vrh;
    vector<string> vn;
    for(int i=0; i<RelateIndex.size(); i++)
    {
        for(int j=0; j<RelateIndex[i].size(); j++)
        {
            child = pt.get_child(std::to_string(RelateIndex[i][j]));
            vp.push_back(child.get<int>("pathDirection"));
            vl.push_back(child.get<int>("laserDirection"));
            vre.push_back(child.get<int>("rotateYEnd"));
            vrh.push_back(child.get<int>("rotateYHead"));
            vn.push_back(child.get<string>("seamName"));
        }
        pathDirection.push_back(vp);
        laserDirection.push_back(vl);
        rotateYEnd.push_back(vre);
        rotateYHead.push_back(vrh);
        nameLsts.push_back(vn);
        vp.clear();
        vl.clear();
        vre.clear();
        vrh.clear();
        vn.clear();
    }
    child = pt.get_child(std::to_string(RelateIndex[0][0]));
    region.push_back({child.get<int>("Xnum"),child.get<double>("robotX1"),child.get<double>("robotX2")});
    region.push_back({child.get<int>("Ynum"),child.get<double>("robotY1"),child.get<double>("robotY2")});
    region.push_back({child.get<int>("Znum"),child.get<double>("robotZ1"),child.get<double>("robotZ2")});
    region.push_back({child.get<int>("rotateNum"),child.get<double>("rotatePhase1"),child.get<double>("rotatePhase2")});
    region.push_back({child.get<int>("flipNum"),child.get<double>("flipPhase1"),child.get<double>("flipPhase2")});
    region.push_back({child.get<int>("VXnum"),child.get<double>("VX1"),child.get<double>("VX2")});
    region.push_back({child.get<int>("VYnum"),child.get<double>("VY1"),child.get<double>("VY2")});
    region.push_back({child.get<int>("VZnum"),child.get<double>("VZ1"),child.get<double>("VZ2")});
    region.push_back({child.get<int>("rotateVNum"),child.get<double>("rotateV1"),child.get<double>("rotateV2")});
    region.push_back({child.get<int>("flipVNum"),child.get<double>("flipV1"),child.get<double>("flipV2")});
}


// 读配置文件(读取角度)
void MainWindow::readConfig_agl(string configPath, string name, double &aglWeld, double &aglInclude, double &aglHead, double &aglEnd)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    aglWeld = child.get<double>("aglWeld");
    aglInclude = child.get<double>("aglInclude");
    aglHead = child.get<double>("aglHead");
    aglEnd = child.get<double>("aglEnd");
}


// 读配置文件(RobotPos)
void MainWindow::readConfig_RobotPos(string configPath, string name, RobotPos &pos)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    pos.x = child.get<double>("x");
    pos.y = child.get<double>("y");
    pos.z = child.get<double>("z");
    pos.a = child.get<double>("a");
    pos.b = child.get<double>("b");
    pos.c = child.get<double>("c");
}


// 读配置文件(读取字符串)
void MainWindow::readConfig_Str(string configPath, string name, string petName, string &Str)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    Str = child.get<string>(petName);
}


// 读配置文件(读取double)
void MainWindow::readConfig_Dou(string configPath, string name, string petName, double &Dou)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    Dou = child.get<double>(petName);
}


// 读配置文件(读取int)
void MainWindow::readConfig_Int(string configPath, string name, string petName, int &Int)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    Int = child.get<int>(petName);
}


// 判断某个关键字是否存在()
void MainWindow::readConfig_Find(string configPath, string name, string petName)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    boost::property_tree::ptree::assoc_iterator it;
//    it = child.find(petName);
}


//读取焊缝关联关系
void MainWindow::readConfig_Seam(string configPath, vector<vector<vector<int>>> &RelateIndexSeek, vector<vector<vector<int>>> &RelateIndexTrack)
{
   boost::property_tree::ptree pt, child1, child2, child3;
   vector<string> state, relation, seam;
   string seamID;
   int index;
   vector<int> indexLst;
   vector<vector<int>> indexLsts;
   bool finish;
   state = {"state1","state2","state3","state4","state5","state6","state7","state8","state9","state10"};
   relation = {"relation1","relation2","relation3","relation4","relation5","relation6","relation7","relation8","relation9","relation10"};
   seam = {"seam1","seam2","seam3","seam4","seam5","seam6","seam7","seam8","seam9","seam10"};
   read_info(configPath, pt);

   child1 = pt.get_child("Seek_Relation");
   finish = false;
   for(int i=0; i<10; i++)
   {
       if(finish)
           break;
       child2 = child1.get_child(state[i]);
       for(int j=0; j<10; j++)
       {
           if(finish)
               break;
           child3 = child2.get_child(relation[j]);
           for(int k=0; k<10; k++)
           {
               seamID = child3.get<string>(seam[k]);
               if(j==0 && k==0 && seamID=="end")
               {
                   finish = true;
                   break;
               }
               else if(k==0 && seamID=="end")
               {
                   RelateIndexSeek.push_back(indexLsts);
                   indexLsts.clear();
               }
               else if(seamID=="end")
               {
                   indexLsts.push_back(indexLst);
                   indexLst.clear();
               }
               else
               {
                   index = atoi(seamID.c_str());
                   indexLst.push_back(index);
               }
           }
       }
   }

   child1 = pt.get_child("Track_Relation");
   finish = false;
   for(int i=0; i<10; i++)
   {
       if(finish)
           break;
       child2 = child1.get_child(state[i]);
       for(int j=0; j<10; j++)
       {
           if(finish)
               break;
           child3 = child2.get_child(relation[j]);
           for(int k=0; k<10; k++)
           {
               seamID = child3.get<string>(seam[k]);
               if(j==0 && k==0 && seamID=="end")
               {
                   finish = true;
                   break;
               }
               else if(k==0 && seamID=="end")
               {
                   RelateIndexTrack.push_back(indexLsts);
                   indexLsts.clear();
               }
               else if(seamID=="end")
               {
                   indexLsts.push_back(indexLst);
                   indexLst.clear();
               }
               else
               {
                   index = atoi(seamID.c_str());
                   indexLst.push_back(index);
               }
           }
       }
   }

}


// 得到零点状态下的工具坐标系
void MainWindow::GetToolCoordinate(RobotPos &TPos, double orbitX, double rotateAgl, double flipAgl)
{
    TPos.x += orbitX;
    PosRotate(cposb, TPos, -flipAgl);
    PosRotate(cposa, TPos, -rotateAgl);
}


// 寻找船型焊起始相位角
void MainWindow::findStartAngle(RobotPos pos, vector<double> &rotateRegion, vector<double> &flipRegion)
{
    int rA1 ,rA2, fA1, fA2;
    if(rotateRegion[1]>rotateRegion[2])
    {
        rA2 = rotateRegion[1];
        rA1 = rotateRegion[2];
    }
    else
    {
        rA1 = rotateRegion[1];
        rA2 = rotateRegion[2];
    }
    if(flipRegion[1]>flipRegion[2])
    {
        fA2 = flipRegion[1];
        fA1 = flipRegion[2];
    }
    else
    {
        fA1 = flipRegion[1];
        fA2 = flipRegion[2];
    }
    RobotPos r;
    double f, fmin;
    vector<double> fLst;
    vector<int> aglPair, aglPairB;
    vector<vector<int>> aglLst;
    for(int i=fA1; i<=fA2; i++)
    {
        for(int j=rA1; j<=rA2; j++)
        {
            r = pos;
            PosRotate(cposa, r, j);
            PosRotate(cposb, r, i);
            f = 180-abs(r.b);
            aglPair = {j,i};
            fLst.push_back(f);
            aglLst.push_back(aglPair);
        }
    }
    fmin = 1e10;
    for(int i=0; i<fLst.size(); i++)
    {
        if(fmin>fLst[i]){
            fmin = fLst[i];
            aglPairB = aglLst[i];
        }
    }

    rotateRegion[1] = aglPairB[0];
    rotateRegion[2] = aglPairB[0];
    flipRegion[1] = aglPairB[1];
    flipRegion[2] = aglPairB[1];
}


// 回撤处理
void MainWindow::RetracementProcess(vector<RobotPos> &path, double distance, int backPosNum)
{
    int posNum = backPosNum;
    double posInterval = -distance/posNum;
    RobotPos posS, posE, posB, posI;
    vector<RobotPos> path2;
    Matrix4d Ms, Me, Mb, Mi;
    posS = path[0];
    posE = path[path.size()-1];
    pos2Matrix(posS, Ms);
    pos2Matrix(posE, Me);

    // 路径首端添加回撤轨迹
    for(int i=posNum; i>0; i--)
    {
        pos2Matrix(posS, Ms);
        posB = {0,0,i*posInterval, 0,0,0};
        pos2Matrix(posB, Mb);
        Mi = Ms*Mb;
        Matrix2pos(Mi, posI);
        path2.push_back(posI);
    }

    // 添加路径本身
    for(int i=0; i<path.size(); i++)
        path2.push_back(path[i]);

    // 路径尾端添加回撤轨迹
    for(int i=1; i<=posNum; i++)
    {
        pos2Matrix(posE, Me);
        posB = {0,0,i*posInterval, 0,0,0};
        pos2Matrix(posB, Mb);
        Mi = Me*Mb;
        Matrix2pos(Mi, posI);
        path2.push_back(posI);
    }
    path.clear();
    path = path2;
}


void MainWindow::interpolation(vector<RobotPos> &seam, vector<RobotPos> pos_set)
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
        R.clear();

        double abs0 = 0.01;
        for(int i=0; i<(int)seam.size(); i++){
            if(abs(pos_set[j].x-seam[i].x)<abs0 && abs(pos_set[j].y-seam[i].y)<abs0 && abs(pos_set[j].z-seam[i].z)<abs0)
                n1 = i;
            if(abs(pos_set[j+1].x-seam[i].x)<abs0 && abs(pos_set[j+1].y-seam[i].y)<abs0 && abs(pos_set[j+1].z-seam[i].z)<abs0){
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

            Rz << cos(a2*(i-n1)/N), -sin(a2*(i-n1)/N), 0, 0,
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


void MainWindow::Matrix2pos(Eigen::Matrix4d &m,  RobotPos &pos)
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


void MainWindow::pos2Matrix(const RobotPos &pos, Eigen::Matrix4d &m)
{
    double a = pos.a * M_PI / 180 ;
    double b = pos.b * M_PI / 180 ;
    double c = pos.c * M_PI / 180 ;

   Matrix4d rz1, ry, rz2;
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


bool MainWindow::Pos2Joint(vector<RobotPos> Pos_set, vector<RobotAxle> &agl_set, RobotAxle agl_acl, RobotPos AInB)
{
    vector<Eigen::Matrix4d> R;
    Eigen::Matrix4d rz1, ry, rz2, r;
    Eigen::Matrix4d Tz, Txyz, Ro, Ra, Rt;
    RobotAxle agl;
    bool Limit = false;

    double dz6 = 115;
    double x = AInB.x;
    double y = AInB.y;
    double z = AInB.z;
    double o = AInB.a/(180/M_PI);
    double a = AInB.b/(180/M_PI);
    double t = AInB.c/(180/M_PI);

    Tz << 1, 0, 0,   0,
          0, 1, 0,   0,
          0, 0, 1, dz6,
          0, 0, 0,   1;
    Txyz << 1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1;
    Ro << cos(o), -sin(o), 0, 0,
          sin(o),  cos(o), 0, 0,
               0,       0, 1, 0,
               0,       0, 0, 1;

    Ra << cos(a), 0, sin(a), 0,
               0, 1,      0, 0,
         -sin(a), 0, cos(a), 0,
               0, 0,      0, 1;

    Rt << cos(t), -sin(t), 0, 0,
          sin(t),  cos(t), 0, 0,
               0,       0, 1, 0,
               0,       0, 0, 1;

    for(auto p : Pos_set){
        rz1 << cos(p.a/(180/M_PI)), -sin(p.a/(180/M_PI)), 0, 0,
               sin(p.a/(180/M_PI)),  cos(p.a/(180/M_PI)), 0, 0,
                                 0,                    0, 1, 0,
                                 0,                    0, 0, 1;

        ry << cos(p.b/(180/M_PI)), 0, sin(p.b/(180/M_PI)), 0,
                                0, 1,                   0, 0,
             -sin(p.b/(180/M_PI)), 0, cos(p.b/(180/M_PI)), 0,
                                0, 0,                   0, 1;

        rz2 << cos(p.c/(180/M_PI)), -sin(p.c/(180/M_PI)), 0, 0,
               sin(p.c/(180/M_PI)),  cos(p.c/(180/M_PI)), 0, 0,
                                 0,                    0, 1, 0,
                                 0,                    0, 0, 1;
        r = rz1*ry*rz2;
        r(0,3) = p.x;
        r(1,3) = p.y;
        r(2,3) = p.z;
        R.push_back(r);
    }
    for(auto &p : R)
        p = p*Rt.inverse()*Ra.inverse()*Ro.inverse()*Txyz.inverse()*Tz.inverse();

    double l, l2, l3, d, h;
    double a_, a2, a3, ad, ah;
    Eigen::Vector3d y4, z4, y4af, z4af, x6, y6, z6, z6af, z5;
    int i = 0;
    for(auto p : R){
        i++;
        agl.a1 = atan2(p(0,3), p(1,3));
        l2 = fabs(550*sin(agl.a1));
        l3 = fabs(sqrt(210*210+700*700)*sin(agl.a1));
        d = p(0,3) - 165*sin(agl.a1);
        h = p(2,3)*sin(agl.a1) - 430*sin(agl.a1);
        l = sqrt(d*d+h*h);
        a_ = atan2(210, 700);
        ah = atan(h/d);
        ad = M_PI/2 - ah;
        double t1 = (l*l+l3*l3-l2*l2)/(2*l*l3);
        double t2 = (l*l+l2*l2-l3*l3)/(2*l*l2);
        if(t1>1 || t1<-1 || t2>1 ||t2<-1){
            Limit = true;
            break;
        }
        a2 = acos(t1);
        a3 = acos(t2);
        agl.a2 = M_PI/2 - a3 - ah;
        agl.a3 = agl.a2 - a2 -ad - a_;
        z6 = {p(0,2), p(1,2), p(2,2)};
        y4 = {cos(agl.a1), -sin(agl.a1), 0};
        z4 = {cos(agl.a3)*sin(agl.a1)*sin(agl.a2) - cos(agl.a2)*sin(agl.a1)*sin(agl.a3),
             cos(agl.a1)*cos(agl.a3)*sin(agl.a2) - cos(agl.a1)*cos(agl.a2)*sin(agl.a3),
             cos(agl.a2)*cos(agl.a3) + sin(agl.a2)*sin(agl.a3)};
        y4af = z4.cross(z6);
        agl.a5 = acos(z4.dot(z6)/(sqrt(z4.dot(z4))*sqrt(z6.dot(z6))));

        if(agl_set.size() == 0){
            if(agl_acl.a5 < 0){
                y4af = -y4af;
                agl.a5 = -agl.a5;
            }
        }
        else{
            if(agl_set[agl_set.size()-1].a5 < 0){
                if(agl_set[agl_set.size()-1].a5 > -1){
                    double a41 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
                    z4af = y4.cross(y4af);
                    if(z4.dot(z4af) <0){
                        a41 = -a41;
                    }
                    y4af = -y4af;
                    double a42 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
                    z4af = y4.cross(y4af);
                    if(z4.dot(z4af) <0){
                        a42 = -a42;
                    }
                    double abs1 = fabs(a41*180/M_PI - agl_set[agl_set.size()-1].a4);
                    double abs2 = fabs(a42*180/M_PI - agl_set[agl_set.size()-1].a4);
                    y4af = -y4af;
                    if(abs1 > abs2){
                        y4af = -y4af;
                        agl.a5 = -agl.a5;
                    }
                }
                else{
                    y4af = -y4af;
                    agl.a5 = -agl.a5;
                }
            }
            else{
                if(agl_set[agl_set.size()-1].a5 < 1){
                    double a41 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
                    z4af = y4.cross(y4af);
                    if(z4.dot(z4af) <0){
                        a41 = -a41;
                    }
                    y4af = -y4af;
                    double a42 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
                    z4af = y4.cross(y4af);
                    if(z4.dot(z4af) <0){
                        a42 = -a42;
                    }
                    double abs1 = fabs(a41*180/M_PI - agl_set[agl_set.size()-1].a4);
                    double abs2 = fabs(a42*180/M_PI - agl_set[agl_set.size()-1].a4);
                    y4af = -y4af;
                    if(abs1 > abs2){
                        y4af = -y4af;
                        agl.a5 = -agl.a5;
                    }
                }
            }
        }

        agl.a4 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
        z4af = y4.cross(y4af);
        if(z4.dot(z4af) <0){
            agl.a4 = -agl.a4;
        }

        y4 = {-cos(agl.a1)*cos(agl.a4)+(-cos(agl.a2)*cos(agl.a3)*sin(agl.a1)-sin(agl.a1)*sin(agl.a2)*sin(agl.a3))*sin(agl.a4),
              cos(agl.a4)*sin(agl.a1)+(-cos(agl.a1)*cos(agl.a2)*cos(agl.a3)-cos(agl.a1)*sin(agl.a2)*sin(agl.a3))*sin(agl.a4),
              (cos(agl.a3)*sin(agl.a2)-cos(agl.a2)*sin(agl.a3))*sin(agl.a4)};
        x6 = {-p(0,0), -p(1,0), -p(2,0)};
        agl.a6 = acos(y4.dot(x6)/(sqrt(y4.dot(y4))*sqrt(x6.dot(x6))));
        z6af = y4.cross(x6);
        if(z6.dot(z6af) < 0)
            agl.a6 = -agl.a6;
        agl.a1 = agl.a1*(180/M_PI);
        agl.a2 = agl.a2*(180/M_PI);
        agl.a3 = agl.a3*(180/M_PI);
        agl.a4 = agl.a4*(180/M_PI);
        agl.a5 = agl.a5*(180/M_PI);
        agl.a6 = agl.a6*(180/M_PI);

        if(agl_set.size() == 0){
            if(agl_acl.a4 > 150){
                if(agl.a4 < 0)
                    agl.a4 = 360 + agl.a4;
            }
            else if(agl_acl.a4 < -150){
                if(agl.a4 > 0)
                    agl.a4 = agl.a4 -360;
            }
        }
        else{
            if(agl_set[agl_set.size()-1].a4 > 150){
                if(agl.a4 < 0)
                    agl.a4 = 360 + agl.a4;
            }
            else if(agl_set[agl_set.size()-1].a4 < -150){
                if(agl.a4 > 0)
                    agl.a4 = agl.a4 -360;
            }
        }

        if(agl_set.size() == 0){
            if(agl_acl.a6 > 330){
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 + 360;
                if(agl.a6 < 0)
                    agl.a6 = agl.a6 + 360;
            }
            else if(agl_acl.a6 < -330){
                if(agl.a6 < 0)
                    agl.a6 = agl.a6 -360;
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 -360;
            }
            else if(agl_acl.a6 > 150){
                if(agl.a6 < 0)
                    agl.a6 = 360 + agl.a6;
            }
            else if(agl_acl.a6 < -150){
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 -360;
            }
        }
        else{
            if(agl_set[agl_set.size()-1].a6 > 330){
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 + 360;
                if(agl.a6 < 0)
                    agl.a6 = agl.a6 + 360;
            }
            else if(agl_set[agl_set.size()-1].a6 < -330){
                if(agl.a6 < 0)
                    agl.a6 = agl.a6 -360;
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 -360;
            }
            else if(agl_set[agl_set.size()-1].a6 > 150){
                if(agl.a6 < 0)
                    agl.a6 = 360 + agl.a6;
            }
            else if(agl_set[agl_set.size()-1].a6 < -150){
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 -360;
            }
        }

        agl_set.push_back(agl);

        if(agl_set.size() > 1){
            if(agl_set[agl_set.size()-1].a4 < -180 || agl_set[agl_set.size()-1].a4 > 180){
                Limit = true;
                break;
            }
            if(agl_set[agl_set.size()-1].a6 < -360 || agl_set[agl_set.size()-1].a6 > 360){
                Limit = true;
                break;
            }

//            if(agl_set[agl_set.size()-1].a1 >= 45 || agl_set[agl_set.size()-1].a1 <= -45){
//                Limit = true;
//                break;
//            }
//            if(agl_set[agl_set.size()-1].a3 >= -45 || agl_set[agl_set.size()-1].a3 <= -135){
//                Limit = true;
//                break;
//            }
//            if(agl_set[agl_set.size()-1].a5 >= -15 || agl_set[agl_set.size()-1].a5 <= -105){
//                Limit = true;
//                break;
//            }
            if(agl_set[agl_set.size()-1].a6 >= 180 || agl_set[agl_set.size()-1].a6 <= -180){
                Limit = true;
                break;
            }
        }
    }

    return Limit;
}


bool MainWindow::Pos2Joint2(vector<RobotPos> Pos_set, vector<RobotAxle> &agl_set, RobotAxle agl_acl, RobotPos AInB)
{
    vector<Eigen::Matrix4d> R;
    Eigen::Matrix4d rz1, ry, rz2, r;
    Eigen::Matrix4d Tz, Txyz, Ro, Ra, Rt;
    RobotAxle agl;
    bool Limit = false;

    double dz6 = 115;
    double x = AInB.x;
    double y = AInB.y;
    double z = AInB.z;
    double o = AInB.a/(180/M_PI);
    double a = AInB.b/(180/M_PI);
    double t = AInB.c/(180/M_PI);

    Tz << 1, 0, 0,   0,
          0, 1, 0,   0,
          0, 0, 1, dz6,
          0, 0, 0,   1;
    Txyz << 1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1;
    Ro << cos(o), -sin(o), 0, 0,
          sin(o),  cos(o), 0, 0,
               0,       0, 1, 0,
               0,       0, 0, 1;

    Ra << cos(a), 0, sin(a), 0,
               0, 1,      0, 0,
         -sin(a), 0, cos(a), 0,
               0, 0,      0, 1;

    Rt << cos(t), -sin(t), 0, 0,
          sin(t),  cos(t), 0, 0,
               0,       0, 1, 0,
               0,       0, 0, 1;

    for(auto p : Pos_set){
        rz1 << cos(p.a/(180/M_PI)), -sin(p.a/(180/M_PI)), 0, 0,
               sin(p.a/(180/M_PI)),  cos(p.a/(180/M_PI)), 0, 0,
                                 0,                    0, 1, 0,
                                 0,                    0, 0, 1;

        ry << cos(p.b/(180/M_PI)), 0, sin(p.b/(180/M_PI)), 0,
                                0, 1,                   0, 0,
             -sin(p.b/(180/M_PI)), 0, cos(p.b/(180/M_PI)), 0,
                                0, 0,                   0, 1;

        rz2 << cos(p.c/(180/M_PI)), -sin(p.c/(180/M_PI)), 0, 0,
               sin(p.c/(180/M_PI)),  cos(p.c/(180/M_PI)), 0, 0,
                                 0,                    0, 1, 0,
                                 0,                    0, 0, 1;
        r = rz1*ry*rz2;
        r(0,3) = p.x;
        r(1,3) = p.y;
        r(2,3) = p.z;
        R.push_back(r);
    }
    for(auto &p : R)
        p = p*Rt.inverse()*Ra.inverse()*Ro.inverse()*Txyz.inverse()*Tz.inverse();

    double l, l2, l3, d, h;
    double a_, a2, a3, ad, ah;
    Eigen::Vector3d y4, z4, y4af, z4af, x6, y6, z6, z6af, z5;
    int i = 0;
    for(auto p : R){
        i++;
        agl.a1 = atan2(p(0,3), p(1,3));
        l2 = fabs(550*sin(agl.a1));
        l3 = fabs(sqrt(210*210+700*700)*sin(agl.a1));
        d = p(0,3) - 165*sin(agl.a1);
        h = p(2,3)*sin(agl.a1) - 430*sin(agl.a1);
        l = sqrt(d*d+h*h);
        a_ = atan2(210, 700);
        ah = atan(h/d);
        ad = M_PI/2 - ah;
        double t1 = (l*l+l3*l3-l2*l2)/(2*l*l3);
        double t2 = (l*l+l2*l2-l3*l3)/(2*l*l2);
        if(t1>1 || t1<-1 || t2>1 ||t2<-1){
            Limit = true;
            break;
        }
        a2 = acos(t1);
        a3 = acos(t2);
        agl.a2 = M_PI/2 - a3 - ah;
        agl.a3 = agl.a2 - a2 -ad - a_;
        z6 = {p(0,2), p(1,2), p(2,2)};
        y4 = {cos(agl.a1), -sin(agl.a1), 0};
        z4 = {cos(agl.a3)*sin(agl.a1)*sin(agl.a2) - cos(agl.a2)*sin(agl.a1)*sin(agl.a3),
             cos(agl.a1)*cos(agl.a3)*sin(agl.a2) - cos(agl.a1)*cos(agl.a2)*sin(agl.a3),
             cos(agl.a2)*cos(agl.a3) + sin(agl.a2)*sin(agl.a3)};
        y4af = z4.cross(z6);
        agl.a5 = acos(z4.dot(z6)/(sqrt(z4.dot(z4))*sqrt(z6.dot(z6))));

        if(agl_set.size() == 0){
            if(agl_acl.a5 < 0){
                y4af = -y4af;
                agl.a5 = -agl.a5;
            }
        }
        else{
            if(agl_set[agl_set.size()-1].a5 < 0){
                if(agl_set[agl_set.size()-1].a5 > -1){
                    double a41 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
                    z4af = y4.cross(y4af);
                    if(z4.dot(z4af) <0){
                        a41 = -a41;
                    }
                    y4af = -y4af;
                    double a42 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
                    z4af = y4.cross(y4af);
                    if(z4.dot(z4af) <0){
                        a42 = -a42;
                    }
                    double abs1 = fabs(a41*180/M_PI - agl_set[agl_set.size()-1].a4);
                    double abs2 = fabs(a42*180/M_PI - agl_set[agl_set.size()-1].a4);
                    y4af = -y4af;
                    if(abs1 > abs2){
                        y4af = -y4af;
                        agl.a5 = -agl.a5;
                    }
                }
                else{
                    y4af = -y4af;
                    agl.a5 = -agl.a5;
                }
            }
            else{
                if(agl_set[agl_set.size()-1].a5 < 1){
                    double a41 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
                    z4af = y4.cross(y4af);
                    if(z4.dot(z4af) <0){
                        a41 = -a41;
                    }
                    y4af = -y4af;
                    double a42 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
                    z4af = y4.cross(y4af);
                    if(z4.dot(z4af) <0){
                        a42 = -a42;
                    }
                    double abs1 = fabs(a41*180/M_PI - agl_set[agl_set.size()-1].a4);
                    double abs2 = fabs(a42*180/M_PI - agl_set[agl_set.size()-1].a4);
                    y4af = -y4af;
                    if(abs1 > abs2){
                        y4af = -y4af;
                        agl.a5 = -agl.a5;
                    }
                }
            }
        }

        agl.a4 = acos(y4.dot(y4af)/(sqrt(y4.dot(y4))*sqrt(y4af.dot(y4af))));
        z4af = y4.cross(y4af);
        if(z4.dot(z4af) <0){
            agl.a4 = -agl.a4;
        }

        y4 = {-cos(agl.a1)*cos(agl.a4)+(-cos(agl.a2)*cos(agl.a3)*sin(agl.a1)-sin(agl.a1)*sin(agl.a2)*sin(agl.a3))*sin(agl.a4),
              cos(agl.a4)*sin(agl.a1)+(-cos(agl.a1)*cos(agl.a2)*cos(agl.a3)-cos(agl.a1)*sin(agl.a2)*sin(agl.a3))*sin(agl.a4),
              (cos(agl.a3)*sin(agl.a2)-cos(agl.a2)*sin(agl.a3))*sin(agl.a4)};
        x6 = {-p(0,0), -p(1,0), -p(2,0)};
        agl.a6 = acos(y4.dot(x6)/(sqrt(y4.dot(y4))*sqrt(x6.dot(x6))));
        z6af = y4.cross(x6);
        if(z6.dot(z6af) < 0)
            agl.a6 = -agl.a6;
        agl.a1 = agl.a1*(180/M_PI);
        agl.a2 = agl.a2*(180/M_PI);
        agl.a3 = agl.a3*(180/M_PI);
        agl.a4 = agl.a4*(180/M_PI);
        agl.a5 = agl.a5*(180/M_PI);
        agl.a6 = agl.a6*(180/M_PI);

        if(agl_set.size() == 0){
            if(agl_acl.a4 > 150){
                if(agl.a4 < 0)
                    agl.a4 = 360 + agl.a4;
            }
            else if(agl_acl.a4 < -150){
                if(agl.a4 > 0)
                    agl.a4 = agl.a4 -360;
            }
        }
        else{
            if(agl_set[agl_set.size()-1].a4 > 150){
                if(agl.a4 < 0)
                    agl.a4 = 360 + agl.a4;
            }
            else if(agl_set[agl_set.size()-1].a4 < -150){
                if(agl.a4 > 0)
                    agl.a4 = agl.a4 -360;
            }
        }

        if(agl_set.size() == 0){
            if(agl_acl.a6 > 330){
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 + 360;
                if(agl.a6 < 0)
                    agl.a6 = agl.a6 + 360;
            }
            else if(agl_acl.a6 < -330){
                if(agl.a6 < 0)
                    agl.a6 = agl.a6 -360;
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 -360;
            }
            else if(agl_acl.a6 > 150){
                if(agl.a6 < 0)
                    agl.a6 = 360 + agl.a6;
            }
            else if(agl_acl.a6 < -150){
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 -360;
            }
        }
        else{
            if(agl_set[agl_set.size()-1].a6 > 330){
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 + 360;
                if(agl.a6 < 0)
                    agl.a6 = agl.a6 + 360;
            }
            else if(agl_set[agl_set.size()-1].a6 < -330){
                if(agl.a6 < 0)
                    agl.a6 = agl.a6 -360;
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 -360;
            }
            else if(agl_set[agl_set.size()-1].a6 > 150){
                if(agl.a6 < 0)
                    agl.a6 = 360 + agl.a6;
            }
            else if(agl_set[agl_set.size()-1].a6 < -150){
                if(agl.a6 > 0)
                    agl.a6 = agl.a6 -360;
            }
        }

        agl_set.push_back(agl);

        if(agl_set.size() > 1){
            if(agl_set[agl_set.size()-1].a4 < -180 || agl_set[agl_set.size()-1].a4 > 180){
                Limit = true;
                break;
            }
            if(agl_set[agl_set.size()-1].a6 < -360 || agl_set[agl_set.size()-1].a6 > 360){
                Limit = true;
                break;
            }

            if(agl_set[agl_set.size()-1].a1 >= 45 || agl_set[agl_set.size()-1].a1 <= -60){
                Limit = true;
                break;
            }
            if(agl_set[agl_set.size()-1].a6 >= 180 || agl_set[agl_set.size()-1].a6 <= -180){
                Limit = true;
                break;
            }
        }
    }

    return Limit;
}



void MainWindow::Read_File(vector<RobotPos> &Pos_set, QString filename, QString KeyWord)
{
    vector<QString> Qstr;
    QFile file1(filename);
    if (!file1.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream in(&file1);
    while (!in.atEnd()){
        QString line = in.readLine();
        Qstr.push_back(line);
    }

    bool sign = false;
    QStringList list;
    RobotPos pos;
    for(auto r : Qstr){
        if(r == KeyWord){
            sign = true;
            continue;
        }
        if(sign == true){
            if(r.length() == 0)
                break;
            list = r.split(' ');
            pos.x = list[9].toDouble();
            pos.y = list[10].toDouble();
            pos.z = list[11].toDouble();
            pos.a = list[12].toDouble();
            pos.b = list[13].toDouble();
            pos.c = list[14].toDouble();
            Pos_set.push_back(pos);
        }
    }
}


void MainWindow::Read_pathlst(vector<vector<RobotPos>> &PathLst, QString filename)
{
    vector<QString> Qstr;
    QFile file1(filename);
    if (!file1.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream in(&file1);
    while (!in.atEnd()){
        QString line = in.readLine();
        Qstr.push_back(line);
    }

    vector<RobotPos> poslst;
    RobotPos pos;
    QStringList list;
    for(int i=0; i<Qstr.size(); i++){
        list = Qstr[i].split(',');
        pos.x = list[1].toDouble();
        pos.y = list[2].toDouble();
        pos.z = list[3].toDouble();
        pos.a = list[4].toDouble();
        pos.b = list[5].toDouble();
        pos.c = list[6].toDouble();
        if(list[0] == "P0001" && poslst.size() != 0){
            PathLst.push_back(poslst);
            poslst.clear();
        }
        poslst.push_back(pos);
        if(i == Qstr.size()-1)
            PathLst.push_back(poslst);
    }
}


void MainWindow::Read_pathlst2(vector<vector<RobotPos>> &PathLst, QString filename)
{
    vector<QString> Qstr;
    QFile file1(filename);
    if (!file1.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream in(&file1);
    while (!in.atEnd()){
        QString line = in.readLine();
        Qstr.push_back(line);
    }

    vector<RobotPos> poslst;
    RobotPos pos;
    QStringList list;
    for(int i=0; i<Qstr.size(); i++){
        list = Qstr[i].split(' ');
        if(list.size() == 6){
            pos.x = list[0].toDouble();
            pos.y = list[1].toDouble();
            pos.z = list[2].toDouble();
            pos.a = list[3].toDouble();
            pos.b = list[4].toDouble();
            pos.c = list[5].toDouble();
            poslst.push_back(pos);
        }
        else if(i!=0){
            PathLst.push_back(poslst);
            poslst.clear();
        }
        if(i == Qstr.size()-1)
            PathLst.push_back(poslst);
    }
}


void MainWindow::PosRotate(RobotPos cpos, RobotPos &pos, double agl)
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


double MainWindow::Evaluation_function(vector<RobotAxle> agllst, vector<double> Operating_agl, vector<double> Walking_agl)
{
    double f = 0;
    vector<double> fi = {0,0,0,0,0,0,0,0};
    vector<double> ki = {1,1,1,1,1,1,1,1};
    for(int i=0; i<agllst.size()-1; i++){
        fi[0] += abs(agllst[i+1].a1 - agllst[i].a1);
        fi[1] += abs(agllst[i+1].a2 - agllst[i].a2);
        fi[2] += abs(agllst[i+1].a3 - agllst[i].a3);
        fi[3] += abs(agllst[i+1].a4 - agllst[i].a4);
        fi[4] += abs(agllst[i+1].a5 - agllst[i].a5);
        fi[5] += abs(agllst[i+1].a6 - agllst[i].a6);
    }
    for(int i=0; i<8; i++)
        f += ki[i]*fi[i];
    return f;
}


void MainWindow::Write_Filec(vector<vector<RobotPos>> pathLst, string filename)
{
    ofstream out(filename);
    if (out.is_open())
    {
        for(int i=0; i<(int)pathLst.size(); i++){
            out << i+1 <<  "\n";
            for(int j=0; j<(int)pathLst[i].size(); j++)
                out << pathLst[i][j].x << " " << pathLst[i][j].y << " " << pathLst[i][j].z << " "
                    << pathLst[i][j].a << " " << pathLst[i][j].b << " " << pathLst[i][j].c <<  "\n";
        }
    }
}


void MainWindow::Write_File(vector<vector<RobotPos>> pathLst, QString filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    for(int i=0; i<(int)pathLst.size(); i++){
        out << i+1 <<  "\n";
        for(int j=0; j<(int)pathLst[i].size(); j++)
            out << pathLst[i][j].x << " " << pathLst[i][j].y << " " << pathLst[i][j].z << " "
                << pathLst[i][j].a << " " << pathLst[i][j].b << " " << pathLst[i][j].c <<  "\n";
    }
}


void MainWindow::Write_File2(vector<vector<vector<RobotPos>>> pathLst_s, vector<vector<double>> platform, QString filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    vector<vector<RobotPos>> pathLst;
    for(int i=0; i<(int)pathLst_s.size(); i++){
        pathLst = pathLst_s[i];
        out << platform[i][0] << " " << platform[i][1] << " " << platform[i][2] << " "
            << platform[i][3] << " " << platform[i][4] << "\n";
        for(int j=0; j<(int)pathLst.size(); j++){
            for(int k=0; k<(int)pathLst[j].size(); k++)
                out << pathLst[j][k].x << " " << pathLst[j][k].y << " " << pathLst[j][k].z << " "
                    << pathLst[j][k].a << " " << pathLst[j][k].b << " " << pathLst[j][k].c <<  "\n";
            out << "\n";
        }
        out << "\n";
    }
}


void MainWindow::Write_File3(vector<vector<vector<RobotPos>>> pathLst_s,
                             vector<vector<vector<RobotPos>>> pathLst_s2,
                             vector<vector<vector<RobotAxle>>> AglLst_ss,
                             vector<vector<double>> platform,
                             QString filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    vector<vector<RobotPos>> pathLst, pathLst2;
    vector<vector<RobotAxle>> AglLst_s;
    for(int i=0; i<(int)pathLst_s.size(); i++){
        pathLst = pathLst_s[i];
        pathLst2 = pathLst_s2[i];
        AglLst_s = AglLst_ss[i];

//        out << "焊缝号：" << i << " " << "机器人位置：" << platform[i][0] << " " << "翻转轴角度：" << platform[i][4] << " "
//            << "旋转轴角度：" << platform[i][3] << " " << "评分：" << 0 << "\n";

        out << i << " " << platform[i][0] << " " << platform[i][4] << " "
            << platform[i][3] << " " << "\n";

//        out << "焊缝轨迹（工件坐标系）：" << "\n";
        for(int j=0; j<(int)pathLst.size(); j++){
            for(int k=0; k<(int)pathLst[j].size(); k++)
                out << pathLst[j][k].x << " " << pathLst[j][k].y << " " << pathLst[j][k].z << " "
                    << pathLst[j][k].a << " " << pathLst[j][k].b << " " << pathLst[j][k].c <<  "\n";
            out << "\n";
        }

//        out << "机器人POSE（base坐标系）：" << "\n";
        for(int j=0; j<(int)pathLst2.size(); j++){
            for(int k=0; k<(int)pathLst2[j].size(); k++)
                out << pathLst2[j][k].x << " " << pathLst2[j][k].y << " " << pathLst2[j][k].z << " "
                    << pathLst2[j][k].a << " " << pathLst2[j][k].b << " " << pathLst2[j][k].c <<  "\n";
            out << "\n";
        }

//        out << "机器人6个轴角度：" << "\n";
        for(int j=0; j<(int)AglLst_s.size(); j++){
            for(int k=0; k<(int)AglLst_s[j].size(); k++)
                out << AglLst_s[j][k].a1 << " " << AglLst_s[j][k].a2 << " " << AglLst_s[j][k].a3 << " "
                    << AglLst_s[j][k].a4 << " " << AglLst_s[j][k].a5 << " " << AglLst_s[j][k].a6 <<  "\n";
            out << "\n";
        }
        out << "\n";
        out << "\n";
    }
}


void MainWindow::WeldClassify7(vector<vector<vector<RobotPos>>> pathlst_s, vector<vector<double>> region, vector<vector<int>> &WeldClassifyPlan)
{
    double dx, dy, dz, da, db, Xnum, Ynum, Znum, Anum, Bnum, fun, fmin;
    Xnum = region[0][0];
    Ynum = region[1][0];
    Znum = region[2][0];
    Anum = region[3][0];
    Bnum = region[4][0];
    dx = (region[0][2]-region[0][1])/Xnum;
    dy = (region[1][2]-region[1][1])/Ynum;
    dz = (region[2][2]-region[2][1])/Znum;
    da = (region[3][2]-region[3][1])/Anum;
    db = (region[4][2]-region[4][1])/Bnum;
    bool limit1, limit2, pass;
    vector<int> State, States;
    vector<vector<int>> StateLst;
    vector<double> Evaluate;
    vector<vector<double>> EvaluateLst;
    vector<vector<vector<RobotPos>>> pathlst_s_correct;
    vector<RobotAxle> AglLst1, AglLst2;
    int posNum; //轨迹点数量
    // 每一个状态下(x,y,z,a,b)，所有焊缝的限位情况
    for(int i=0; i<Xnum; i++){
        for(int j=0; j<Ynum; j++){
            for(int k=0; k<Znum; k++){
                for(int m=0; m<Anum; m++){
                    for(int n=0; n<Bnum; n++){
                        pathlst_s_correct = pathlst_s;
                        for(int t1=0; t1<pathlst_s_correct.size(); t1++){
                            for(int t2=0; t2<pathlst_s_correct[t1].size(); t2++){
                                for(auto &r : pathlst_s_correct[t1][t2]){
                                    PosRotate(cposa, r, region[3][1]+m*da);
                                    PosRotate(cposb, r, region[4][1]+n*db);
                                    r.x -= (region[0][1]+i*dx);
                                    r.y -= (region[1][1]+j*dy);
                                    r.z -= (region[2][1]+k*dz);
                                }
                            }
                        }
                        State.push_back(i);
                        State.push_back(j);
                        State.push_back(k);
                        State.push_back(m);
                        State.push_back(n);
                        Evaluate.push_back(i);
                        Evaluate.push_back(j);
                        Evaluate.push_back(k);
                        Evaluate.push_back(m);
                        Evaluate.push_back(n);
                        for(int t1=0; t1<pathlst_s_correct.size(); t1++)
                        {
                            pass = true;
                            fun = 0;
                            posNum = 0;
                            for(int t2=0; t2<pathlst_s_correct[t1].size(); t2++)
                            {
                                limit1 = Pos2Joint2(pathlst_s_correct[t1][t2], AglLst1, agl_acl, TInF);
                                limit2 = Pos2Joint2(pathlst_s_correct[t1][t2], AglLst2, agl_acl, CInF);
                                if(!limit1)
                                    fun += RobotAttitudEvaluate2(AglLst1, pathlst_s_correct[t1][t2], aglBest);
                                if(!limit2)
                                    fun += RobotAttitudEvaluate2(AglLst2, pathlst_s_correct[t1][t2], aglBest);
                                posNum += 2*pathlst_s_correct[t1][t2].size();
                                AglLst1.clear();
                                AglLst2.clear();
                                if(limit1 || limit2)
                                    pass = false;
                                if(!pass)
                                {
                                    fun = 1e10;
                                    break;
                                }
                            }
                            State.push_back(int(pass));
                            fun = fun/posNum;
                            Evaluate.push_back(fun);
                        }
                        pathlst_s_correct.clear();
                        StateLst.push_back(State);
                        EvaluateLst.push_back(Evaluate);
                        State.clear();
                        Evaluate.clear();
                    }
                }
            }
        }
    }

    // 判断总共有多少条焊缝可以一次性焊接完成
    int SureWeldNum, SureWeldNumi,  SureWeldNumMax;
    vector<int> SureState;
    SureState = StateLst[0];
    for(int i=0; i<(int)StateLst.size(); i++){
        State = StateLst[i];
        for(int j=5; j<(int)State.size(); j++)
            if(SureState[j] || State[j])
                SureState[j] = 1;
    }
    SureWeldNum = 0;
    for(int i=5; i<(int)SureState.size(); i++)
        SureWeldNum += SureState[i];

    // 筛选出最适当的焊接姿态并保存
    int index;
    vector<int> SelectState;
    for(int i=5; i<(int)SureState.size(); i++)
    {
        if(SureState[i])
        {
            fmin = 1e10;
            for(int j=0; j<(int)EvaluateLst.size(); j++)
            {
                if(EvaluateLst[j][i]<fmin){
                    fmin = EvaluateLst[j][i];
                    index = j;
                }
            }
            SelectState = StateLst[index];
            for(int j=5; j<(int)SelectState.size(); j++)
                if(j != i)
                    SelectState[j] = 0;
            WeldClassifyPlan.push_back(SelectState);
        }
    }
}


void MainWindow::WeldClassify8(vector<vector<vector<RobotPos>>> pathlst_s,
                               vector<vector<double>> region,
                               vector<vector<int>> &WeldClassifyPlan,
                               vector<vector<int>> &laserDirection,
                               vector<vector<int>> rotateY)
{
    double dx, dy, dz, da, db, Xnum, Ynum, Znum, Anum, Bnum, fun, fmin;
    Xnum = region[0][0];
    Ynum = region[1][0];
    Znum = region[2][0];
    Anum = region[3][0];
    Bnum = region[4][0];
    dx = (region[0][2]-region[0][1])/Xnum;
    dy = (region[1][2]-region[1][1])/Ynum;
    dz = (region[2][2]-region[2][1])/Znum;
    da = (region[3][2]-region[3][1])/Anum;
    db = (region[4][2]-region[4][1])/Bnum;
    bool limit1, limit2, pass;
    vector<int> State;
    vector<vector<int>> StateLst;
    vector<double> Evaluate;
    vector<vector<double>> EvaluateLst;
    vector<vector<vector<RobotPos>>> pathlst_s_correct;
    vector<RobotAxle> AglLst1, AglLst2;
    vector<vector<int>> laserDirection2 = laserDirection;
    vector<vector<vector<int>>> laserDirectionLst;
    RobotPos posS, posE;
    int posNum;
    double dmax, dmin;
    // 每一个状态下(x,y,z,a,b)，所有焊缝的限位情况
    for(int i=0; i<Xnum; i++){
        for(int j=0; j<Ynum; j++){
            for(int k=0; k<Znum; k++){
                for(int m=0; m<Anum; m++){
                    for(int n=0; n<Bnum; n++){
                        pathlst_s_correct = pathlst_s;
                        for(int t1=0; t1<pathlst_s_correct.size(); t1++){
                            for(int t2=0; t2<pathlst_s_correct[t1].size(); t2++){
                                if(laserDirection2[t1][t2]==0)
                                {
                                    laserDirection[t1][t2] = 0;
                                    posRotateY(pathlst_s_correct[t1][t2], 32);
                                    if(rotateY[t1][t2]==1)
                                        pathEndRotate(pathlst_s_correct[t1][t2],15,5,1);
                                }
                                else
                                {
                                    posS = pathlst_s_correct[t1][t2][0];
                                    posE = pathlst_s_correct[t1][t2][pathlst_s_correct[t1][t2].size()-1];
                                    dmin = pow(posS.x,2) + pow(posS.y,2);
                                    dmax = pow(posE.x,2) + pow(posE.y,2);
                                    if(dmax>dmin)
                                    {
                                        laserDirection[t1][t2] = 0;
                                        posRotateY(pathlst_s_correct[t1][t2], 32);
                                        if(rotateY[t1][t2]==1)
                                            pathEndRotate(pathlst_s_correct[t1][t2],15,5,1);
                                    }
                                    else
                                    {
                                        laserDirection[t1][t2] = 1;
                                        laserReverse(pathlst_s_correct[t1][t2]);
                                        if(rotateY[t1][t2]==1)
                                            pathEndRotate(pathlst_s_correct[t1][t2],15,5,0);
                                    }
                                }
                                for(auto &r : pathlst_s_correct[t1][t2]){
                                    PosRotate(cposa, r, region[3][1]+m*da);
                                    PosRotate(cposb, r, region[4][1]+n*db);
                                    r.x -= (region[0][1]+i*dx);
                                    r.y -= (region[1][1]+j*dy);
                                    r.z -= (region[2][1]+k*dz);
                                }
                            }
                        }
                        State.push_back(i);
                        State.push_back(j);
                        State.push_back(k);
                        State.push_back(m);
                        State.push_back(n);
                        Evaluate.push_back(i);
                        Evaluate.push_back(j);
                        Evaluate.push_back(k);
                        Evaluate.push_back(m);
                        Evaluate.push_back(n);
                        for(int t1=0; t1<pathlst_s_correct.size(); t1++)
                        {
                            pass = true;
                            fun = 0;
                            posNum = 0;
                            for(int t2=0; t2<pathlst_s_correct[t1].size(); t2++)
                            {
                                limit1 = Pos2Joint2(pathlst_s_correct[t1][t2], AglLst1, agl_acl, TInF);
                                limit2 = Pos2Joint2(pathlst_s_correct[t1][t2], AglLst2, agl_acl, CInF);
                                if(!limit1)
                                    fun += RobotAttitudEvaluate2(AglLst1, pathlst_s_correct[t1][t2], aglBest);
                                if(!limit2)
                                    fun += RobotAttitudEvaluate2(AglLst2, pathlst_s_correct[t1][t2], aglBest);
                                posNum += 2*pathlst_s_correct[t1][t2].size();
                                AglLst1.clear();
                                AglLst2.clear();
                                if(limit1 || limit2)
                                    pass = false;
                                if(!pass)
                                {
                                    fun = 1e10;
                                    break;
                                }
                            }
                            State.push_back(int(pass));
                            fun = fun/posNum;
                            Evaluate.push_back(fun);
                        }
                        pathlst_s_correct.clear();
                        StateLst.push_back(State);
                        EvaluateLst.push_back(Evaluate);
                        State.clear();
                        Evaluate.clear();
                        laserDirectionLst.push_back(laserDirection);
                    }
                }
            }
        }
    }

    // 判断总共有多少条焊缝可以一次性焊接完成
    int SureWeldNum;
    vector<int> SureState;
    SureState = StateLst[0];
    for(int i=0; i<(int)StateLst.size(); i++){
        State = StateLst[i];
        for(int j=5; j<(int)State.size(); j++)
            if(SureState[j] || State[j])
                SureState[j] = 1;
    }
    SureWeldNum = 0;
    for(int i=5; i<(int)SureState.size(); i++)
        SureWeldNum += SureState[i];

    // 筛选出最适当的焊接姿态并保存
    int index;
    vector<int> SelectState;
    for(int i=5; i<(int)SureState.size(); i++)
    {
        if(SureState[i])
        {
            fmin = 1e10;
            for(int j=0; j<(int)EvaluateLst.size(); j++)
            {
                if(EvaluateLst[j][i]<fmin){
                    fmin = EvaluateLst[j][i];
                    index = j;
                }
            }
            laserDirection[i-5] = laserDirectionLst[index][i-5];
            SelectState = StateLst[index];
            for(int j=5; j<(int)SelectState.size(); j++)
                if(j != i)
                    SelectState[j] = 0;
            WeldClassifyPlan.push_back(SelectState);
        }
    }
}


// 焊缝由焊接方向及激光方向得出相应的轨迹数据
void MainWindow::Weld2Path(vector<RobotPos> &path, vector<int> direction)
{
    if(direction[0]==1){
        pathReverse(path);
        laserReverse(path);
    }

    if(direction[1]==1){
        laserReverse(path);
        posRotateY(path, aglInclude-aglWeld);
    }
    else{
        posRotateY(path, aglInclude+aglWeld);
    }

    if(direction[2]==1){
        if(direction[1]==1){
            pathHeadRotate(path,aglWeld-aglHead,5,1);
        }
        else{
            pathHeadRotate(path,aglWeld-aglHead,5,0);
        }
    }

    if(direction[3]==1){
        if(direction[1]==1){
            pathEndRotate(path,aglEnd-aglWeld,5,0);
        }
        else{
            pathEndRotate(path,aglEnd-aglWeld,5,1);
        }
    }
}


// 同条焊缝由焊接方向及激光方向得出若干组轨迹数据
void MainWindow::pathMode(vector<RobotPos> path, vector<vector<RobotPos>> &pathlst, vector<int> direction, vector<vector<int>> &directionLst)
{
    vector<RobotPos> path2 = path;
    vector<int> direction2 = direction;
    if(direction[0]==0 && direction[1]==0)
    {
        Weld2Path(path2, direction2);
        pathlst.push_back(path2);
        directionLst.push_back(direction2);
    }

    if(direction[0]==0 && direction[1]==1)
    {
        Weld2Path(path2, direction2);
        pathlst.push_back(path2);
        directionLst.push_back(direction2);
        path2 = path;
        direction2 = direction;
        direction2[1] = 0;
        Weld2Path(path2, direction2);
        pathlst.push_back(path2);
        directionLst.push_back(direction2);
    }

    if(direction[0]==1 && direction[1]==0)
    {
        Weld2Path(path2, direction2);
        pathlst.push_back(path2);
        directionLst.push_back(direction2);
        path2 = path;
        direction2 = direction;
        direction2[0] = 0;
        Weld2Path(path2, direction2);
        pathlst.push_back(path2);
        directionLst.push_back(direction2);
    }

    if(direction[0]==1 && direction[1]==1)
    {
        Weld2Path(path2, direction2);
        pathlst.push_back(path2);
        directionLst.push_back(direction2);
        path2 = path;
        direction2 = direction;
        direction2[0] = 0;
        Weld2Path(path2, direction2);
        pathlst.push_back(path2);
        directionLst.push_back(direction2);
        path2 = path;
        direction2 = direction;
        direction2[1] = 0;
        Weld2Path(path2, direction2);
        pathlst.push_back(path2);
        directionLst.push_back(direction2);
        path2 = path;
        direction2 = direction;
        direction2[0] = 0;
        direction2[1] = 0;
        Weld2Path(path2, direction2);
        pathlst.push_back(path2);
        directionLst.push_back(direction2);
    }
}


// 对若干组轨迹数据进行评价并给出最好的结果
bool MainWindow::pathModeTest(vector<vector<RobotPos>> pathlst, vector<vector<int>> directionLst, vector<int> &direction, double &evaluate)
{
    double fun, posNum;
    vector<double> flst;
    bool limit1, limit2, pass;
    vector<RobotAxle> AglLst1, AglLst2;
    pass = false;
    for(int i=0; i<pathlst.size(); i++)
    {
        fun = 0;
        posNum = 0;
        limit1 = Pos2Joint2(pathlst[i], AglLst1, agl_acl, TInF);
        limit2 = Pos2Joint2(pathlst[i], AglLst2, agl_acl, CInF);
        if(!limit1)
            fun += RobotAttitudEvaluate2(AglLst1, pathlst[i], aglBest);
        if(!limit2)
            fun += RobotAttitudEvaluate2(AglLst2, pathlst[i], aglBest);
        posNum += 2*pathlst[i].size();
        fun = fun/posNum;
        AglLst1.clear();
        AglLst2.clear();
        if(!limit1 && !limit2)
            pass = true;
        else
            fun = 1e10;
        flst.push_back(fun);
    }
    evaluate = 1e10;
    if(pass)
    {
        for(int i=0; i<pathlst.size(); i++)
        {
            if(evaluate>flst[i]){
                evaluate = flst[i];
                direction = directionLst[i];
            }
        }
    }
    return pass;
}


void MainWindow::WeldClassify9(vector<vector<vector<RobotPos>>> pathlst_s,
                               vector<vector<double>> region,
                               vector<vector<int>> &WeldClassifyPlan,
                               vector<vector<vector<int>>> &directionLst_s,
                               vector<vector<int>> pathDirection,
                               vector<vector<int>> laserDirection,
                               vector<vector<int>> rotateYHead,
                               vector<vector<int>> rotateYEnd)
{
    double dx, dy, dz, da, db, Xnum, Ynum, Znum, Anum, Bnum, fun, fmin;
    Xnum = region[0][0];
    Ynum = region[1][0];
    Znum = region[2][0];
    Anum = region[3][0];
    Bnum = region[4][0];
    dx = (region[0][2]-region[0][1])/Xnum;
    dy = (region[1][2]-region[1][1])/Ynum;
    dz = (region[2][2]-region[2][1])/Znum;
    da = (region[3][2]-region[3][1])/Anum;
    db = (region[4][2]-region[4][1])/Bnum;
    bool limit, pass;
    vector<int> State;
    vector<vector<int>> StateLst;
    vector<double> Evaluate;
    vector<vector<double>> EvaluateLst;
    vector<vector<vector<RobotPos>>> pathlst_s_correct;
    int posNum;
    vector<vector<RobotPos>> pathlst;
    vector<int> direction;
    vector<vector<int>> directionLst, dirLst;
    vector<vector<vector<int>>> dirLst_s;
    vector<vector<vector<vector<int>>>> dirLst_ss;
    double evaluate;
    // 每一个状态下(x,y,z,a,b)，所有焊缝的限位情况
    for(int i=0; i<Xnum; i++){
        for(int j=0; j<Ynum; j++){
            for(int k=0; k<Znum; k++){
                for(int m=0; m<Anum; m++){
                    for(int n=0; n<Bnum; n++){
                        pathlst_s_correct = pathlst_s;
                        for(int t1=0; t1<pathlst_s_correct.size(); t1++){
                            for(int t2=0; t2<pathlst_s_correct[t1].size(); t2++){
                                for(auto &r : pathlst_s_correct[t1][t2]){
                                    PosRotate(cposa, r, region[3][1]+m*da);
                                    PosRotate(cposb, r, region[4][1]+n*db);
                                    r.x -= (region[0][1]+i*dx);
                                    r.y -= (region[1][1]+j*dy);
                                    r.z -= (region[2][1]+k*dz);
                                }
                            }
                        }
                        State.push_back(i);
                        State.push_back(j);
                        State.push_back(k);
                        State.push_back(m);
                        State.push_back(n);
                        Evaluate.push_back(i);
                        Evaluate.push_back(j);
                        Evaluate.push_back(k);
                        Evaluate.push_back(m);
                        Evaluate.push_back(n);
                        for(int t1=0; t1<pathlst_s_correct.size(); t1++)
                        {
                            pass = true;
                            fun = 0;
                            posNum = 0;
                            for(int t2=0; t2<pathlst_s_correct[t1].size(); t2++)
                            {
                                direction.push_back(pathDirection[t1][t2]);
                                direction.push_back(laserDirection[t1][t2]);
                                direction.push_back(rotateYHead[t1][t2]);
                                direction.push_back(rotateYEnd[t1][t2]);
                                pathMode(pathlst_s_correct[t1][t2], pathlst, direction, directionLst);
                                limit = pathModeTest(pathlst, directionLst, direction, evaluate);
                                fun += evaluate;
                                dirLst.push_back(direction);
                                direction.clear();
                                directionLst.clear();
                                pathlst.clear();
                                if(!limit)
                                {
                                    pass = false;
                                    fun = 1e10;
                                    break;
                                }
                            }
                            State.push_back(int(pass));
                            Evaluate.push_back(fun);
                            dirLst_s.push_back(dirLst);
                            dirLst.clear();
                        }
                        pathlst_s_correct.clear();
                        StateLst.push_back(State);
                        EvaluateLst.push_back(Evaluate);
                        State.clear();
                        Evaluate.clear();
                        dirLst_ss.push_back(dirLst_s);
                        dirLst_s.clear();
                    }
                }
            }
        }
    }

    // 判断总共有多少条焊缝可以一次性焊接完成
    int SureWeldNum;
    vector<int> SureState;
    SureState = StateLst[0];
    for(int i=0; i<(int)StateLst.size(); i++){
        State = StateLst[i];
        for(int j=5; j<(int)State.size(); j++)
            if(SureState[j] || State[j])
                SureState[j] = 1;
    }
    SureWeldNum = 0;
    for(int i=5; i<(int)SureState.size(); i++)
        SureWeldNum += SureState[i];

    // 筛选出最适当的焊接姿态并保存
    int index;
    vector<int> SelectState;
    for(int i=5; i<(int)SureState.size(); i++)
    {
        if(SureState[i])
        {
            fmin = 1e10;
            for(int j=0; j<(int)EvaluateLst.size(); j++)
            {
                if(EvaluateLst[j][i]<fmin){
                    fmin = EvaluateLst[j][i];
                    index = j;
                }
            }
            directionLst_s[i-5] = dirLst_ss[index][i-5];
            SelectState = StateLst[index];
            for(int j=5; j<(int)SelectState.size(); j++)
                if(j != i)
                    SelectState[j] = 0;
            WeldClassifyPlan.push_back(SelectState);
        }
    }
}


// 船型焊
void MainWindow::ShipForm_Weld(vector<vector<RobotPos>> pathlst, vector<vector<double>> region, vector<double> &StateWithSpeed, double &weldTime, vector<int> Variables, double WeldSpeed, int pso_Num, int time, int splitNum)
{
    vector<double> x, v;
    vector<vector<double>> pso_x, pso_v;
    vector<double> pso_f;
    vector<vector<double>> pso_pbestx;
    vector<double> pso_pbestf, pso_gbestx;
    double pso_gbestf;
//    vector<double> xmax = {-400,0,0,180,-42,   0,0,0,1,0}; // 机器人变位机姿态及姿态变化率
//    vector<double> xmin = {-400,0,0,-180,-42,   0,0,0,-1,0};
//    vector<double> xmax = {-400,0,0,-130,-42,   0,0,0,-0.4,0}; // 机器人变位机姿态及姿态变化率
//    vector<double> xmin = {-400,0,0,-150,-42,   0,0,0,-0.6,0};
    vector<double> xmax = {region[0][2],region[1][2],region[2][2],region[3][2],region[4][2],   0,0,0,0.6,0}; // 机器人变位机姿态及姿态变化率
    vector<double> xmin = {region[0][1],region[1][1],region[2][1],region[3][1],region[4][1],   0,0,0,0.4,0};
    vector<double> vmax = {10,0,0,10,1,    0,0,0,1,0};
    vector<double> vmin = {10,0,0,-10,1,   0,0,0,-1,0};
    vector<double> pso_standard = {0,0,0,0,0, 0,0,0,0,0};
    for(int i=0; i<pso_Num; i++){
        if(i == 0){
            for(int j=0; j<10; j++){
                x.push_back(0);
                v.push_back(0);
            }
        }
        else{
            for(int j=0; j<10; j++){
                x.push_back((rand()/double(RAND_MAX))*(xmax[j]-xmin[j]) + xmin[j]);
                v.push_back((rand()/double(RAND_MAX))*(vmax[j]-vmin[j]) + vmin[j]);
            }
        }
        pso_x.push_back(x);
        pso_v.push_back(v);
        pso_pbestx.push_back(x);
        pso_f.push_back(1e10);
        pso_pbestf.push_back(1e10);
        x.clear();
        v.clear();
    }
    for(int j=0; j<10; j++)
        pso_gbestx.push_back(0);
    pso_gbestf = 1e10;

    double c = 0.5;
    double c1 = 0.5;
    double c2 = 0.5;
    vector<RobotPos> PosLst, PosLst2;
    if(pathlst.size()==1)
        PosLst = pathlst[0];
    else
    {
        if(pathlst[0].size()>pathlst[1].size()){
            PosLst = pathlst[0];
            PosLst2 = pathlst[1];
        }
        else{
            PosLst = pathlst[1];
            PosLst2 = pathlst[0];
        }
    }
    for(int t=0; t<time; t++)
    {
        for(int i=0; i<pso_Num; i++)
        {
            for(int j=0; j<10; j++){
                if(Variables[j]== 1){
                    pso_v[i][j] = c*pso_v[i][j] + c1*(rand()/double(RAND_MAX))*(pso_pbestx[i][j]-pso_x[i][j])
                            + c2*(rand()/double(RAND_MAX))*(pso_gbestx[j]-pso_x[i][j]);
                    if(pso_v[i][j] > vmax[j])
                        pso_v[i][j] = vmax[j];
                    if(pso_v[i][j] < vmin[j])
                        pso_v[i][j] = vmin[j];
                    pso_x[i][j] += pso_v[i][j];
                    if(pso_x[i][j] > xmax[j])
                        pso_x[i][j] = xmax[j];
                    if(pso_x[i][j] < xmin[j])
                        pso_x[i][j] = xmin[j];
                }
                else
                    pso_x[i][j] = pso_standard[j];
            }
        }
        RobotPos pos;
        vector<RobotPos> PosLst_correct, PosLst_correct2;
        vector<RobotAxle> AglLst, AglLst1, AglLst2;
        bool limit, limit1, limit2;
        double timegap = sqrt(pow(PosLst[0].x-PosLst[1].x,2)
                     +pow(PosLst[0].y-PosLst[1].y,2)
                     +pow(PosLst[0].z-PosLst[1].z,2))/WeldSpeed;
        weldTime = timegap*splitNum;
        int jj = 0;
        for(int i=0; i<pso_Num; i++)
        {
            for(int j=0; j<PosLst.size(); j++)
            {
                pos = PosLst[j];
                if(j>splitNum)
                    jj = splitNum;
                else
                    jj = j;
                PosRotate(cposa, pos, pso_x[i][3]+jj*timegap*pso_x[i][8]);
                PosRotate(cposb, pos, pso_x[i][4]+jj*timegap*pso_x[i][9]);
                pos.x -= pso_x[i][0]+jj*timegap*pso_x[i][5];
                pos.y -= pso_x[i][1]+jj*timegap*pso_x[i][6];
                pos.z -= pso_x[i][2]+jj*timegap*pso_x[i][7];
                PosLst_correct.push_back(pos);
            }
            if(pathlst.size()>1)
            {
                for(int j=0; j<PosLst2.size(); j++){
                    pos = PosLst2[j];
                    PosRotate(cposa, pos, pso_x[i][3]+splitNum*timegap*pso_x[i][8]);
                    PosRotate(cposb, pos, pso_x[i][4]+splitNum*timegap*pso_x[i][9]);
                    pos.x -= pso_x[i][0]+splitNum*timegap*pso_x[i][5];
                    pos.y -= pso_x[i][1]+splitNum*timegap*pso_x[i][6];
                    pos.z -= pso_x[i][2]+splitNum*timegap*pso_x[i][7];
                    PosLst_correct2.push_back(pos);
                }
            }
            if(pathlst.size()==1)
            {
                limit = Pos2Joint(PosLst_correct, AglLst, agl_acl, TInF);
                if(limit)
                    pso_f[i] = 1e10;
                else
                {
                    pso_f[i] = 0;
                    for(int j=0; j<PosLst_correct.size(); j++)
                        pso_f[i] += 180-abs(PosLst_correct[j].b);
                }
            }
            else
            {
                limit1 = Pos2Joint(PosLst_correct, AglLst1, agl_acl, TInF);
                limit2 = Pos2Joint(PosLst_correct2, AglLst2, agl_acl, TInF);
                if(limit1 || limit2)
                    pso_f[i] = 1e10;
                else
                {
                    pso_f[i] = 0;
                    for(int j=0; j<PosLst_correct.size(); j++)
                        pso_f[i] += 180-abs(PosLst_correct[j].b);
                }
            }
            PosLst_correct.clear();
            PosLst_correct2.clear();
            AglLst.clear();
            AglLst1.clear();
            AglLst2.clear();
        }

        for(int i=0; i<pso_Num; i++){
            if(pso_f[i] < pso_pbestf[i]){
                pso_pbestf[i] = pso_f[i];
                pso_pbestx[i] = pso_x[i];
            }
        }
        for(int i=0; i<pso_Num; i++){
            if(pso_pbestf[i] < pso_gbestf){
                pso_gbestf = pso_pbestf[i];
                pso_gbestx = pso_pbestx[i];
            }
        }
    }
    StateWithSpeed = pso_gbestx;
}


void MainWindow::ShipForm_Weld2(vector<vector<RobotPos>> pathlst, vector<vector<double>> region, vector<double> &StateWithSpeed, double &weldTime, vector<int> Variables, double WeldSpeed, int pso_Num, int time, int splitNum)
{
    vector<double> x, v;
    vector<vector<double>> pso_x, pso_v;
    vector<double> pso_f, pso_f2;
    vector<vector<double>> pso_pbestx;
    vector<double> pso_pbestf, pso_gbestx;
    double pso_gbestf, pso_gbestf2;
    double Px, Pa, Pb, Vx, Va, Vb, dpx, dpa, dpb, dvx, dva, dvb;
    Px = region[0][0];
    Pa = region[3][0];
    Pb = region[4][0];
    Vx = region[5][0];
    Va = region[8][0];
    Vb = region[9][0];
    dpx = (region[0][2]-region[0][1])/Px;
    dpa = (region[3][2]-region[3][1])/Pa;
    dpb = (region[4][2]-region[4][1])/Pb;
    dvx = (region[5][2]-region[5][1])/Vx;
    dva = (region[8][2]-region[8][1])/Va;
    dvb = (region[9][2]-region[9][1])/Vb;
    for(int i=0; i<Px; i++){
        for(int j=0; j<Pa; j++){
            for(int k=0; k<Pb; k++){
                for(int m=0; m<Vx; m++){
                    for(int n=0; n<Va; n++){
                        for(int nn=0; nn<Vb; nn++){
                           x.push_back(region[0][1]+i*dpx);
                           x.push_back(region[3][1]+j*dpa);
                           x.push_back(region[4][1]+k*dpb);
                           x.push_back(region[5][1]+m*dvx);
                           x.push_back(region[8][1]+n*dva);
                           x.push_back(region[9][1]+nn*dvb);
                           v = {0,0,0,0,0,0};
                           pso_x.push_back(x);
                           pso_v.push_back(v);
                           pso_pbestx.push_back(x);
                           pso_f.push_back(1e10);
                           pso_f2.push_back(1e10);
                           pso_pbestf.push_back(1e10);
                           x.clear();
                           v.clear();
                        }
                    }
                }
            }
        }
    }
    for(int j=0; j<10; j++)
        pso_gbestx.push_back(0);
    pso_gbestf = 1e10;
    pso_gbestf2 = 1e10;

    double c = 0.5;
    double c1 = 0.5;
    double c2 = 0.5;
    vector<RobotPos> PosLst, PosLst2;
    int index;
    vector<int> indexLst;
    if(pathlst.size()==1)
        PosLst = pathlst[0];
    else
    {
        if(pathlst[0].size()>pathlst[1].size()){
            PosLst = pathlst[0];
            PosLst2 = pathlst[1];
        }
        else{
            PosLst = pathlst[1];
            PosLst2 = pathlst[0];
        }
    }
    for(int t=0; t<1; t++)
    {
        RobotPos pos;
        vector<RobotPos> PosLst_correct, PosLst_correct2;
        vector<RobotAxle> AglLst, AglLst2;
        bool limit, limit1, limit2;
        double timegap = sqrt(pow(PosLst[0].x-PosLst[1].x,2)
                     +pow(PosLst[0].y-PosLst[1].y,2)
                     +pow(PosLst[0].z-PosLst[1].z,2))/WeldSpeed;
        weldTime = timegap*splitNum;
        int jj = 0;
        pso_Num = pso_x.size();
        for(int i=0; i<pso_Num; i++)
        {
            for(int j=0; j<PosLst.size(); j++)
            {
                pos = PosLst[j];
                if(j>splitNum)
                    jj = splitNum;
                else
                    jj = j;
                PosRotate(cposa, pos, pso_x[i][1]+jj*timegap*pso_x[i][4]);
                PosRotate(cposb, pos, pso_x[i][2]+jj*timegap*pso_x[i][5]);
                pos.x -= pso_x[i][0]+jj*timegap*pso_x[i][3];
                PosLst_correct.push_back(pos);
            }
            if(pathlst.size()>1)
            {
                for(int j=0; j<PosLst2.size(); j++){
                    pos = PosLst2[j];
                    PosRotate(cposa, pos, pso_x[i][1]+splitNum*timegap*pso_x[i][4]);
                    PosRotate(cposb, pos, pso_x[i][2]+splitNum*timegap*pso_x[i][5]);
                    pos.x -= pso_x[i][0]+splitNum*timegap*pso_x[i][3];
                    PosLst_correct2.push_back(pos);
                }
            }
            if(pathlst.size()==1)
            {
                limit = Pos2Joint(PosLst_correct, AglLst, agl_acl, TInF);
                if(limit)
                {
                    pso_f[i] = 1e10;
                    pso_f2[i] = 1e10;
                }
                else
                {
                    pso_f[i] = 0;
                    pso_f2[i] = 0;
                    for(int j=0; j<PosLst_correct.size(); j++)
                        pso_f[i] += 180-abs(PosLst_correct[j].b);
                    pso_f2[i] += RobotAttitudEvaluate2(AglLst, PosLst_correct, aglBest);
                    pso_f2[i] = pso_f2[i]/PosLst_correct.size();
                }
            }
            else
            {
                limit1 = Pos2Joint(PosLst_correct, AglLst, agl_acl, TInF);
                limit2 = Pos2Joint(PosLst_correct2, AglLst2, agl_acl, TInF);
                if(limit1 || limit2)
                {
                    pso_f[i] = 1e10;
                    pso_f2[i] = 1e10;
                }
                else
                {
                    pso_f[i] = 0;
                    pso_f2[i] = 0;
                    for(int j=0; j<PosLst_correct.size(); j++)
                        pso_f[i] += 180-abs(PosLst_correct[j].b);
                    pso_f2[i] += RobotAttitudEvaluate2(AglLst, PosLst_correct, aglBest);
                    pso_f2[i] += RobotAttitudEvaluate2(AglLst2, PosLst_correct2, aglBest);
                    pso_f2[i] = pso_f2[i]/(PosLst_correct.size()+PosLst_correct2.size());
                }
            }
            PosLst_correct.clear();
            PosLst_correct2.clear();
            AglLst.clear();
            AglLst2.clear();
        }

        for(int i=0; i<pso_Num; i++)
            if(pso_f[i] < pso_gbestf)
                pso_gbestf = pso_f[i];
        for(int i=0; i<pso_Num; i++)
            if(pso_f[i] == pso_gbestf)
                indexLst.push_back(i);
        for(int i=0; i<indexLst.size(); i++)
            if(pso_f2[indexLst[i]] < pso_gbestf2)
            {
                pso_gbestf2 = pso_f2[indexLst[i]];
                index = indexLst[i];
            }
    }
    StateWithSpeed = pso_x[index];
}


// 关联处理(手动设置)
void MainWindow::RelateProcess3(vector<vector<RobotPos>> PathLst, vector<vector<int>> RelateIndex, vector<vector<vector<RobotPos>>> &RelatePathLst_s)
{
    vector<vector<RobotPos>> PathLst2;
    // 根据关联索进行分类
    for(int i=0; i<RelateIndex.size(); i++)
    {
        for(int j=0; j<RelateIndex[i].size(); j++)
        {
            PathLst2.push_back(PathLst[RelateIndex[i][j]]);
        }
        RelatePathLst_s.push_back(PathLst2);
        PathLst2.clear();
    }
}


void MainWindow::PostureAdjustment(vector<RobotPos> &path)
{
    bool piPei, wetherEnd;
    vector<int> piPeiLst;
    RobotPos pos, posi;
    vector<RobotPos> path2;
    Matrix4d Mt, Mti, Mc, MCInT, MTInC;
    wetherEnd = false;
    double rotate180 = 0;
    int index;
    for(int i=0; i<path.size(); i+=5)
    {
        piPei = false;
        for(double j=-100; j<900; j++)
        {
            pos = path[i];
            PosRotate(path[i], pos, rotate180+0.1*j);
            pos2Matrix(pos, Mt);
            pos2Matrix(CInT, MCInT);
            Mc = Mt*MCInT;
            for(double k=i; k<i+200; k++){
                if(k>path.size()-5){
                    wetherEnd = true;
                    index = i;
                    break;
                }
                posi = path[k];
                pos2Matrix(posi, Mti);
                MTInC = Mc.inverse()*Mti;
                if(MTInC(0,3)<=0){
                    if(abs(MTInC(1,3))<=1){
                        if(path2.size()>0)
                        {
                            RobotPos pos1 = path2[path2.size()-1];
                            double dis = sqrt(pow(pos1.x-pos.x,2)+pow(pos1.y-pos.y,2)+pow(pos1.z-pos.z,2));
                            dis++;
                        }
                        path2.push_back(pos);
                        piPei = true;

                    }
                    break;
                }
            }
            if(wetherEnd)
                break;
            if(piPei)
                break;
        }
        if(wetherEnd)
            break;
        if(!piPei)
            piPeiLst.push_back(i/5);
    }
    for(int i=index; i<path.size(); i+=5)
    {
        pos = path[i];
        path2.push_back(pos);
//        pos = path2[path2.size()-1];
//        pos.x = path[i].x;
//        pos.y = path[i].y;
//        pos.z = path[i].z;
//        path2.push_back(pos);
    }
    path2.push_back(path[path.size()-1]);
    path.clear();
    path = path2;
}


// 只取两个点
void MainWindow::PostureAdjustment2(vector<RobotPos> &path)
{
    bool piPei, wetherEnd;
    RobotPos pos, posi;
    vector<RobotPos> path2;
    Matrix4d Mt, Mti, Mc, MCInT, MTInC;
    wetherEnd = false;
    double rotate180 = 0;
    path2.push_back(path[0]);
    for(int i=0; i<1; i+=5)
    {
        piPei = false;
        for(double j=-100; j<900; j++)
        {
            pos = path[i];
            PosRotate(path[i], pos, rotate180+0.1*j);
            pos2Matrix(pos, Mt);
            pos2Matrix(CInT, MCInT);
            Mc = Mt*MCInT;
            for(double k=i; k<i+200; k++){
                posi = path[k];
                pos2Matrix(posi, Mti);
                MTInC = Mc.inverse()*Mti;
                if(MTInC(0,3)<=0){
                    if(abs(MTInC(1,3))<=1){
                        path2.push_back(path[k]);
                        piPei = true;
                        break;
                    }
                    break;
                }
            }
            if(piPei)
                break;
        }
    }
    path.clear();
    path = path2;
}


// 将轨迹转化为间隔为20mm的初步龙骨
void MainWindow::KeeldataReorganize(vector<RobotPos> &path)
{
    RobotPos pos, pos1, pos2;
    vector<RobotPos> path2;
    int num = 2000;
    // 细化为间隔0.01mm的轨迹数据
    for(int i=0; i<path.size()-1; i++)
    {
        pos1 = path[i];
        pos2 = path[i+1];
        for(int k=0; k<num; k++){
            pos.x = pos1.x + k*(pos2.x-pos1.x)/num;
            pos.y = pos1.y + k*(pos2.y-pos1.y)/num;
            pos.z = pos1.z + k*(pos2.z-pos1.z)/num;
            pos.a = pos1.a;
            pos.b = pos1.b;
            pos.c = pos1.c;
            path2.push_back(pos);
        }
    }
    path2.push_back(pos2);
    interpolation(path2, path);

    // 摘取间隔为20mm的轨迹点
    double dis;
    path.clear();
    pos = path2[0];
    path.push_back(pos);
    for(int i=0; i<path2.size(); i++)
    {
        pos1 = path2[i];
        dis = sqrt(pow(pos1.x-pos.x,2)+pow(pos1.y-pos.y,2)+pow(pos1.z-pos.z,2));
        if(dis>=4.99)
        {
            path.push_back(pos1);
            pos = pos1;
        }
    }
    path.push_back(path2[path2.size()-1]);
}


// 获取跟踪的龙骨数据
void MainWindow::CreateKeel(vector<RobotPos> &path)
{
    vector<RobotPos> path2, path3;
    RobotPos pos, pos1, pos2;

    // 焊缝点细化（20mm间隔转化为1mm间隔）
    for(int j=0; j<path.size()-1; j++){
        pos1 = path[j];
        pos2 = path[j+1];
        for(int k=0; k<5; k++){
            pos.x = pos1.x + k*(pos2.x-pos1.x)/5;
            pos.y = pos1.y + k*(pos2.y-pos1.y)/5;
            pos.z = pos1.z + k*(pos2.z-pos1.z)/5;
            pos.a = pos1.a;
            pos.b = pos1.b;
            pos.c = pos1.c;
            path2.push_back(pos);
        }
        path3.push_back(pos1);
    }
    path3.push_back(pos2);
    path2.push_back(pos2);
    interpolation(path2, path3);
    path.clear();
    path = path2;

    // 生成龙骨
    PostureAdjustment(path);
}


// 获取跟踪的扫描数据
void MainWindow::CreateKeel2(vector<RobotPos> &path)
{
    vector<RobotPos> path2, path3;
    RobotPos pos, pos1, pos2;

    // 焊缝点细化（20mm间隔转化为1mm间隔）
    for(int j=0; j<path.size()-1; j++){
        pos1 = path[j];
        pos2 = path[j+1];
        for(int k=0; k<5; k++){
            pos.x = pos1.x + k*(pos2.x-pos1.x)/5;
            pos.y = pos1.y + k*(pos2.y-pos1.y)/5;
            pos.z = pos1.z + k*(pos2.z-pos1.z)/5;
            pos.a = pos1.a;
            pos.b = pos1.b;
            pos.c = pos1.c;
            path2.push_back(pos);
        }
        path3.push_back(pos1);
    }
    path3.push_back(pos2);
    path2.push_back(pos2);
    interpolation(path2, path3);
    path.clear();
    path = path2;

    // 生成龙骨
    PostureAdjustment2(path);
}


// 获取寻位的龙骨数据
void MainWindow::CreateKeelScan(vector<RobotPos> &path)
{
    vector<RobotPos> path2, path3;
    RobotPos pos, pos1, pos2;

    // 焊缝点细化（20mm间隔转化为5mm间隔）
    for(int j=0; j<path.size()-1; j++){
        pos1 = path[j];
        pos2 = path[j+1];
        for(int k=0; k<4; k++){
            pos.x = pos1.x + k*(pos2.x-pos1.x)/4;
            pos.y = pos1.y + k*(pos2.y-pos1.y)/4;
            pos.z = pos1.z + k*(pos2.z-pos1.z)/4;
            pos.a = pos1.a;
            pos.b = pos1.b;
            pos.c = pos1.c;
            path2.push_back(pos);
        }
        path3.push_back(pos1);
    }
    path3.push_back(pos2);
    path2.push_back(pos2);
    interpolation(path2, path3);
    path.clear();
    path = path2;
}


void MainWindow::pathReverse(vector<RobotPos> &path)
{
    RobotPos pos;
    vector<RobotPos> path2;
    for(int i=path.size()-1; i>=0; i--){
        pos = path[i];
        path2.push_back(pos);
    }
    path.clear();
    path = path2;
}


void MainWindow::laserReverse(vector<RobotPos> &path)
{
    for(auto &r : path)
        PosRotate(r, r, 180);
}


// 由CAD得到工件坐标系
void MainWindow::WorkpieceCoordinates(RobotPos posO, RobotPos posX, RobotPos &posW)
{
    Eigen::Vector3d vx, vy, vz;
    vx = {posX.x-posO.x, posX.y-posO.y, posX.z-posO.z};
    vz = {0, 0, -1};
    vx = vx/sqrt(vx.dot(vx));
    vy = vz.cross(vx);
    vy = vy/sqrt(vy.dot(vy));
    Eigen::Matrix4d m;
    m << vx[0], vy[0], vz[0],     posO.x,
         vx[1], vy[1], vz[1],     posO.y,
         vx[2], vy[2], vz[2],     posO.z,
             0,     0,     0,          1;
    Matrix2pos(m, posW);
}


// 由CAD数据获取工件坐标系下的轨迹
void MainWindow::posInTool(vector<RobotPos> &path, RobotPos posW)
{
    Eigen::Matrix4d mw, mr, mt;
    pos2Matrix(posW, mw);
    for(auto &r : path)
    {
        pos2Matrix(r, mr);
        mt = mw.inverse()*mr;
        Matrix2pos(mt, r);
    }
}


// 由CAD数据获取基坐标系下的轨迹
void MainWindow::posInBase(vector<RobotPos> &path,
                           RobotPos posW1,  //CAD工件坐标系
                           RobotPos posW2  //实际工件坐标系
                           )
{
    Eigen::Matrix4d mw1, mw2, mr, mt;
    pos2Matrix(posW1, mw1);
    pos2Matrix(posW2, mw2);
    for(auto &r : path)
    {
        pos2Matrix(r, mr);
        mt = mw1.inverse()*mr;
        mt = mw2*mt;
        Matrix2pos(mt, r);
    }
}


// 机器人姿态评价
double MainWindow::RobotAttitudEvaluate2(vector<RobotAxle> aglLst, vector<RobotPos> path, RobotAxle aglBest)
{
    double fun = 0;
    RobotAxle agl;
    RobotPos pos;
    double k = 0.1;
    double dis = 1200;
    int posNum = backPosNum;
    for(int i=posNum; i<aglLst.size()-posNum; i++)
    {
        agl = aglLst[i];
        pos = path[i];
        fun += pow(agl.a3-aglBest.a3,2);
        fun += pow(agl.a5-aglBest.a5,2);
        fun += k*pow(abs(sqrt(pow(pos.x,2)+pow(pos.y,2))-dis),2);
//        if(abs(agl.a3-aglBest.a3)>30)
//            fun += pow(agl.a3-aglBest.a3,2);
//        if(abs(agl.a5-aglBest.a5)>30)
//            fun += pow(agl.a5-aglBest.a5,2);
//        if(abs(sqrt(pow(pos.x,2)+pow(pos.y,2))-dis)>300)
//            fun += k*pow(abs(sqrt(pow(pos.x,2)+pow(pos.y,2))-dis),2);
    }
//    fun = fun/aglLst.size();
    return fun;
}


// 路径末端姿态修改(绕y轴旋转)
void MainWindow::pathEndRotate(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection)
{
    RobotPos cpos;
    Matrix4d mc, rx;
    double agl = rotataAgle/posNum;
    if(!rotateDirection)
        agl = -agl;
    rx << 1,            0,             0,  0,
          0, cos(-M_PI/2), -sin(-M_PI/2),  0,
          0, sin(-M_PI/2),  cos(-M_PI/2),  0,
          0,            0,             0,  1;
    if(posNum>path.size())
        posNum = path.size();
    int startIdx = path.size()-posNum-1;
    for(int i=startIdx+1; i<path.size(); i++)
    {
        cpos = path[i];
        pos2Matrix(cpos, mc);
        mc = mc*rx;
        Matrix2pos(mc, cpos);
        PosRotate(cpos, path[i], (i-startIdx)*agl);
    }
}


// 路径首端姿态修改(绕y轴旋转)
void MainWindow::pathHeadRotate(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection)
{
    RobotPos cpos;
    Matrix4d mc, rx;
    double agl = rotataAgle/posNum;
    if(!rotateDirection)
        agl = -agl;
    rx << 1,            0,             0,  0,
          0, cos(-M_PI/2), -sin(-M_PI/2),  0,
          0, sin(-M_PI/2),  cos(-M_PI/2),  0,
          0,            0,             0,  1;
    int startIdx = 0;
    if(posNum>path.size())
        posNum = path.size();
    for(int i=startIdx; i<posNum; i++)
    {
        cpos = path[i];
        pos2Matrix(cpos, mc);
        mc = mc*rx;
        Matrix2pos(mc, cpos);
        PosRotate(cpos, path[i], (posNum-i)*agl);
    }
}


// 绕x轴旋转
double MainWindow::posRotateX(vector<RobotPos> &path, double rotataAgle)
{
    RobotPos cpos;
    Matrix4d mc, ry;
    ry << cos(M_PI/2), 0, sin(M_PI/2), 0,
                    0, 1,           0, 0,
         -sin(M_PI/2), 0, cos(M_PI/2), 0,
                    0, 0,           0, 1;
    for(int i=0; i<path.size(); i++)
    {
        cpos = path[i];
        pos2Matrix(cpos, mc);
        mc = mc*ry;
        Matrix2pos(mc, cpos);
        PosRotate(cpos, path[i], rotataAgle);
    }
}


// 路径末端姿态修改(绕x轴旋转)
void MainWindow::pathEndRotateX(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection)
{
    RobotPos cpos;
    Matrix4d mc, ry;
    double agl = rotataAgle/posNum;
    if(!rotateDirection)
        agl = -agl;
    ry << cos(M_PI/2), 0, sin(M_PI/2), 0,
                    0, 1,           0, 0,
         -sin(M_PI/2), 0, cos(M_PI/2), 0,
                    0, 0,           0, 1;
    if(posNum>path.size())
        posNum = path.size();
    int startIdx = path.size()-posNum-1;
    for(int i=startIdx+1; i<path.size(); i++)
    {
        cpos = path[i];
        pos2Matrix(cpos, mc);
        mc = mc*ry;
        Matrix2pos(mc, cpos);
        PosRotate(cpos, path[i], (i-startIdx)*agl);
    }
}


// 绕y轴旋转
double MainWindow::posRotateY(vector<RobotPos> &path, double rotataAgle)
{
    RobotPos cpos;
    Matrix4d mc, rx;
    rx << 1,            0,             0,  0,
          0, cos(-M_PI/2), -sin(-M_PI/2),  0,
          0, sin(-M_PI/2),  cos(-M_PI/2),  0,
          0,            0,             0,  1;
    for(int i=0; i<path.size(); i++)
    {
        cpos = path[i];
        pos2Matrix(cpos, mc);
        mc = mc*rx;
        Matrix2pos(mc, cpos);
        PosRotate(cpos, path[i], rotataAgle);
    }
}


// 将基坐标系下的轨迹转化为CAD中的轨迹
void MainWindow::base2Cad(vector<RobotPos> &path,
                           RobotPos posW1,  //CAD工件坐标系
                           RobotPos posW2  //实际工件坐标系
                           )
{
    Eigen::Matrix4d mw1, mw2, mr, mt;
    pos2Matrix(posW1, mw1);
    pos2Matrix(posW2, mw2);
    for(auto &r : path)
    {
        pos2Matrix(r, mr);
        mt = mw2.inverse()*mr;
        mt = mw1*mt;
        Matrix2pos(mt, r);
    }
}


void MainWindow::on_UnLinkage_clicked()
{
    vector<vector<RobotPos>> pathLst;
    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;
    vector<vector<int>> WeldClassifyPlan;
    Read_pathlst(pathLst, filename);

    // 绕X轴旋转
    for(int i=0; i<pathLst.size(); i++)
        posRotateX(pathLst[i], 15);

    // 焊缝关联信息，关联分类
    vector<vector<int>> RelateIndex;
//    RelateIndex = {{3}, {5}, {7,8}, {9,10}, {11,12,13}, {0}};
//    RelateIndex = {{0, 1}};
//    RelateIndex = {{4, 5}};
//    RelateIndex = {{6, 7}};
//    RelateIndex = {{10, 11}};
//    RelateIndex = {{4}};
//    RelateIndex = {{6}};
//    RelateIndex = {{7, 9, 32}};
    RelateIndex = {{0}};
//    RelateIndex = {{1,2}};
    RelateProcess3(pathLst, RelateIndex, pathLst_s);

    // 读配置文件
    vector<vector<string>> nameLsts;
    vector<vector<double>> region;
    vector<int> direction;
    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead, directionLst;
    vector<vector<vector<int>>> directionLst_s;
    readConfig(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead, region, nameLsts);
    for(int i=0; i<RelateIndex.size(); i++)
    {
        for(int j=0; j<RelateIndex[i].size(); j++)
        {
            direction = {pathDirection[i][j], laserDirection[i][j], rotateYHead[i][j], rotateYEnd[i][j]};
            directionLst.push_back(direction);
        }
        directionLst_s.push_back(directionLst);
        directionLst.clear();
    }
    double dx, dy, dz, da, db;
    dx = (region[0][2]-region[0][1])/region[0][0];
    dy = (region[1][2]-region[1][1])/region[1][0];
    dz = (region[2][2]-region[2][1])/region[2][0];
    da = (region[3][2]-region[3][1])/region[3][0];
    db = (region[4][2]-region[4][1])/region[4][0];

    // 转换为工件坐标系下的位姿
    pathLst_s2 = pathLst_s;
    for(auto &r0 : pathLst_s2)
        for(auto &r1 : r0)
            posInTool(r1, posW1);

    // 转换为零点下状态时基标系下的位姿
    for(auto &r0 : pathLst_s)
        for(auto &r1 : r0)
            posInBase(r1, posW1, posW2);

   // 非联动处理
   WeldClassify9(pathLst_s, region, WeldClassifyPlan, directionLst_s, pathDirection, laserDirection, rotateYHead, rotateYEnd);

   // 根据焊缝附带信息调节焊缝姿态
   for(int i=0; i<pathLst_s2.size(); i++)
   {
       for(int j=0; j<pathLst_s2[i].size(); j++)
       {
           Weld2Path(pathLst_s2[i][j], directionLst_s[i][j]);
       }
   }


   vector<RobotPos> path;
   RobotPos cpos = {0,0,0,0,0,0};
   path = pathLst_s2[0][0];
   for(int i=1; i<12; i++)
   {
       for(auto &r : path)
           PosRotate(cpos, r, -30);
       pathLst_s[0][0] = path;
       pathLst_s2.push_back(pathLst_s[0]);
   }
   pathLst_s3 = pathLst_s2;


//   // 焊缝整理
//   RepeatData *Data = new RepeatData;
//   Zitai_Info Zitai;
//   RepeatSeamArray seam_s;
//   Seaminfo seam;
//   RepeatPos posLst1, posLst2;
//   Zitai.spacing1 = 10;
//   Zitai.spacing2 = 10;
//   Zitai.distance1 = 10;
//   Zitai.distance2 = 10;
//   for(int i=0; i<12; i++)
//   {
//       for(int k=0; k<pathLst_s2[i].size(); k++)
//           CreateKeelScan(pathLst_s3[i][k]);  //扫描龙骨数据
//       for(int k=0; k<pathLst_s2[i].size(); k++)
//       {
//           for(int ki=0; ki<pathLst_s2[i][k].size(); ki++){
//               RobotPos p = pathLst_s2[i][k][ki];
//               posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           for(int ki=0; ki<pathLst_s3[i][k].size(); ki++){
//               RobotPos p = pathLst_s3[i][k][ki];
//               posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           seam.scanePos = posLst1;
//           seam.framePos = posLst2;
//           seam.seamName = "seam"+std::to_string(i+1)+"_1_0";
//           seam.seamIndex = 0;
//           seam.orderIndex = 0;
//           seam.bind1 = 0;
//           seam.bind2 = 0;
//           seam_s.push_back(seam);
//           posLst1.clear();
//           posLst2.clear();
//       }

//       Zitai.cadid = CadName;
//       Zitai.posArray = seam_s;
//       seam_s.clear();
//       Zitai.pos_motor_start = 700;
//       Zitai.pos_motor_stop = 700;
//       Zitai.angle1_motor_start = -30;
//       Zitai.angle1_motor_stop = -30;
//       Zitai.angle2_motor_start = 56-30*i;
//       Zitai.angle2_motor_stop = 56-30*i;
//       Zitai.motor1speed = 0;
//       Zitai.motor2speed = 0;
//       Zitai.motor3speed = 0;
//       Data->push_back(Zitai);
//   }
//   std::string ret = InputControl::CreatSerial(*Data);




//    vector<vector<RobotPos>> pathLst;
//    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;
//    vector<vector<int>> WeldClassifyPlan;
//    Read_pathlst(pathLst, filename);

//    // 绕X轴旋转
//    for(int i=0; i<pathLst.size(); i++)
//        posRotateX(pathLst[i], 15);

//    // 焊缝关联信息，关联分类
//    vector<vector<int>> RelateIndex;
////    RelateIndex = {{3}, {5}, {7,8}, {9,10}, {11,12,13}, {0}};
////    RelateIndex = {{0, 1}};
////    RelateIndex = {{4, 5}};
////    RelateIndex = {{6, 7}};
////    RelateIndex = {{10, 11}};
////    RelateIndex = {{4}};
////    RelateIndex = {{6}};
////    RelateIndex = {{7, 9, 32}};
//    RelateIndex = {{0}};
////    RelateIndex = {{1,2}};
//    RelateProcess3(pathLst, RelateIndex, pathLst_s);

//    // 读配置文件
//    vector<vector<string>> nameLsts;
//    vector<vector<double>> region;
//    vector<int> direction;
//    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead, directionLst;
//    vector<vector<vector<int>>> directionLst_s;
//    readConfig(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead, region, nameLsts);
//    for(int i=0; i<RelateIndex.size(); i++)
//    {
//        for(int j=0; j<RelateIndex[i].size(); j++)
//        {
//            direction = {pathDirection[i][j], laserDirection[i][j], rotateYHead[i][j], rotateYEnd[i][j]};
//            directionLst.push_back(direction);
//        }
//        directionLst_s.push_back(directionLst);
//        directionLst.clear();
//    }
//    double dx, dy, dz, da, db;
//    dx = (region[0][2]-region[0][1])/region[0][0];
//    dy = (region[1][2]-region[1][1])/region[1][0];
//    dz = (region[2][2]-region[2][1])/region[2][0];
//    da = (region[3][2]-region[3][1])/region[3][0];
//    db = (region[4][2]-region[4][1])/region[4][0];

//    // 转换为工件坐标系下的位姿
//    pathLst_s2 = pathLst_s;
//    for(auto &r0 : pathLst_s2)
//        for(auto &r1 : r0)
//            posInTool(r1, posW1);

//    // 转换为零点下状态时基标系下的位姿
//    for(auto &r0 : pathLst_s)
//        for(auto &r1 : r0)
//            posInBase(r1, posW1, posW2);

//   // 非联动处理
//   WeldClassify9(pathLst_s, region, WeldClassifyPlan, directionLst_s, pathDirection, laserDirection, rotateYHead, rotateYEnd);

//   // 根据焊缝附带信息调节焊缝姿态
//   for(int i=0; i<pathLst_s2.size(); i++)
//   {
//       for(int j=0; j<pathLst_s2[i].size(); j++)
//       {
//           Weld2Path(pathLst_s2[i][j], directionLst_s[i][j]);
//       }
//   }
//   pathLst_s3 = pathLst_s2;



//   // 焊缝整理
//   RepeatData *Data = new RepeatData;
//   Zitai_Info Zitai;
//   RepeatSeamArray seam_s;
//   Seaminfo seam;
//   RepeatPos posLst1, posLst2;
//   Zitai.spacing1 = 10;
//   Zitai.spacing2 = 10;
//   Zitai.distance1 = 10;
//   Zitai.distance2 = 10;
//   int orderIndex = 0;
//   for(int i=0; i<WeldClassifyPlan.size(); i++)
//   {
//       int kk = 0;
//       for(int j=5; j<WeldClassifyPlan[i].size(); j++)
//       {
//           if(WeldClassifyPlan[i][j]==1)
//           {
//               kk++;
//               orderIndex++;
//               for(int k=0; k<pathLst_s2[j-5].size(); k++)
//                   CreateKeelScan(pathLst_s3[j-5][k]);  //扫描龙骨数据
//               for(int k=0; k<pathLst_s2[j-5].size(); k++)
//               {
//                   for(int ki=0; ki<pathLst_s2[j-5][k].size(); ki++){
//                       RobotPos p = pathLst_s2[j-5][k][ki];
//                       posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//                   }
//                   for(int ki=0; ki<pathLst_s3[j-5][k].size(); ki++){
//                       RobotPos p = pathLst_s3[j-5][k][ki];
//                       posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//                   }
//                   seam.scanePos = posLst1;
//                   seam.framePos = posLst2;
//                   seam.seamName = nameLsts[j-5][k];
//                   seam.seamIndex = 0;
//                   seam.orderIndex = 0;
//                   seam.bind1 = 0;
//                   seam.bind2 = 0;
//                   seam_s.push_back(seam);
//                   posLst1.clear();
//                   posLst2.clear();
//               }
//           }
//       }
//       Zitai.cadid = CadName;
//       Zitai.posArray = seam_s;
//       seam_s.clear();
//       Zitai.pos_motor_start = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
//       Zitai.pos_motor_stop = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
//       Zitai.angle1_motor_start = region[4][1]+WeldClassifyPlan[i][4]*db;
//       Zitai.angle1_motor_stop = region[4][1]+WeldClassifyPlan[i][4]*db;
//       Zitai.angle2_motor_start = -(region[3][1]+WeldClassifyPlan[i][3]*da);
//       Zitai.angle2_motor_stop = -(region[3][1]+WeldClassifyPlan[i][3]*da);
//       Zitai.motor1speed = 0;
//       Zitai.motor2speed = 0;
//       Zitai.motor3speed = 0;
//       Data->push_back(Zitai);
//   }
//   std::string ret = InputControl::CreatSerial(*Data);
}


// 写入数据库
void MainWindow::Write2Database_Seek(RepeatData &Data, vector<vector<int>> WeldClassifyPlan, double dx, double da,
                                vector<vector<double>> region, vector<vector<string>> nameLsts,
                                vector<vector<vector<RobotPos>>> pathLst_sS, vector<vector<vector<RobotPos>>> pathLst_sK)
{
    AScanePos sPos;
    AFramePos fPos;
    QList<AScanePos> scanePos;
    QList<AFramePos> framePos;
    SeamInfo seam;
    QList<SeamInfo> seams;
    MachineGrond  machine;
    for(int i=0; i<WeldClassifyPlan.size(); i++)
    {
        for(int j=5; j<WeldClassifyPlan[i].size(); j++)
        {
            if(WeldClassifyPlan[i][j]==1)
            {
                for(int k=0; k<pathLst_sK[j-5].size(); k++)
                    CreateKeelScan(pathLst_sK[j-5][k]);  //扫描龙骨数据
                for(int k=0; k<pathLst_sS[j-5].size(); k++)
                {
                    for(int ki=0; ki<pathLst_sS[j-5][k].size(); ki++){
                        RobotPos p = pathLst_sS[j-5][k][ki];
                        sPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
                        sPos.index1 = ki;
                        scanePos.push_back(sPos);
                    }
                    for(int ki=0; ki<pathLst_sK[j-5][k].size(); ki++){
                        RobotPos p = pathLst_sK[j-5][k][ki];
                        fPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
                        fPos.index1 = ki;
                        framePos.push_back(fPos);
                    }
                    seam.scanePos = scanePos;
                    seam.framePos = framePos;
                    seam.seamName = nameLsts[j-5][k];
                    machine.pos_motor_start = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
                    machine.pos_motor_stop = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
//                    machine.angle1_motor_start = region[4][1]+WeldClassifyPlan[i][4]*db;
//                    machine.angle1_motor_stop = region[4][1]+WeldClassifyPlan[i][4]*db;
                    machine.angle2_motor_start = -(region[3][1]+WeldClassifyPlan[i][3]*da);
                    machine.angle2_motor_stop = -(region[3][1]+WeldClassifyPlan[i][3]*da);
                    seam.machine = machine;
                    Data.seams.push_back(seam);
                    scanePos.clear();
                    framePos.clear();
                }
            }
        }
    }
}


void MainWindow::on_Linkage_clicked()
{
    vector<vector<RobotPos>> pathLst;
    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s4, pathLst_s0;
    Read_pathlst(pathLst, filename);

    // 焊缝关联信息，关联分类
    vector<vector<int>> RelateIndex;
//    RelateIndex = {{29,1}};
//    RelateIndex = {{0,1}};
//    RelateIndex = {{2,0}};
//    RelateIndex = {{8,6}};
//    RelateIndex = {{12}};
//    RelateIndex = {{13}};
//    RelateIndex = {{14}};
//    RelateIndex = {{15}};
    RelateIndex = {{16}};
    RelateProcess3(pathLst, RelateIndex, pathLst_s);
    pathLst_s0 = pathLst_s;

    // 读配置文件
    vector<vector<string>> nameLsts;
    vector<vector<double>> region, region2;
    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead;
    readConfig(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead, region, nameLsts);
    double dx, dy, dz, da, db;
    dx = (region[0][2]-region[0][1])/region[0][0];
    dy = (region[1][2]-region[1][1])/region[1][0];
    dz = (region[2][2]-region[2][1])/region[2][0];
    da = (region[3][2]-region[3][1])/region[3][0];
    db = (region[4][2]-region[4][1])/region[4][0];

    // 根据焊缝附带信息调节焊缝姿态
    for(int i=0; i<pathLst_s.size(); i++)
    {
        for(int j=0; j<pathLst_s[i].size(); j++)
        {
            if(pathDirection[i][j]==1){
                pathReverse(pathLst_s[i][j]);
                laserReverse(pathLst_s[i][j]);
            }

            if(laserDirection[i][j]==1){
                laserReverse(pathLst_s[i][j]);
                posRotateY(pathLst_s[i][j], aglInclude-aglWeld);
            }
            else{
                posRotateY(pathLst_s[i][j], aglInclude+aglWeld);
            }

            if(rotateYEnd[i][j]==1){
                if(laserDirection[i][j]==1){
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,5,0);
                }
                else{
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,5,1);
                }
            }

            if(rotateYHead[i][j]==1){
                if(laserDirection[i][j]==1){
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,5,1);
                }
                else{
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,5,0);
                }
            }
        }
    }
    for(int i=0; i<pathLst_s0.size(); i++)
    {
        for(int j=0; j<pathLst_s0[i].size(); j++)
        {
            if(pathDirection[i][j]==1){
                pathReverse(pathLst_s0[i][j]);
                laserReverse(pathLst_s0[i][j]);
            }
        }
    }

    // 转换为工件坐标系下的位姿
//    RobotPos posW1, posO, posX;
//    posO = {-62, 2017.5, 551, 0,0,0};
//    posX = {-640, 1691, 551, 0,0,0};
//    WorkpieceCoordinates(posO, posX, posW1);
    pathLst_s2 = pathLst_s;
    for(auto &r0 : pathLst_s2)
        for(auto &r1 : r0)
            posInTool(r1, posW1);
    pathLst_s4 = pathLst_s2;

    // 转换为零点下状态时基标系下的位姿
    for(auto &r0 : pathLst_s)
        for(auto &r1 : r0)
            posInBase(r1, posW1, posW2);
    for(auto &r0 : pathLst_s0)
        for(auto &r1 : r0)
            posInBase(r1, posW1, posW2);


   // 联动处理
    vector<int> index;
    vector<double> StateWithSpeed;
    vector<vector<double>> StateWithSpeedLst;
    vector<int> Variables = {1,0,0,1,1, 0,0,0,1,0};
    double weldTime;
    double weldSpeed = 5;
    vector<double> weldTimeLst;
    int pso_Num = 100;
    int time = 5;
    int splitNum, rank, rttDir;
    double offsetAgl = 30;
    readConfig_Int(configPath, std::to_string(RelateIndex[0][0]), "splitNum", splitNum);
    readConfig_Int(configPath, std::to_string(RelateIndex[0][0]), "rank", rank);
    readConfig_Int(configPath, std::to_string(RelateIndex[0][0]), "rttDir", rttDir);
    double aglE, aglS, aglG, wBest;
    {
        // 找到变位机最佳起始相位角与终止相位角
        region2 = region;
        findStartAngle(pathLst_s0[0][0][splitNum], region2[3], region2[4]);
        aglE = region2[3][1];
        findStartAngle(pathLst_s0[0][0][0], region[3], region[4]);
        aglS = region[3][1];
        aglG = aglE-aglS;
        if(region[4][1]<0)
            offsetAgl = -offsetAgl;
        region[3][1] += offsetAgl;
        region[3][2] += offsetAgl;
        if(rttDir){
            if(aglG<=0)
                aglG += 360;
        }
        else{
            if(aglG>=0)
                aglG -= 360;
        }
    }
    RobotPos pos, posi;
    {
       // 记录原始pos分界点
       pos = pathLst_s[0][0][splitNum];
    }
    for(int i=0; i<pathLst_s.size(); i++)
    {
       KeeldataReorganize(pathLst_s[i][0]);
       CreateKeel(pathLst_s[i][0]);
       {
           // 找到龙骨pos分界点
           double dis;
           for(int i=0; i<pathLst_s[0][0].size(); i++)
           {
               posi = pathLst_s[0][0][i];
               dis = sqrt(pow(posi.x-pos.x,2)+pow(posi.y-pos.y,2)+pow(posi.z-pos.z,2));
               if(dis<5)
               {
                   splitNum = i;
                   break;
               }
           }
       }
       if(rank!=3){
           splitNum = pathLst_s[0][0].size()-1;
       }
       {
           // 找到变位机最佳旋转速度
           wBest = aglG/(splitNum*5/weldSpeed);
           region[8][1] = wBest;
           region[8][2] = wBest;
       }
       ShipForm_Weld2(pathLst_s[i], region, StateWithSpeed, weldTime, Variables, weldSpeed, pso_Num, time, splitNum);
       StateWithSpeedLst.push_back(StateWithSpeed);
       weldTimeLst.push_back(weldTime);
       StateWithSpeed.clear();
       index.push_back(i);
    }

//    // 焊缝处理
//    RepeatData *Data = new RepeatData;
//    Zitai_Info Zitai;
//    RepeatSeamArray seam_s;
//    Seaminfo seam;
//    RepeatPos posLst1, posLst2;
//    Zitai.spacing1 = 10;
//    Zitai.spacing2 = 10;
//    Zitai.distance1 = 10;
//    Zitai.distance2 = 10;
//    int orderIndex = 0;
//    for(int i=0; i<StateWithSpeedLst.size(); i++)
//    {
//       // 跟踪焊缝
//       KeeldataReorganize(pathLst_s2[index[i]][0]);
//       CreateKeel2(pathLst_s2[index[i]][0]);  //扫描数据
//       KeeldataReorganize(pathLst_s4[index[i]][0]);
//       CreateKeel(pathLst_s4[index[i]][0]);  //龙骨数据
//       for(int k=0; k<1; k++)
//       {
//           orderIndex++;
//           for(int ki=0; ki<pathLst_s2[index[i]][k].size(); ki++){
//               RobotPos p = pathLst_s2[index[i]][k][ki];
//               posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           for(int ki=0; ki<pathLst_s4[index[i]][k].size(); ki++){
//               RobotPos p = pathLst_s4[index[i]][k][ki];
//               posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           seam.scanePos = posLst1;
//           seam.framePos = posLst2;
//           seam.seamName = nameLsts[i][0];
//           seam.orderIndex = orderIndex;
//           seam.bind1 = 0;
//           seam.bind2 = 0;
//           seam_s.push_back(seam);
//           posLst1.clear();
//           posLst2.clear();
//       }
//       Zitai.cadid = CadName;
//       Zitai.posArray = seam_s;
//       seam_s.clear();
//       Zitai.motor1speed = 0;
//       Zitai.motor2speed = ((int)(10*(StateWithSpeedLst[i][5]*10800/360)))/10.0;
//       Zitai.motor3speed = ((int)(10*(-StateWithSpeedLst[i][4]*12100/360)))/10.0;
//       Zitai.pos_motor_start = (int)(-StateWithSpeedLst[i][0]);
//       Zitai.pos_motor_stop = (int)(-StateWithSpeedLst[i][0]);
//       Zitai.angle1_motor_start = (int)(StateWithSpeedLst[i][2]);
//       Zitai.angle1_motor_stop = (int)(StateWithSpeedLst[i][2]);
//       Zitai.angle2_motor_start = (int)(-StateWithSpeedLst[i][1]);
//       Zitai.angle2_motor_stop = (int)(-StateWithSpeedLst[i][1] + weldTimeLst[i]*Zitai.motor3speed*360/12100);
//       Data->push_back(Zitai);

//       // 非跟踪焊缝
//       if(pathLst_s4[index[i]].size()>1)
//       {
//           CreateKeelScan(pathLst_s4[index[i]][1]);  //龙骨数据
//           for(int k=1; k<2; k++)
//           {
//               orderIndex++;
//               for(int ki=0; ki<pathLst_s2[index[i]][k].size(); ki++){
//                   RobotPos p = pathLst_s2[index[i]][k][ki];
//                   posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//               }
//               for(int ki=0; ki<pathLst_s4[index[i]][k].size(); ki++){
//                   RobotPos p = pathLst_s4[index[i]][k][ki];
//                   posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//               }
//               seam.scanePos = posLst1;
//               seam.framePos = posLst2;
//               seam.seamName = nameLsts[i][1];
//               seam.orderIndex = orderIndex;
//               seam.bind1 = 0;
//               seam.bind2 = 0;
//               seam_s.push_back(seam);
//               posLst1.clear();
//               posLst2.clear();
//           }
//           Zitai.posArray = seam_s;
//           seam_s.clear();
//           Zitai.motor1speed = 0;
//           Zitai.motor2speed = 0;
//           Zitai.motor3speed = 0;
//           Zitai.pos_motor_start = (int)(-StateWithSpeedLst[i][0]);
//           Zitai.pos_motor_stop = (int)(-StateWithSpeedLst[i][0]);
//           Zitai.angle1_motor_start = (int)(StateWithSpeedLst[i][2]);
//           Zitai.angle1_motor_stop = (int)(StateWithSpeedLst[i][2]);
//           Zitai.angle2_motor_start = Zitai.angle2_motor_stop;
//           Zitai.angle2_motor_stop = Zitai.angle2_motor_stop;
//           Data->push_back(Zitai);
//       }
//    }
//    std::string ret = InputControl::CreatSerial(*Data);
}


// 写入数据库
void MainWindow::Write2Database_Track(RepeatData &Data, vector<vector<double>> StateWithSpeedLst,
                                vector<int> index, vector<vector<string>> nameLsts, vector<double> weldTimeLst,
                                vector<vector<vector<RobotPos>>> pathLst_sS, vector<vector<vector<RobotPos>>> pathLst_sK)
{
    AScanePos sPos;
    AFramePos fPos;
    QList<AScanePos> scanePos;
    QList<AFramePos> framePos;
    SeamInfo seam;
    QList<SeamInfo> seams;
    MachineGrond  machine;

    for(int i=0; i<StateWithSpeedLst.size(); i++)
    {
       // 跟踪焊缝
       KeeldataReorganize(pathLst_sS[index[i]][0]);
       CreateKeel2(pathLst_sS[index[i]][0]);  //扫描数据
       KeeldataReorganize(pathLst_sK[index[i]][0]);
       CreateKeel(pathLst_sK[index[i]][0]);  //龙骨数据
       for(int k=0; k<1; k++)
       {
           for(int ki=0; ki<pathLst_sS[index[i]][k].size(); ki++){
               RobotPos p = pathLst_sS[index[i]][k][ki];
               sPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
               sPos.index1 = ki;
               scanePos.push_back(sPos);
           }
           for(int ki=0; ki<pathLst_sK[index[i]][k].size(); ki++){
               RobotPos p = pathLst_sK[index[i]][k][ki];
               fPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
               fPos.index1 = ki;
               framePos.push_back(fPos);
           }
           seam.scanePos = scanePos;
           seam.framePos = framePos;
           seam.seamName = nameLsts[i][k];
           machine.motor1speed = 0;
           machine.motor2speed = ((int)(10*(StateWithSpeedLst[i][5]*10800/360)))/10.0;
           machine.motor3speed = ((int)(10*(-StateWithSpeedLst[i][4]*12100/360)))/10.0;
           machine.pos_motor_start = (int)(-StateWithSpeedLst[i][0]);
           machine.pos_motor_stop = (int)(-StateWithSpeedLst[i][0]);
           machine.angle1_motor_start = (int)(StateWithSpeedLst[i][2]);
           machine.angle1_motor_stop = (int)(StateWithSpeedLst[i][2]);
           machine.angle2_motor_start = (int)(-StateWithSpeedLst[i][1]);
           machine.angle2_motor_stop = (int)(-StateWithSpeedLst[i][1] + weldTimeLst[i]*machine.motor3speed*360/12100);
           seam.machine = machine;
           Data.seams.push_back(seam);
           scanePos.clear();
           framePos.clear();
       }

       // 非跟踪焊缝
       if(pathLst_sK[index[i]].size()>1)
       {
           CreateKeelScan(pathLst_sK[index[i]][1]);  //龙骨数据
           for(int k=1; k<2; k++)
           {
               for(int ki=0; ki<pathLst_sS[index[i]][k].size(); ki++){
                   RobotPos p = pathLst_sS[index[i]][k][ki];
                   sPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
                   sPos.index1 = ki;
                   scanePos.push_back(sPos);
               }
               for(int ki=0; ki<pathLst_sK[index[i]][k].size(); ki++){
                   RobotPos p = pathLst_sK[index[i]][k][ki];
                   fPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
                   fPos.index1 = ki;
                   framePos.push_back(fPos);
               }
               seam.scanePos = scanePos;
               seam.framePos = framePos;
               seam.seamName = nameLsts[i][k];
               machine.motor1speed = 0;
               machine.motor2speed = 0;
               machine.motor3speed = 0;
               machine.pos_motor_start = (int)(-StateWithSpeedLst[i][0]);
               machine.pos_motor_stop = (int)(-StateWithSpeedLst[i][0]);
               machine.angle1_motor_start = (int)(StateWithSpeedLst[i][2]);
               machine.angle1_motor_stop = (int)(StateWithSpeedLst[i][2]);
               machine.angle2_motor_start = machine.angle2_motor_stop;
               machine.angle2_motor_stop = machine.angle2_motor_stop;
               seam.machine = machine;
               Data.seams.push_back(seam);
               scanePos.clear();
               framePos.clear();
           }
       }
    }
}


// 三个点拟合一段圆弧
void MainWindow::getScanPos(vector<RobotPos> &path)
{
    // 摘取间隔为20mm的轨迹点
    double dis;
    RobotPos pos, pos1;
    vector<RobotPos> Opath;
    Opath = path;
    path.clear();
    pos = Opath[0];
    path.push_back(pos);
    for(int i=0; i<Opath.size(); i++)
    {
        pos1 = Opath[i];
        dis = sqrt(pow(pos1.x-pos.x,2)+pow(pos1.y-pos.y,2)+pow(pos1.z-pos.z,2));
        if(dis>=19.9)
        {
            path.push_back(pos1);
            pos = pos1;
        }
    }
}


// 三个点拟合一段圆弧
void MainWindow::getKeelPos(vector<RobotPos> &path)
{
    // 摘取间隔为5mm的轨迹点
    double dis;
    RobotPos pos, pos1;
    vector<RobotPos> Opath;
    Opath = path;
    path.clear();
    pos = Opath[0];
    path.push_back(pos);
    for(int i=0; i<Opath.size(); i++)
    {
        pos1 = Opath[i];
        dis = sqrt(pow(pos1.x-pos.x,2)+pow(pos1.y-pos.y,2)+pow(pos1.z-pos.z,2));
        if(dis>=4.9)
        {
            path.push_back(pos1);
            pos = pos1;
        }
    }
}


// 三个点拟合一段圆弧
void MainWindow::cmove(vector<RobotPos> path, int n1, int n2, vector<RobotPos> &path1, vector<RobotPos> &path2)
{
    Vector3d p0, p1, p2, p3, V, dp1, dp2, b;
    Matrix3d A;
    double b1, b2, b3;
    p1 = {path[0].x, path[0].y, path[0].z};
    p2 = {path[1].x, path[1].y, path[1].z};
    p3 = {path[2].x, path[2].y, path[2].z};
    dp1 = p2 - p1;
    dp2 = p3 - p2;
    V = (dp1).cross(dp2);
    A << 2*dp1(0), 2*dp1(1), 2*dp1(2),
         2*dp2(0), 2*dp2(1), 2*dp2(2),
             V(0),     V(1),     V(2);
    b1 = (pow(p2(0),2)+pow(p2(1),2)+pow(p2(2),2))-(pow(p1(0),2)+pow(p1(1),2)+pow(p1(2),2));
    b2 = (pow(p3(0),2)+pow(p3(1),2)+pow(p3(2),2))-(pow(p2(0),2)+pow(p2(1),2)+pow(p2(2),2));
    b3 = V.dot(p1);
    b = {b1, b2, b3};
    p0 = A.inverse()*b;

    Vector3d Vx, Vy, Vz;
    Matrix4d cM;
    RobotPos cpos;
    Vz = {0,0,1};
    Vx = V.cross(Vz);
    Vz = V;
    Vy = Vz.cross(Vx);
    Vx = Vx / sqrt(Vx.dot(Vx));
    Vy = Vy / sqrt(Vy.dot(Vy));
    Vz = Vz / sqrt(Vz.dot(Vz));
    cM << Vx(0), Vy(0), Vz(0), p0(0),
          Vx(1), Vy(1), Vz(1), p0(1),
          Vx(2), Vy(2), Vz(2), p0(2),
              0,     0,     0,     1;
    Matrix2pos(cM, cpos);

    Vector3d V1, V2, V3;
    double a12, a23;
    vector<RobotPos> path1_, path2_;
    V1 = p1 - p0;
    V2 = p2 - p0;
    V3 = p3 - p0;
    a12 = acos(V1.dot(V2)/(sqrt(V1.dot(V1))*sqrt(V2.dot(V2))));
    a23 = acos(V2.dot(V3)/(sqrt(V2.dot(V2))*sqrt(V3.dot(V3))));
    a12 = a12 / n1;
    a23 = a23 / n2;
    path1_.push_back(path[0]);
    path1_.push_back(path[1]);
    path2_.push_back(path[1]);
    path2_.push_back(path[2]);
    for(int i=0; i<n1; i++)
    {
        PosRotate(cpos, path[0], a12*180/M_PI);
        path1.push_back(path[0]);
    }
    path1.push_back(path[1]);
    interpolation(path1, path1_);
    for(int i=0; i<n2; i++)
    {
        PosRotate(cpos, path[1], a23*180/M_PI);
        path2.push_back(path[1]);
    }
    path2.push_back(path[2]);
    interpolation(path2, path2_);
}


void MainWindow::Smooth(vector<RobotPos> &path, vector<RobotPos> &pathRf)
{
    double lenth, len, per;
    vector<double> lenLst, perLst;
    RobotPos pos1, pos2, pos;
    vector<RobotPos> path2, path3;

    lenth = 0;
    for(int i=0; i<pathRf.size()-1; i++)
    {
        pos1 = pathRf[i];
        pos2 = pathRf[i+1];
        len = sqrt(pow(pos2.x-pos1.x,2)+pow(pos2.y-pos1.y,2)+pow(pos2.z-pos1.z,2));
        lenLst.push_back(len);
        lenth = lenth+len;
        len = 0;
    }
    perLst.push_back(0);
    for(int i=0; i<lenLst.size()-1; i++)
    {
        len += lenLst[i];
        per = len/lenth;
        perLst.push_back(per);
    }

    // 细化为间隔0.01mm的轨迹数据
    lenth = 0;
    int num = 1000;
    for(int i=0; i<path.size()-1; i++)
    {
        pos1 = path[i];
        pos2 = path[i+1];
        len = sqrt(pow(pos2.x-pos1.x,2)+pow(pos2.y-pos1.y,2)+pow(pos2.z-pos1.z,2));
        lenth = lenth+len;
        len = 0;
        for(int k=0; k<num; k++)
        {
            pos.x = pos1.x + k*(pos2.x-pos1.x)/num;
            pos.y = pos1.y + k*(pos2.y-pos1.y)/num;
            pos.z = pos1.z + k*(pos2.z-pos1.z)/num;
            pos.a = pos1.a;
            pos.b = pos1.b;
            pos.c = pos1.c;
            path2.push_back(pos);
        }
    }

    int idx = 0;
    for(int i=0; i<path2.size()-1; i++)
    {
        pos1 = path2[i];
        pos2 = path2[i+1];
        len += sqrt(pow(pos2.x-pos1.x,2)+pow(pos2.y-pos1.y,2)+pow(pos2.z-pos1.z,2));
        per = len/lenth;
        if(idx<perLst.size())
        {
            if(per>perLst[idx])
            {
                pos2.a = pathRf[idx].a;
                pos2.b = pathRf[idx].b;
                pos2.c = pathRf[idx].c;
                path3.push_back(pos2);
                idx++;
            }
        }
        if(i==path2.size()-2)
        {
            pos2.a = pathRf[pathRf.size()-1].a;
            pos2.b = pathRf[pathRf.size()-1].b;
            pos2.c = pathRf[pathRf.size()-1].c;
            path3.push_back(pos2);
        }
    }
    interpolation(path2, path3);

    // 摘取间隔为20mm的轨迹点
    double dis;
    path.clear();
    pos = path2[0];
    path.push_back(pos);
    for(int i=0; i<path2.size(); i++)
    {
        pos1 = path2[i];
        dis = sqrt(pow(pos1.x-pos.x,2)+pow(pos1.y-pos.y,2)+pow(pos1.z-pos.z,2));
        if(dis>=19.9)
        {
            path.push_back(pos1);
            pos = pos1;
        }
    }
    // 摘取间隔为5mm的轨迹点
    path3.clear();
    pos = path2[0];
    path3.push_back(pos);
    for(int i=0; i<path2.size(); i++)
    {
        pos1 = path2[i];
        dis = sqrt(pow(pos1.x-pos.x,2)+pow(pos1.y-pos.y,2)+pow(pos1.z-pos.z,2));
        if(dis>=4.9)
        {
            path3.push_back(pos1);
            pos = pos1;
        }
    }
    pathRf.clear();
    pathRf = path3;
}


void MainWindow::on_pushButton_4_clicked()
{
//    Matrix4d Ma, Mt, M;
//    RobotPos tpos;
//    pos2Matrix(cposa, Ma);
//    pos2Matrix(posW2, Mt);
//    M = Ma.inverse()*Mt;
//    Matrix2pos(M, tpos);
//    qDebug() << tpos.x << "," << tpos.y << "," << tpos.z << ","
//        << tpos.a << "," << tpos.b << "," << tpos.c <<  "\n";


//    RobotPos tpos = {-878.382,809.450,387.067,-7.113,44.167,77.474};
//    GetToolCoordinate(tpos, 0, 130, 45);
//    qDebug() << tpos.x << "," << tpos.y << "," << tpos.z << ","
//        << tpos.a << "," << tpos.b << "," << tpos.c <<  "\n";



    vector<RobotPos> path, path2, path3, path4;
    vector<vector<RobotPos>> pathLst;
    int nx = 2;
    int ny = 8;
    double dx = -170;
    double dy = 100;

    path.push_back({-230,60,72, 0,-135,-90});
    path.push_back({-230,100,72, 0,-135,-90});
    path2.push_back({-230,60,72, 0,-135,-90});
    path2.push_back({-230,100,72, 0,-135,-90});
    KeeldataReorganize(path);
    CreateKeel(path);
    KeeldataReorganize(path2);
    CreateKeel2(path2);

    path3 = path;
    path4 = path2;
    pathLst.push_back(path);
    pathLst.push_back(path2);
    for(int i=1; i<ny; i++)
    {
        for(int j=0; j<path3.size(); j++)
            path3[j].y += dy;
        pathLst.push_back(path3);
        for(int j=0; j<path4.size(); j++)
            path4[j].y += dy;
        pathLst.push_back(path4);
    }

    path3 = path;
    path4 = path2;
    for(int j=0; j<path3.size(); j++)
        path3[j].x += dx;
    for(int j=0; j<path4.size(); j++)
        path4[j].x += dx;
    pathLst.push_back(path3);
    pathLst.push_back(path4);
    for(int i=1; i<ny; i++)
    {
        for(int j=0; j<path3.size(); j++)
            path3[j].y += dy;
        pathLst.push_back(path3);
        for(int j=0; j<path4.size(); j++)
            path4[j].y += dy;
        pathLst.push_back(path4);
    }

//    path3 = path;
//    path4 = path2;
//    for(int j=0; j<path3.size(); j++)
//        path3[j].x += 383;
//    for(int j=0; j<path4.size(); j++)
//        path4[j].x += 383;
//    pathLst.push_back(path3);
//    pathLst.push_back(path4);
//    for(int i=1; i<4; i++)
//    {
//        for(int j=0; j<path3.size(); j++)
//            path3[j].y += 160;
//        pathLst.push_back(path3);
//        for(int j=0; j<path4.size(); j++)
//            path4[j].y += 160;
//        pathLst.push_back(path4);
//    }

    QString filename3 = "pathlst.txt";
    Write_File(pathLst, filename3);




//    RobotPos cpos1, cpos2, cpos3;
//    Matrix4d M1, M2, M3;
//    cpos1 = {0,0,0,0,0,-0.146};
//    cpos2 = {-1333.918, 847.477, -83.005, 83.038, 90.172, -163.799};
//    cpos3 = {396.078, -121.114, 13.941, -17.052, 90.200, -43.267};
//    pos2Matrix(cpos1, M1);
//    pos2Matrix(cpos2, M2);
//    pos2Matrix(cpos3, M3);
//    M2 = M1*M2;
//    M3 = M2*M3;
//    Matrix2pos(M2, cpos2);
//    Matrix2pos(M3, cpos3);
//    qDebug() << cpos2.x << "," << cpos2.y << "," << cpos2.z << ","
//             << cpos2.a << "," << cpos2.b << "," << cpos2.c <<  "\n";
//    qDebug() << cpos3.x << "," << cpos3.y << "," << cpos3.z << ","
//             << cpos3.a << "," << cpos3.b << "," << cpos3.c <<  "\n";





//    vector<RobotPos> path, path2;

//    path.push_back({-1115.6,700.7,-64,154.4,147.4,-86.9});
//    path.push_back({-1117.1,713.6,-82.9,154.1,146.6,-95.7});
//    path.push_back({-1121.3,743.6,-119.4,153.3,144.3,-119});
//    path.push_back({-1127.5,757.9,-133.9,143.2,140.2,-130.2});
//    path.push_back({-1138.4,773.9,-144.9,134.9,135.2,-139.4});
//    for(int i=0; i<path.size(); i++)
//        GetToolCoordinate(path[i], -900, 15, -90);
//    base2Cad(path, posW1, posW2);

////    path.push_back({247.3,726.7,60.5,40.6,158.2,105.3});
////    path.push_back({257.1,772.7,27.8,-6.3,148.3,63.1});
////    path.push_back({259.9,791.2,16.6,-19.6,140.4,51.4});
////    path.push_back({270.8,807.5,-1.4,-6.6,130.0,60.5});
////    path.push_back({276.9,818.3,-10.5,-8.0,128.2,59.6});
////    for(int i=0; i<path.size(); i++)
////        GetToolCoordinate(path[i], -900, 20, 100);
////    base2Cad(path, posW1, posW2);

//    vector<vector<RobotPos>> pathLst;
//    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;
//    Read_pathlst(pathLst, filename);

//    // 焊缝关联信息，关联分类
//    vector<vector<int>> RelateIndex;
//    RelateIndex = {{7}};
//    RelateProcess3(pathLst, RelateIndex, pathLst_s);
//    path2 = pathLst_s[0][0];

//    Smooth(path2, path);

//    // 转换为工件坐标系下的位姿
//    posInTool(path, posW1);
//    posInTool(path2, posW1);

//    // 轨迹微调
//    RobotPos pos, pos1, pos2;
//    double dx, dy, dz;
//    pos1 = path[path.size()-2];
//    pos2 = path[path.size()-1];
//    pos = path[path.size()-1];
//    dx = pos2.x-pos1.x;
//    dy = pos2.y-pos1.y;
//    dz = pos2.z-pos1.z;
//    for(int i=1; i<9; i++)
//    {
//        pos.x = pos2.x + i*dx;
//        pos.y = pos2.y + i*dy;
//        pos.z = pos2.z + i*dz;
//        path.push_back(pos);
//    }
//    pathEndRotate(path, 10, 8, 1);
////    pathEndRotateX(path, 20, 16, 1);
//    pos1 = path2[path2.size()-2];
//    pos2 = path2[path2.size()-1];
//    pos = path2[path2.size()-1];
//    dx = pos2.x-pos1.x;
//    dy = pos2.y-pos1.y;
//    dz = pos2.z-pos1.z;
//    for(int i=1; i<3; i++)
//    {
//        pos.x = pos2.x + i*dx;
//        pos.y = pos2.y + i*dy;
//        pos.z = pos2.z + i*dz;
//        path2.push_back(pos);
//    }
//    pathEndRotate(path2, 20, 4, 1);
////    pathEndRotateX(path2, 20, 4, 1);


//    RobotPos cpos = {0,0,0,0,0,0};
//    for(auto &r1 : path)
//        PosRotate(cpos, r1, 150);
//    for(auto &r1 : path2)
//        PosRotate(cpos, r1, 150);

////    RobotPos cpos = {0,0,0,0,0,0};
////    for(auto &r1 : path)
////        PosRotate(cpos, r1, 240);
////    for(auto &r1 : path2)
////        PosRotate(cpos, r1, 240);

//    pathLst_s[0][0] = path2;
//    pathLst_s2.push_back(pathLst_s[0]);
//    pathLst_s[0][0] = path;
//    pathLst_s3.push_back(pathLst_s[0]);

//    for(int i=1; i<12; i++)
//    {
//        for(auto &r : path)
//            PosRotate(cpos, r, -30);
//        for(auto &r : path2)
//            PosRotate(cpos, r, -30);
//        pathLst_s[0][0] = path2;
//        pathLst_s2.push_back(pathLst_s[0]);
//        pathLst_s[0][0] = path;
//        pathLst_s3.push_back(pathLst_s[0]);
//    }

//    // 焊缝整理
//    RepeatData *Data = new RepeatData;
//    Zitai_Info Zitai;
//    RepeatSeamArray seam_s;
//    Seaminfo seam;
//    RepeatPos posLst1, posLst2;
//    Zitai.spacing1 = 10;
//    Zitai.spacing2 = 10;
//    Zitai.distance1 = 10;
//    Zitai.distance2 = 10;
//    int orderIndex = 0;
//    for(int i=0; i<12; i++)
//    {
//        for(int k=0; k<pathLst_s2[i].size(); k++)
//        {
//           for(int ki=0; ki<pathLst_s2[i][k].size(); ki++)
//           {
//               RobotPos p = pathLst_s2[i][k][ki];
//               posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           for(int ki=0; ki<pathLst_s3[i][k].size(); ki++)
//           {
//               RobotPos p = pathLst_s3[i][k][ki];
//               posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           seam.scanePos = posLst1;
//           seam.framePos = posLst2;
//           seam.seamName = "seam"+std::to_string(i+1)+"_3_0";
//           seam.seamIndex = 0;
//           seam.orderIndex = 0;
//           seam.bind1 = 0;
//           seam.bind2 = 0;
//           seam_s.push_back(seam);
//           posLst1.clear();
//           posLst2.clear();
//        }


//        Zitai.cadid = CadName;
//        Zitai.posArray = seam_s;
//        seam_s.clear();

//        Zitai.pos_motor_start = 900;
//        Zitai.pos_motor_stop = 900;
//        Zitai.angle1_motor_start = -90;
//        Zitai.angle1_motor_stop = -90;
//        Zitai.angle2_motor_start = 150-15-30*i;
//        Zitai.angle2_motor_stop = 150-15-30*i;

////        Zitai.pos_motor_start = 900;
////        Zitai.pos_motor_stop = 900;
////        Zitai.angle1_motor_start = 100;
////        Zitai.angle1_motor_stop = 100;
////        Zitai.angle2_motor_start = 240-20-30*i;
////        Zitai.angle2_motor_stop = 240-20-30*i;

//        Zitai.motor1speed = 0;
//        Zitai.motor2speed = 0;
//        Zitai.motor3speed = 0;
//        Data->push_back(Zitai);
//    }
//    std::string ret = InputControl::CreatSerial(*Data);






//    vector<RobotPos> path, path1, path2, path3;
//    vector<vector<RobotPos>> pathLst;
//    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;
//    Read_pathlst(pathLst, filename);

//    // 焊缝关联信息，关联分类
//    vector<vector<int>> RelateIndex;
//    RelateIndex = {{5}};
//    RelateProcess3(pathLst, RelateIndex, pathLst_s);
//    for(int i=0; i<pathLst_s[0][0].size(); i++)
//    {
//        GetToolCoordinate(pathLst_s[0][0][i], -900, 15, -90);
////        GetToolCoordinate(pathLst_s[0][0][i], -900, 20, 100);
//    }
////    base2Cad(pathLst_s[0][0], posW1, posW2);
//    // 转换为工件坐标系下的位姿
//    for(auto &r0 : pathLst_s)
//        for(auto &r1 : r0)
//            posInTool(r1, posW2);

//    for(int i=0; i<pathLst_s[0][0].size()-2; i++)
//    {
//        path.push_back(pathLst_s[0][0][i]);
//        path.push_back(pathLst_s[0][0][i+1]);
//        path.push_back(pathLst_s[0][0][i+2]);
//        cmove(path, 1000, 1000, path1, path2);
//        for(int i=0; i<path1.size(); i++)
//            path3.push_back(path1[i]);
//        if(i<pathLst_s[0][0].size()-3)
//        {
//            path1.clear();
//            path2.clear();
//            path.clear();
//        }
//    }
//    for(int i=0; i<path2.size(); i++)
//        path3.push_back(path2[i]);

//    RobotPos cpos = {0,0,0,0,0,0};
//    for(auto &r1 : path3)
//    {
////        PosRotate(cpos, r1, 150);
//        PosRotate(cpos, r1, 240);
//    }
//    pathLst_s[0][0] = path3;
//    pathLst_s2.push_back(pathLst_s[0]);
//    pathLst_s3.push_back(pathLst_s[0]);
//    for(int i=1; i<12; i++)
//    {
//        for(auto &r1 : path3)
//        {
//            PosRotate(cpos, r1, -30);
//        }
//        pathLst_s[0][0] = path3;
//        pathLst_s2.push_back(pathLst_s[0]);
//        pathLst_s3.push_back(pathLst_s[0]);
//    }

//    // 焊缝整理
//    RepeatData *Data = new RepeatData;
//    Zitai_Info Zitai;
//    RepeatSeamArray seam_s;
//    Seaminfo seam;
//    RepeatPos posLst1, posLst2;
//    Zitai.spacing1 = 10;
//    Zitai.spacing2 = 10;
//    Zitai.distance1 = 10;
//    Zitai.distance2 = 10;
//    int orderIndex = 0;
//    for(int i=0; i<12; i++)
//    {
//        for(int k=0; k<pathLst_s2[i].size(); k++)
//        {
//            getScanPos(pathLst_s2[i][k]);
//            getKeelPos(pathLst_s3[i][k]);  //扫描龙骨数据
//        }
//        for(int k=0; k<pathLst_s2[i].size(); k++)
//        {
//           for(int ki=0; ki<pathLst_s2[i][k].size(); ki++)
//           {
//               RobotPos p = pathLst_s2[i][k][ki];
//               posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           for(int ki=0; ki<pathLst_s3[i][k].size(); ki++)
//           {
//               RobotPos p = pathLst_s3[i][k][ki];
//               posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           seam.scanePos = posLst1;
//           seam.framePos = posLst2;
//           seam.seamName = "seam"+std::to_string(i+1)+"_5_0";
//           seam.seamIndex = 0;
//           seam.orderIndex = 0;
//           seam.bind1 = 0;
//           seam.bind2 = 0;
//           seam_s.push_back(seam);
//           posLst1.clear();
//           posLst2.clear();
//        }


//        Zitai.cadid = CadName;
//        Zitai.posArray = seam_s;
//        seam_s.clear();

//        Zitai.pos_motor_start = 900;
//        Zitai.pos_motor_stop = 900;
//        Zitai.angle1_motor_start = -90;
//        Zitai.angle1_motor_stop = -90;
//        Zitai.angle2_motor_start = 150-15-30*i;
//        Zitai.angle2_motor_stop = 150-15-30*i;

//        Zitai.pos_motor_start = 900;
//        Zitai.pos_motor_stop = 900;
//        Zitai.angle1_motor_start = 100;
//        Zitai.angle1_motor_stop = 100;
//        Zitai.angle2_motor_start = 240-20-30*i;
//        Zitai.angle2_motor_stop = 240-20-30*i;

//        Zitai.motor1speed = 0;
//        Zitai.motor2speed = 0;
//        Zitai.motor3speed = 0;
//        Data->push_back(Zitai);
//    }
//    std::string ret = InputControl::CreatSerial(*Data);




//    vector<RobotPos> path, path1, path2, path3;
//    vector<vector<RobotPos>> pathLst;
//    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;
//    Read_pathlst(pathLst, filename);

//    // 焊缝关联信息，关联分类
//    vector<vector<int>> RelateIndex;
//    RelateIndex = {{0}};
//    RelateProcess3(pathLst, RelateIndex, pathLst_s);

//    for(auto &r0 : pathLst_s)
//        for(auto &r1 : r0)
//            posInTool(r1, posW1);

//    path3 = pathLst_s[0][0];
//    RobotPos cpos = {0,0,0,0,0,0};
//    pathLst_s[0][0] = path3;
//    pathLst_s2.push_back(pathLst_s[0]);
//    pathLst_s3.push_back(pathLst_s[0]);
//    for(int i=1; i<12; i++)
//    {
//        for(auto &r1 : path3)
//        {
//            PosRotate(cpos, r1, -30);
//        }
//        pathLst_s[0][0] = path3;
//        pathLst_s2.push_back(pathLst_s[0]);
//        pathLst_s3.push_back(pathLst_s[0]);
//    }

//    // 焊缝整理
//    RepeatData *Data = new RepeatData;
//    Zitai_Info Zitai;
//    RepeatSeamArray seam_s;
//    Seaminfo seam;
//    RepeatPos posLst1, posLst2;
//    Zitai.spacing1 = 10;
//    Zitai.spacing2 = 10;
//    Zitai.distance1 = 10;
//    Zitai.distance2 = 10;
//    int orderIndex = 0;
//    for(int i=0; i<12; i++)
//    {
//        for(int k=0; k<pathLst_s2[i].size(); k++)
//            CreateKeelScan(pathLst_s3[i][k]);  //扫描龙骨数据
//        for(int k=0; k<pathLst_s2[i].size(); k++)
//        {
//           for(int ki=0; ki<pathLst_s2[i][k].size(); ki++)
//           {
//               RobotPos p = pathLst_s2[i][k][ki];
//               posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           for(int ki=0; ki<pathLst_s3[i][k].size(); ki++)
//           {
//               RobotPos p = pathLst_s3[i][k][ki];
//               posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           seam.scanePos = posLst1;
//           seam.framePos = posLst2;
//           seam.seamName = "seam"+std::to_string(i+1)+"_1_0";
//           seam.seamIndex = 0;
//           seam.orderIndex = 0;
//           seam.bind1 = 0;
//           seam.bind2 = 0;
//           seam_s.push_back(seam);
//           posLst1.clear();
//           posLst2.clear();
//        }


//        Zitai.cadid = CadName;
//        Zitai.posArray = seam_s;
//        seam_s.clear();

//        Zitai.pos_motor_start = 700;
//        Zitai.pos_motor_stop = 700;
//        Zitai.angle1_motor_start = -30;
//        Zitai.angle1_motor_stop = -30;
//        Zitai.angle2_motor_start = 56-30*i;
//        Zitai.angle2_motor_stop = 56-30*i;

//        Zitai.motor1speed = 0;
//        Zitai.motor2speed = 0;
//        Zitai.motor3speed = 0;
//        Data->push_back(Zitai);
//    }
//    std::string ret = InputControl::CreatSerial(*Data);





//    vector<RobotPos> path, path2;
//    path.push_back({-299,708,0,-141.2,116.6,-123.1});
//    path.push_back({-379,700,0,-156.5,111.3,-129.1});
//    path.push_back({-459,711,0,-172.0,106.1,-140.0});
//    for(int i=0; i<path.size(); i++)
//        GetToolCoordinate(path[i], -850, -24, 0);
//    base2Cad(path, posW1, posW2);

//    vector<vector<RobotPos>> pathLst;
//    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;
//    Read_pathlst(pathLst, filename);

//    // 焊缝关联信息，关联分类
//    vector<vector<int>> RelateIndex;
//    RelateIndex = {{3}};
//    RelateProcess3(pathLst, RelateIndex, pathLst_s);

//    pathLst_s[0][0][0].a = path[0].a;
//    pathLst_s[0][0][0].b = path[0].b;
//    pathLst_s[0][0][0].c = path[0].c;
//    pathLst_s[0][0][pathLst_s[0][0].size()/2].a = path[1].a;
//    pathLst_s[0][0][pathLst_s[0][0].size()/2].b = path[1].b;
//    pathLst_s[0][0][pathLst_s[0][0].size()/2].c = path[1].c;
//    pathLst_s[0][0][pathLst_s[0][0].size()-1].a = path[2].a;
//    pathLst_s[0][0][pathLst_s[0][0].size()-1].b = path[2].b;
//    pathLst_s[0][0][pathLst_s[0][0].size()-1].c = path[2].c;
//    path2.push_back(pathLst_s[0][0][0]);
//    path2.push_back(pathLst_s[0][0][pathLst_s[0][0].size()/2]);
//    path2.push_back(pathLst_s[0][0][pathLst_s[0][0].size()-1]);
//    interpolation(pathLst_s[0][0], path2);

//    // 读配置文件
//    vector<vector<string>> nameLsts;
//    vector<vector<double>> region;
//    vector<int> direction;
//    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead, directionLst;
//    vector<vector<vector<int>>> directionLst_s;
//    readConfig(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead, region, nameLsts);
//    for(int i=0; i<RelateIndex.size(); i++)
//    {
//        for(int j=0; j<RelateIndex[i].size(); j++)
//        {
//            direction = {pathDirection[i][j], laserDirection[i][j], rotateYHead[i][j], rotateYEnd[i][j]};
//            directionLst.push_back(direction);
//        }
//        directionLst_s.push_back(directionLst);
//        directionLst.clear();
//    }
//    double dx, dy, dz, da, db;
//    dx = (region[0][2]-region[0][1])/region[0][0];
//    dy = (region[1][2]-region[1][1])/region[1][0];
//    dz = (region[2][2]-region[2][1])/region[2][0];
//    da = (region[3][2]-region[3][1])/region[3][0];
//    db = (region[4][2]-region[4][1])/region[4][0];

//    // 转换为工件坐标系下的位姿
//    pathLst_s2 = pathLst_s;
//    for(auto &r0 : pathLst_s2)
//        for(auto &r1 : r0)
//            posInTool(r1, posW1);

//    // 转换为零点下状态时基标系下的位姿
//    for(auto &r0 : pathLst_s)
//        for(auto &r1 : r0)
//            posInBase(r1, posW1, posW2);

//    RobotPos cpos = {0,0,0,0,0,0};
//    pathLst = pathLst_s2[0];
//    for(int i=1; i<12; i++)
//    {
//        for(auto &r0 : pathLst)
//           for(auto &r1 : r0)
//               PosRotate(cpos, r1, 30);
//        pathLst_s2.push_back(pathLst);
//    }
//    pathLst_s3 = pathLst_s2;

//    // 焊缝整理
//    RepeatData *Data = new RepeatData;
//    Zitai_Info Zitai;
//    RepeatSeamArray seam_s;
//    Seaminfo seam;
//    RepeatPos posLst1, posLst2;
//    Zitai.spacing1 = 10;
//    Zitai.spacing2 = 10;
//    Zitai.distance1 = 10;
//    Zitai.distance2 = 10;
//    int orderIndex = 0;
//    for(int i=0; i<12; i++)
//    {
//        for(int k=0; k<pathLst_s2[i].size(); k++)
//           CreateKeelScan(pathLst_s3[i][k]);  //扫描龙骨数据
//        for(int k=0; k<pathLst_s2[i].size(); k++)
//        {
//           for(int ki=0; ki<pathLst_s2[i][k].size(); ki++)
//           {
//               RobotPos p = pathLst_s2[i][k][ki];
//               posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           for(int ki=0; ki<pathLst_s3[i][k].size(); ki++)
//           {
//               RobotPos p = pathLst_s3[i][k][ki];
//               posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           seam.scanePos = posLst1;
//           seam.framePos = posLst2;
//           seam.seamName = "seam"+std::to_string(i+1)+"_2_0";
//           seam.seamIndex = 0;
//           seam.orderIndex = 0;
//           seam.bind1 = 0;
//           seam.bind2 = 0;
//           seam_s.push_back(seam);
//           posLst1.clear();
//           posLst2.clear();
//        }


//        Zitai.cadid = CadName;
//        Zitai.posArray = seam_s;
//        seam_s.clear();
//        Zitai.pos_motor_start = 850;
//        Zitai.pos_motor_stop = 850;
//        Zitai.angle1_motor_start = 0;
//        Zitai.angle1_motor_stop = 0;
//        Zitai.angle2_motor_start = 24+i*30;
//        Zitai.angle2_motor_stop = 24+i*30;
//        Zitai.motor1speed = 0;
//        Zitai.motor2speed = 0;
//        Zitai.motor3speed = 0;
//        Data->push_back(Zitai);
//    }
//    std::string ret = InputControl::CreatSerial(*Data);

}

void MainWindow::on_pushButton_5_clicked()
{
//    RobotPos pos = {-441.493, 890.126, 926.022, 164.456, 176.419, -100.478};
//    vector<RobotPos> path;
//    path.push_back(pos);
//    vector<RobotAxle> agllst;
//    bool limit;
//    limit = Pos2Joint(path, agllst, agl_acl, TInF);

    vector<vector<RobotPos>> pathLst;
    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3, pathLst_s4;
    vector<vector<int>> WeldClassifyPlan;
    Read_pathlst(pathLst, filename);

    // 焊缝关联信息，关联分类
    vector<vector<int>> RelateIndex;
//    RelateIndex = {{3}, {5}, {7,8}, {9,10}, {11,12,13}, {0,31}};
//    RelateIndex = {{14,15,16}};
//    RelateIndex = {{17,18,19,20}};
//    RelateIndex = {{0,31}};
//    RelateIndex = {{2}};
//    RelateIndex = {{0}, {1}};
//    RelateIndex = {{4}, {5}};
//    RelateIndex = {{6}, {7}};
//    RelateIndex = {{10}, {11}};
    RelateIndex = {{4}, {5}, {10}, {11}};
    RelateProcess3(pathLst, RelateIndex, pathLst_s);

    // 读配置文件
    vector<vector<string>> nameLsts;
    vector<vector<double>> region;
    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead;
    readConfig(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead, region, nameLsts);
    double dx, dy, dz, da, db;
    dx = (region[0][2]-region[0][1])/region[0][0];
    dy = (region[1][2]-region[1][1])/region[1][0];
    dz = (region[2][2]-region[2][1])/region[2][0];
    da = (region[3][2]-region[3][1])/region[3][0];
    db = (region[4][2]-region[4][1])/region[4][0];

    // 根据焊缝附带信息调节焊缝姿态
    for(int i=0; i<pathLst_s.size(); i++)
    {
        for(int j=0; j<pathLst_s[i].size(); j++)
        {
            if(pathDirection[i][j]==1){
                pathReverse(pathLst_s[i][j]);
                laserReverse(pathLst_s[i][j]);
            }

            if(laserDirection[i][j]==1){
                laserReverse(pathLst_s[i][j]);
                posRotateY(pathLst_s[i][j], aglInclude-aglWeld);
            }
            else{
                posRotateY(pathLst_s[i][j], aglInclude+aglWeld);
            }

            if(rotateYEnd[i][j]==1){
                if(laserDirection[i][j]==1){
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,5,0);
                }
                else{
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,5,1);
                }
            }

            if(rotateYHead[i][j]==1){
                if(laserDirection[i][j]==1){
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,5,1);
                }
                else{
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,5,0);
                }
            }
        }
    }

    // 转换为工件坐标系下的位姿
//    RobotPos posW1, posO, posX;
//    posO = {-62, 2017.5, 551, 0,0,0};
//    posX = {-640, 1691, 551, 0,0,0};
//    WorkpieceCoordinates(posO, posX, posW1);
    pathLst_s2 = pathLst_s;
    for(auto &r0 : pathLst_s2)
        for(auto &r1 : r0)
            posInTool(r1, posW1);
    pathLst_s4 = pathLst_s2;

    // 添加回撤路径
    for(int i=0; i<pathLst_s.size(); i++)
        for(int j=0; j<pathLst_s[i].size(); j++)
            RetracementProcess(pathLst_s[i][j], backDistance, backPosNum);

    // 转换为零点下状态时基标系下的位姿
    for(auto &r0 : pathLst_s)
        for(auto &r1 : r0)
            posInBase(r1, posW1, posW2);

   // 非联动处理
//   WeldClassify7(pathLst_s, region, WeldClassifyPlan);
//   WeldClassify8(pathLst_s, region, WeldClassifyPlan, laserDirection, rotateY);
    WeldClassifyPlan.push_back({0,0,0,0,0,1,0,0,0});
    WeldClassifyPlan.push_back({0,0,0,0,0,0,1,0,0});
    WeldClassifyPlan.push_back({0,0,0,0,0,0,0,1,0});
    WeldClassifyPlan.push_back({0,0,0,0,0,0,0,0,1});


//   // 焊缝整理
//   RepeatData *Data = new RepeatData;
//   Zitai_Info Zitai;
//   RepeatSeamArray seam_s;
//   Seaminfo seam;
//   RepeatPos posLst1, posLst2;
//   Zitai.spacing1 = 10;
//   Zitai.spacing2 = 10;
//   Zitai.distance1 = 10;
//   Zitai.distance2 = 10;
//   int orderIndex = 0;
//   for(int i=0; i<WeldClassifyPlan.size(); i++)
//   {
//       pathLst_s3 = pathLst_s2;
//       int kk = 0;
//       for(int j=5; j<WeldClassifyPlan[i].size(); j++)
//       {
//           if(WeldClassifyPlan[i][j]==1)
//           {
//               kk++;
//               orderIndex++;
//               for(int k=0; k<pathLst_s2[j-5].size(); k++)
//                   CreateKeelScan(pathLst_s3[j-5][k]);  //扫描龙骨数据
//               for(int k=0; k<pathLst_s2[j-5].size(); k++)
//               {
//                   for(int ki=0; ki<pathLst_s2[j-5][k].size(); ki++){
//                       RobotPos p = pathLst_s2[j-5][k][ki];
//                       posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//                   }
//                   for(int ki=0; ki<pathLst_s3[j-5][k].size(); ki++){
//                       RobotPos p = pathLst_s3[j-5][k][ki];
//                       posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//                   }
//                   seam.scanePos = posLst1;
//                   seam.framePos = posLst2;
//                   seam.seamName = nameLsts[j-5][k];
//                   seam.seamIndex = 0;
//                   seam.orderIndex = 0;
//                   seam.bind1 = 0;
//                   seam.bind2 = 0;
//                   seam_s.push_back(seam);
//                   posLst1.clear();
//                   posLst2.clear();
//               }
//           }
//       }
//       Zitai.cadid = CadName;
//       Zitai.posArray = seam_s;
//       seam_s.clear();
//       Zitai.pos_motor_start = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
//       Zitai.pos_motor_stop = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
//       Zitai.angle1_motor_start = region[4][1]+WeldClassifyPlan[i][4]*db;
//       Zitai.angle1_motor_stop = region[4][1]+WeldClassifyPlan[i][4]*db;
//       Zitai.angle2_motor_start = -(region[3][1]+WeldClassifyPlan[i][3]*da);
//       Zitai.angle2_motor_stop = -(region[3][1]+WeldClassifyPlan[i][3]*da);
//       Zitai.motor1speed = 0;
//       Zitai.motor2speed = 0;
//       Zitai.motor3speed = 0;
//       Data->push_back(Zitai);
//   }
//   std::string ret = InputControl::CreatSerial(*Data);
}


//void MainWindow::on_pushButton_6_clicked()
//{
//    //    vector<vector<RobotPos>> pathLst;
//    //    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;
//    //    QString filename = "pathlstAll.txt";
//    //    Read_pathlst(pathLst, filename);
//    //    vector<vector<double>> region;
//    //    vector<vector<int>> WeldClassifyPlan;
//    //    region.push_back({20,200,-800});
//    //    region.push_back({1,0,0});
//    //    region.push_back({1,0,0});
//    //    region.push_back({144,-180,180});
//    //    region.push_back({1,0,0});
//    //    double dx, dy, dz, da, db;
//    //    dx = (region[0][2]-region[0][1])/region[0][0];
//    //    dy = (region[1][2]-region[1][1])/region[1][0];
//    //    dz = (region[2][2]-region[2][1])/region[2][0];
//    //    da = (region[3][2]-region[3][1])/region[3][0];
//    //    db = (region[4][2]-region[4][1])/region[4][0];

//    //    // 焊缝关联信息，关联分类
//    //    vector<vector<int>> RelateIndex;
//    //    RelateIndex = {{0,1,2},{3}, {5}, {7,8}, {9,10}, {11,12,13}, {14,15,16}, {17,18,19,20}, {21,22}, {23,24}};
//    //    RelateProcess3(pathLst, RelateIndex, pathLst_s);

//    //    // 焊缝附带信息
//    //    vector<vector<int>> pathDirection, laserDirection, laserDirection2, rotateY;
//    //    pathDirection =   {{0,0,0},{0}, {0}, {0,0}, {0,0}, {0,0,0}, {0,0,0}, {0,0,0,0}, {0,0}, {0,0}};//是否需要改变路径方向
//    //    laserDirection =  {{1,1,1},{1}, {1}, {0,1}, {0,1}, {0,0,0}, {0,0,0}, {1,1,1,1}, {0,0}, {0,0}};//是否可以改变激光面方位
//    //    laserDirection2 = {{0,0,0},{0}, {0}, {0,0}, {0,0}, {0,0,0}, {0,0,0}, {0,0,0,0}, {0,0}, {0,0}};//是否已经改变激光面方位
//    //    rotateY =         {{0,0,0},{0}, {0}, {1,1}, {1,1}, {1,1,1}, {1,1,1}, {0,0,0,0}, {1,1}, {1,1}};//是否需要调整焊枪姿态

//    vector<vector<RobotPos>> pathLst;
//    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3, pathLst_s4;
//    vector<vector<int>> WeldClassifyPlan;
//    Read_pathlst(pathLst, filename);

//    // 焊缝关联信息，关联分类
//    vector<vector<int>> RelateIndex;
//    RelateIndex = {{4}};
//    RelateProcess3(pathLst, RelateIndex, pathLst_s);

//    // 读配置文件
//    vector<vector<string>> nameLsts;
//    vector<vector<double>> region;
//    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead;
//    readConfig(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead, region, nameLsts);
//    double dx, dy, dz, da, db;
//    dx = (region[0][2]-region[0][1])/region[0][0];
//    dy = (region[1][2]-region[1][1])/region[1][0];
//    dz = (region[2][2]-region[2][1])/region[2][0];
//    da = (region[3][2]-region[3][1])/region[3][0];
//    db = (region[4][2]-region[4][1])/region[4][0];


//    // 转换为工件坐标系下的位姿
//    RobotPos posW1, posO, posX;
//    posO = {-62, 2017.5, 551, 0,0,0};
//    posX = {-640, 1691, 551, 0,0,0};
//    WorkpieceCoordinates(posO, posX, posW1);
//    pathLst_s2 = pathLst_s;
//    for(auto &r0 : pathLst_s2)
//        for(auto &r1 : r0)
//            posInTool(r1, posW1);
//    pathLst_s4 = pathLst_s2;

//    // 添加回撤路径
//    for(int i=0; i<pathLst_s.size(); i++)
//        for(int j=0; j<pathLst_s[i].size(); j++)
//            RetracementProcess(pathLst_s[i][j], 50, backPosNum);

//    // 转换为零点下状态时基标系下的位姿
//    for(auto &r0 : pathLst_s)
//        for(auto &r1 : r0)
//            posInBase(r1, posW1, posW2);

//   // 非联动处理
//   WeldClassify7(pathLst_s, region, WeldClassifyPlan);
////   WeldClassify8(pathLst_s, region, WeldClassifyPlan, laserDirection, rotateY);


//   // 焊缝整理
//   RepeatData *Data = new RepeatData;
//   Zitai_Info Zitai;
//   RepeatSeamArray seam_s;
//   Seaminfo seam;
//   RepeatPos posLst1, posLst2;
//   Zitai.spacing1 = 10;
//   Zitai.spacing2 = 10;
//   Zitai.distance1 = 10;
//   Zitai.distance2 = 10;
//   int orderIndex = 0;
//   for(int i=0; i<WeldClassifyPlan.size(); i++)
//   {
//       pathLst_s3 = pathLst_s2;
//       int kk = 0;
//       for(int j=5; j<WeldClassifyPlan[i].size(); j++)
//       {
//           if(WeldClassifyPlan[i][j]==1)
//           {
//               kk++;
//               orderIndex++;
//               for(int k=0; k<pathLst_s2[j-5].size(); k++)
//                   CreateKeelScan(pathLst_s3[j-5][k]);  //扫描龙骨数据
//               for(int k=0; k<pathLst_s2[j-5].size(); k++)
//               {
//                   for(int ki=0; ki<pathLst_s2[j-5][k].size(); ki++){
//                       RobotPos p = pathLst_s2[j-5][k][ki];
//                       posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//                   }
//                   for(int ki=0; ki<pathLst_s3[j-5][k].size(); ki++){
//                       RobotPos p = pathLst_s3[j-5][k][ki];
//                       posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//                   }
//                   seam.scanePos = posLst1;
//                   seam.framePos = posLst2;
//                   seam.seamName = nameLsts[j-5][k];
//                   seam.seamIndex = j-4;
//                   seam.orderIndex = orderIndex;
//                   seam.bind1 = 0;
//                   seam.bind2 = 0;
//                   seam_s.push_back(seam);
//                   posLst1.clear();
//                   posLst2.clear();
//               }
//           }
//       }
//       Zitai.cadid = CadName;
//       Zitai.posArray = seam_s;
//       seam_s.clear();
//       Zitai.pos_motor_start = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
//       Zitai.pos_motor_stop = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
//       Zitai.angle1_motor_start = region[4][1]+WeldClassifyPlan[i][4]*db;
//       Zitai.angle1_motor_stop = region[4][1]+WeldClassifyPlan[i][4]*db;
//       Zitai.angle2_motor_start = -(region[3][1]+WeldClassifyPlan[i][3]*da);
//       Zitai.angle2_motor_stop = -(region[3][1]+WeldClassifyPlan[i][3]*da);
//       Zitai.motor1speed = 0;
//       Zitai.motor2speed = 0;
//       Zitai.motor3speed = 0;
//       Data->push_back(Zitai);
//   }
//   std::string ret = InputControl::CreatSerial(*Data);

//   // 写入文件
//   vector<RobotAxle> AglLst;
//   vector<vector<RobotAxle>> AglLst_s;
//   vector<vector<vector<RobotAxle>>> AglLst_ss;
//   vector<vector<RobotPos>> pathLst2, pathLst3;
//   vector<vector<vector<RobotPos>>> PathLst_ss, PathLst_ss2;
//   bool limit;
//   for(int i=0; i<WeldClassifyPlan.size(); i++)
//   {
//       for(int j=5; j<WeldClassifyPlan[i].size(); j++)
//       {
//           if(WeldClassifyPlan[i][j]==1)
//           {
//               for(int k=0; k<(int)pathLst_s[j-5].size(); k++)
//               {
//                   pathLst2.push_back(pathLst_s[j-5][k]);
//                   pathLst3.push_back(pathLst_s2[j-5][k]);
//               }
//           }
//       }
//       for(int j=0; j<pathLst2.size(); j++)
//       {
//           for(auto &r : pathLst2[j]){
//               PosRotate(cposa, r, region[3][1]+WeldClassifyPlan[i][3]*da);
//               PosRotate(cposb, r, region[4][1]+WeldClassifyPlan[i][4]*db);
//               r.x -= (region[0][1]+WeldClassifyPlan[i][0]*dx);
//               r.y -= (region[1][1]+WeldClassifyPlan[i][1]*dy);
//               r.z -= (region[2][1]+WeldClassifyPlan[i][2]*dz);
//           }
//           limit = Pos2Joint(pathLst2[j], AglLst, agl_acl, TInF);
//           AglLst_s.push_back(AglLst);
//           AglLst.clear();
//       }
//       PathLst_ss.push_back(pathLst2);
//       pathLst2.clear();
//       PathLst_ss2.push_back(pathLst3);
//       pathLst3.clear();
//       AglLst_ss.push_back(AglLst_s);
//       AglLst_s.clear();
//   }

//   vector<double> vec;
//   vector<vector<double>> platform;
//   QString filename2 = "Data.txt";
//   for(int i=0; i<WeldClassifyPlan.size(); i++){
//       vec.push_back(-region[0][1]-WeldClassifyPlan[i][0]*dx);
//       vec.push_back(region[1][1]+WeldClassifyPlan[i][1]*dy);
//       vec.push_back(region[2][1]+WeldClassifyPlan[i][2]*dz);
//       vec.push_back(-region[3][1]-WeldClassifyPlan[i][3]*da);
//       vec.push_back(region[4][1]+WeldClassifyPlan[i][4]*db);
//       platform.push_back(vec);
//       vec.clear();
//   }
//   Write_File3(PathLst_ss2, PathLst_ss, AglLst_ss, platform, filename2);
//   PathLst_ss.clear();
//}



void MainWindow::getSeamType(const string file, vector<string> &seamTypeLst)
{
    ifstream logFile(file);        // 构造一个文件流读取对象
    string str, str7, keyStr1, keyStr2;
    vector<string> strLst;
    keyStr1 = "seam2原始点:";
    keyStr2 = "吐出的seam0:";
    bool repeat;

    while (getline(logFile, str))  // 逐行读入数据
    {
        strLst.push_back(str);
        str.clear();
    }
    logFile.close();               // 关闭文件

    for(int i=0; i<strLst.size(); i++)
    {
        str = strLst[i];
        if(str==keyStr1)
        {
            repeat = false;
            str = strLst[i+1];
            str7 = str.substr(0,7);
            if(str.substr(0,7)=="current")
            {
                for(int j=0; j<seamTypeLst.size(); j++)
                    if(str==seamTypeLst[j])
                        repeat = true;
                if(!repeat)
                    seamTypeLst.push_back(str);
            }
        }
    }
}


int MainWindow::getSeamTypeFromFolder(string path, vector<string> &seamTypeLst)
{
    struct _finddata_t fileinfo;  // 存储文件信息的结构体对象
    string strFile = path + "*.log";

    //遍历目录系统函数要求先尝试寻找一个文件，看是否存在
    long handle;
    if ((handle = _findfirst(strFile.c_str(), &fileinfo)) == -1L)
    {
        return 0;  // 如果查询log文件失败，直接返回
    }
    else
    {
        strFile = path + fileinfo.name;
        getSeamType(strFile, seamTypeLst);
        //直遍历，直到所有.log文件得到加载与处理
        while (!(_findnext(handle, &fileinfo)))
        {
            strFile = path + fileinfo.name;
            getSeamType(strFile, seamTypeLst);
        }

        _findclose(handle);  // 释放遍历目录的句柄
    }
}


void MainWindow::getSeamData(const string file, string seamType, vector<vector<RobotPos>> &pathLst)
{
    ifstream logFile(file);        // 构造一个文件流读取对象
    string str, keyStr1, keyStr2, buf;
    vector<string> strLst;
    keyStr1 = "seam2原始点:";
    keyStr2 = "吐出的seam0:";
    bool wetherRead1 = false;
    bool wetherRead2 = false;
    vector<string> list;
    RobotPos pos;
    vector<RobotPos> path;

    while (getline(logFile, str))  // 逐行读入数据
    {
        strLst.push_back(str);
        str.clear();
    }
    logFile.close();               // 关闭文件

    for(int i=0; i<strLst.size(); i++)
    {
        str = strLst[i];
        if(wetherRead2)
        {
            stringstream ss(str);
            while (ss >> buf)
                list.push_back(buf);
            pos.x = atof(list[0].c_str());
            pos.y = atof(list[1].c_str());
            pos.z = atof(list[2].c_str());
            pos.a = atof(list[3].c_str());
            pos.b = atof(list[4].c_str());
            pos.c = atof(list[5].c_str());
            path.push_back(pos);
            list.clear();
        }
        if(str==seamType)
            wetherRead1 = true;
        if(wetherRead1)
            if(str==keyStr2)
                wetherRead2 = true;
    }
    if(wetherRead2)
        pathLst.push_back(path);
}


int MainWindow::getSeamDataFromFolder(string path, string seamType, vector<vector<RobotPos>> &pathLst)
{
    vector<string> seamTypeLst;
    struct _finddata_t fileinfo;
    string strFile = path + "*.log";

    //遍历目录系统函数要求先尝试寻找一个文件，看是否存在
    long handle;
    if ((handle = _findfirst(strFile.c_str(), &fileinfo)) == -1L)
    {
        return 0;
    }
    else
    {
        strFile = path + fileinfo.name;
        getSeamData(strFile, seamType, pathLst);
        //直遍历，直到所有.log文件得到加载与处理
        while (!(_findnext(handle, &fileinfo)))
        {
            strFile = path + fileinfo.name;
            getSeamData(strFile, seamType, pathLst);
        }

        _findclose(handle);  // 释放遍历目录的句柄
    }
}

void MainWindow::on_pushButton_6_clicked()
{
    string path = "coorationlog/";
    QString fileName;
    vector<string> seamTypeLst;
    vector<vector<RobotPos>> pathLst;
    getSeamTypeFromFolder(path, seamTypeLst);
    for(int i=0; i<seamTypeLst.size(); i++)
    {
        getSeamDataFromFolder(path, seamTypeLst[i], pathLst);
        fileName = "客户/" +  QString::number(i) + ".txt";
        Write_File(pathLst, fileName);
        pathLst.clear();
    }
}
