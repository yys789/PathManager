#include "mainwindow2.h"
#include "ui_mainwindow2.h"
#include "dialog.h"
#include "ui_dialog.h"

MainWindow2::MainWindow2(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow2)
{
    ui->setupUi(this);
    interval = 5;
    CInT= {-84.729,-3.061,5.365, 0,0,0};
    agl_acl = {-15, -36, -132, 44, -62, -66};
}

MainWindow2::~MainWindow2()
{
    delete ui;
}


bool MainWindow2::PhaseJudgment(vector<RobotPos> path, RobotPos cpos)
{
    bool flag = false;
    Matrix4d Mc, Mp;
    RobotPos pos = path[path.size()/2-1];
    cpos.a = 0;
    cpos.b = 0;
    cpos.c = 0;
    pos2Matrix(cpos, Mc);
    pos2Matrix(pos, Mp);
    Mp = Mc.inverse()*Mp;
    Matrix2pos(Mp, pos);
    if(pos.x>0)
        flag = true;
    return flag;
}


void MainWindow2::cposInZ(RobotPos &cpos, double posX)
{
    Eigen::Matrix4d Mc;
    Eigen::Vector3d Vz;
    pos2Matrix(cpos, Mc);
    Vz << Mc(0,2),Mc(1,2),Mc(2,2);
    Vz = Vz/sqrt(Vz.dot(Vz));

//    double Dis = (posX-cpos.x)/Vz(0);
//    cpos.x = posX;
//    cpos.y = cpos.y + Dis*Vz(1);
//    cpos.z = cpos.z + Dis*Vz(2);

    double Dis = (posX-cpos.y)/Vz(1);
    cpos.x = cpos.x + Dis*Vz(0);
    cpos.y = posX;
    cpos.z = cpos.z + Dis*Vz(2);
}


// 写入数据库
void MainWindow2::Write2Database_Seek(RepeatData &Data, vector<vector<int>> WeldClassifyPlan,
                                      double dx, double da,
                                      vector<vector<double>> region, vector<vector<string>> nameLsts,
                                      vector<vector<int>> &orderIDLsts,
                                      vector<vector<int>> &rankLsts,
                                      vector<vector<int>> &TracingLsts,
                                      vector<vector<vector<RobotPos>>> pathLst_sS,
                                      vector<vector<vector<RobotPos>>> pathLst_sK)
{
    AScanePos sPos;
    AFramePos fPos;
    QList<AScanePos> scanePos;
    QList<AFramePos> framePos;
    SeamInfo seam;
    QList<SeamInfo> seams;
    MachineGrond  machine;
    double db = 0;
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
                    for(int ki=0; ki<pathLst_sS[j-5][k].size(); ki++)
                    {
                        RobotPos p = pathLst_sS[j-5][k][ki];
                        sPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
                        sPos.index1 = ki;
                        if(pathLst_sS[j-5][k].size()<=5)
                            sPos.enabled = 1;
                        else if(pathLst_sS[j-5][k].size()<=10)
                        {
                            if(ki==0 || ki==pathLst_sS[j-5][k].size()-1)
                                sPos.enabled = 0;
                        }
                        else if(pathLst_sS[j-5][k].size()<=20)
                        {
                            if(ki==0 || ki==1
                               || ki==pathLst_sS[j-5][k].size()-2
                               || ki==pathLst_sS[j-5][k].size()-1)
                                sPos.enabled = 0;
                        }
                        else
                        {
                            if(ki==0 || ki==1 || ki==2
                               || ki==pathLst_sS[j-5][k].size()-3
                               || ki==pathLst_sS[j-5][k].size()-2
                               || ki==pathLst_sS[j-5][k].size()-1)
                                sPos.enabled = 0;
                        }
                        scanePos.push_back(sPos);
                    }
                    for(int ki=0; ki<pathLst_sK[j-5][k].size(); ki++)
                    {
                        RobotPos p = pathLst_sK[j-5][k][ki];
                        fPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
                        fPos.index1 = ki;
                        framePos.push_back(fPos);
                    }
                    seam.scanePos = scanePos;
                    seam.framePos = framePos;
                    seam.seamName = nameLsts[j-5][k];
                    seam.orderIndex = orderIDLsts[j-5][k];
                    seam.rank = rankLsts[j-5][k];
                    seam.tracing = TracingLsts[j-5][k];
                    machine.pos_motor_start = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
                    machine.pos_motor_stop = -(region[0][1]+WeldClassifyPlan[i][0]*dx);
                    machine.angle1_motor_start = region[4][1]+WeldClassifyPlan[i][4]*db;
                    machine.angle1_motor_stop = region[4][1]+WeldClassifyPlan[i][4]*db;
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


// 写入数据库
void MainWindow2::Write2Database_Track(RepeatData &Data, vector<vector<double>> StateWithSpeedLst,
                                       vector<int> index, vector<vector<string>> nameLsts,
                                       vector<vector<int>> &orderIDLsts,
                                       vector<vector<int>> &rankLsts,
                                       vector<vector<int>> &TracingLsts,
                                       vector<double> weldTimeLst,
                                       vector<vector<vector<RobotPos>>> pathLst_sS,
                                       vector<vector<vector<RobotPos>>> pathLst_sK,
                                       double aglG,
                                       int splitNumS,
                                       int splitNumE)
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
               fPos.angle1 = 0;
               fPos.angle2 = 0;
               if(ki<splitNumS)
                   fPos.angle3 = 0;
               else if(ki>splitNumE)
                   fPos.angle3 = -aglG;
               else
                   fPos.angle3 = -aglG * (ki-splitNumS)/(splitNumE-splitNumS);
               framePos.push_back(fPos);
           }
           seam.scanePos = scanePos;
           seam.framePos = framePos;
           seam.seamName = nameLsts[i][k];
           seam.orderIndex = orderIDLsts[i][k];
           seam.rank = rankLsts[i][k];
           seam.tracing = TracingLsts[i][k];
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
               seam.orderIndex = orderIDLsts[i][k];
               seam.rank = rankLsts[i][k];
               seam.tracing = TracingLsts[i][k];
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


void MainWindow2::Write2Database_FangGuan(RepeatData &Data, vector<vector<double>> StateWithSpeedLst,
                                       vector<int> index, vector<vector<string>> nameLsts,
                                       vector<vector<int>> &orderIDLsts,
                                       vector<vector<int>> &rankLsts,
                                       vector<vector<int>> &TracingLsts,
                                       vector<double> weldTimeLst,
                                       vector<vector<vector<RobotPos>>> pathLst_sS,
                                       vector<vector<vector<RobotPos>>> pathLst_sK,
                                       double aglG,
                                       int splitNumS,
                                       int splitNumE)
{
    AScanePos sPos;
    AFramePos fPos;
    QList<AScanePos> scanePos;
    QList<AFramePos> framePos;
    SeamInfo seam;
    MachineGrond  machine;
    vector<RobotPos> path;

    for(int i=0; i<StateWithSpeedLst.size(); i++)
    {
       // 跟踪焊缝
       RetracementProcess(pathLst_sS[index[i]][0], 200, 1);
       path.push_back(pathLst_sS[index[i]][0][1]);
       path.push_back(pathLst_sS[index[i]][0][pathLst_sS[index[i]][0].size()-2]);
       path.push_back(pathLst_sS[index[i]][0][pathLst_sS[index[i]][0].size()-1]);
       KeeldataReorganize(pathLst_sK[index[i]][0]);
       for(int k=0; k<1; k++)
       {
           for(int ki=0; ki<path.size(); ki++){
               RobotPos p = path[ki];
               sPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
               sPos.index1 = ki;
               scanePos.push_back(sPos);
           }
           for(int ki=0; ki<pathLst_sK[index[i]][k].size(); ki++){
               RobotPos p = pathLst_sK[index[i]][k][ki];
               fPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
               fPos.index1 = ki;
               fPos.angle1 = 0;
               fPos.angle2 = 0;
               if(ki<splitNumS)
                   fPos.angle3 = 0;
               else if(ki>splitNumE)
                   fPos.angle3 = -aglG;
               else
                   fPos.angle3 = -aglG * (ki-splitNumS)/(splitNumE-splitNumS);
               framePos.push_back(fPos);
           }
           seam.scanePos = scanePos;
           seam.framePos = framePos;
           seam.seamName = nameLsts[i][k];
           seam.orderIndex = orderIDLsts[i][k];
           seam.rank = rankLsts[i][k];
           seam.tracing = TracingLsts[i][k];
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
    }
}


// 读配置文件
void MainWindow2::readConfig(string configPath,
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


void MainWindow2::readConfig2(string configPath,
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



void MainWindow2::readConfig3(string configPath,
                            vector<vector<int>> RelateIndex,
                            vector<vector<int>> &pathDirection,
                            vector<vector<int>> &laserDirection,
                            vector<vector<int>> &rotateYEnd,
                            vector<vector<int>> &rotateYHead,
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
}


// 读配置文件(读取状态)
void MainWindow2::readConfig_rgn(string configPath, string name, vector<vector<double>> &region)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
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


// 读配置文件(读取角度)
void MainWindow2::readConfig_agl(string configPath, string name, double &aglWeld, double &aglInclude, double &aglHead, double &aglEnd, double &aglA, double &aglT)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    aglWeld = child.get<double>("aglWeld");
    aglInclude = child.get<double>("aglInclude");
    aglHead = child.get<double>("aglHead");
    aglEnd = child.get<double>("aglEnd");
    aglA = child.get<double>("aglA");
    aglT = child.get<double>("aglT");
}


// 读配置文件(RobotPos)
void MainWindow2::readConfig_RobotPos(string configPath, string name, RobotPos &pos)
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
void MainWindow2::readConfig_Str(string configPath, string name, string petName, string &Str)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    Str = child.get<string>(petName);
}


// 读配置文件(读取double)
void MainWindow2::readConfig_Dou(string configPath, string name, string petName, double &Dou)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    Dou = child.get<double>(petName);
}


// 读配置文件(读取int)
void MainWindow2::readConfig_Int(string configPath, string name, string petName, int &Int)
{
    boost::property_tree::ptree pt, child;
    read_info(configPath, pt);
    child = pt.get_child(name);
    Int = child.get<int>(petName);
}


//读取焊缝关联关系
void MainWindow2::readConfig_Seam(string configPath, vector<vector<vector<int>>> &RelateIndexSeek, vector<vector<vector<int>>> &RelateIndexTrack)
{
   boost::property_tree::ptree pt, child1, child2, child3;
   vector<string> state, relation, seam;
   string seamID;
   int index;
   vector<int> indexLst;
   vector<vector<int>> indexLsts;
   bool finish_i, finish_j;
//   state = {"state1","state2","state3","state4","state5","state6","state7","state8","state9","state10"};
//   relation = {"relation1","relation2","relation3","relation4","relation5","relation6","relation7","relation8","relation9","relation10",};
   for(int i=0; i<100; i++)
   {
       state.push_back("state"+std::to_string(i+1));
   }
   for(int i=0; i<100; i++)
   {
       relation.push_back("relation"+std::to_string(i+1));
   }
   for(int i=0; i<100; i++)
   {
       seam.push_back("seam"+std::to_string(i+1));
   }
//   seam = {"seam1","seam2","seam3","seam4","seam5","seam6","seam7","seam8","seam9","seam10"};
   read_info(configPath, pt);

   child1 = pt.get_child("Seek_Relation");
   finish_i = false;
   finish_j = false;
   for(int i=0; i<100; i++)
   {
       finish_j = false;
       if(finish_i)
           break;
       child2 = child1.get_child(state[i]);
       for(int j=0; j<100; j++)
       {
           if(finish_i || finish_j)
               break;
           child3 = child2.get_child(relation[j]);
           for(int k=0; k<100; k++)
           {
               seamID = child3.get<string>(seam[k]);
               if(j==0 && k==0 && seamID=="end")
               {
                   finish_i = true;
                   break;
               }
               else if(k==0 && seamID=="end")
               {
                   RelateIndexSeek.push_back(indexLsts);
                   indexLsts.clear();
                   finish_j = true;
                   break;
               }
               else if(seamID=="end")
               {
                   indexLsts.push_back(indexLst);
                   indexLst.clear();
                   break;
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
   finish_i = false;
   finish_j = false;
   for(int i=0; i<100; i++)
   {
       finish_j = false;
       if(finish_i)
           break;
       child2 = child1.get_child(state[i]);
       for(int j=0; j<100; j++)
       {
           if(finish_i || finish_j)
               break;
           child3 = child2.get_child(relation[j]);
           for(int k=0; k<100; k++)
           {
               seamID = child3.get<string>(seam[k]);
               if(j==0 && k==0 && seamID=="end")
               {
                   finish_i = true;
                   break;
               }
               else if(k==0 && seamID=="end")
               {
                   RelateIndexTrack.push_back(indexLsts);
                   indexLsts.clear();
                   finish_j = true;
                   break;
               }
               else if(seamID=="end")
               {
                   indexLsts.push_back(indexLst);
                   indexLst.clear();
                   break;
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



// 寻找船型焊起始相位角
void MainWindow2::findStartAngle(RobotPos pos, vector<double> &rotateRegion, vector<double> &flipRegion)
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
void MainWindow2::RetracementProcess(vector<RobotPos> &path, double distance, int backPosNum)
{
    int posNum = backPosNum;
    double posInterval;
    if(posNum != 0)
    {
        posInterval = -distance/posNum;
    }
    else
        posInterval = 0;
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


void MainWindow2::interpolation(vector<RobotPos> &seam, vector<RobotPos> pos_set)
{
    Eigen::Matrix4d r, Rz, Rz1, Ry, Rz2;
    vector<Eigen::Matrix4d> R, R1;
    double pi = M_PI;
    bool sign_n1;

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

        sign_n1 = false;
        double abs0 = 0.001;
        for(int i=0; i<(int)seam.size(); i++){
            if(abs(pos_set[j].x-seam[i].x)<abs0 && abs(pos_set[j].y-seam[i].y)<abs0 && abs(pos_set[j].z-seam[i].z)<abs0)
            {
                n1 = i;
                sign_n1 = true;
            }
            if(sign_n1==true && abs(pos_set[j+1].x-seam[i].x)<abs0 && abs(pos_set[j+1].y-seam[i].y)<abs0 && abs(pos_set[j+1].z-seam[i].z)<abs0){
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


void MainWindow2::Matrix2pos(Eigen::Matrix4d &m,  RobotPos &pos)
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


void MainWindow2::pos2Matrix(const RobotPos &pos, Eigen::Matrix4d &m)
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


bool MainWindow2::Pos2Joint(vector<RobotPos> Pos_set, vector<RobotAxle> &agl_set, RobotAxle agl_acl, RobotPos AInB)
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
//            if(agl_set[agl_set.size()-1].a6 >= 180 || agl_set[agl_set.size()-1].a6 <= -180){
//                Limit = true;
//                break;
//            }
        }
    }

    return Limit;
}


bool MainWindow2::Pos2Joint2(vector<RobotPos> Pos_set, vector<RobotAxle> &agl_set, RobotAxle agl_acl, RobotPos AInB)
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

//            if(agl_set[agl_set.size()-1].a1 >= 45 || agl_set[agl_set.size()-1].a1 <= -60){
//                Limit = true;
//                break;
//            }
//            if(agl_set[agl_set.size()-1].a6 >= 180 || agl_set[agl_set.size()-1].a6 <= -180){
//                Limit = true;
//                break;
//            }
        }
    }

    return Limit;
}


void MainWindow2::Read_pathlst(vector<vector<RobotPos>> &PathLst, QString filename)
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


void MainWindow2::ReadPathLst(vector<vector<RobotPos>> &PathLst, QString filename)
{
    vector<QString> Qstr;
    QFile file1(filename);
    if (!file1.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream in(&file1);
    while (!in.atEnd()) {
        QString line = in.readLine();
        Qstr.push_back(line);
    }

    QStringList list;
    RobotPos pos;
    vector<RobotPos> path;
    for(int i=0; i<(int)Qstr.size(); i++)
    {
        list = Qstr[i].split(',');
        if(list.size()!=6)
        {
            PathLst.push_back(path);
            path.clear();
        }
        else
        {
            pos.x = list[0].toDouble();
            pos.y = list[1].toDouble();
            pos.z = list[2].toDouble();
            pos.a = list[3].toDouble();
            pos.b = list[4].toDouble();
            pos.c = list[5].toDouble();
            path.push_back(pos);
        }

        if(i==(int)Qstr.size()-1)
        {
            PathLst.push_back(path);
            path.clear();
        }
    }
}


void MainWindow2::readPath(QString filename, vector<RobotPos> &path)
{
    path.clear();
    vector<QString> Qstr;
    QFile file1(filename);
    if (!file1.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream in(&file1);
    while (!in.atEnd()){
        QString line = in.readLine();
        Qstr.push_back(line);
    }

    float x,y,z,a,b,c;
    RobotPos pos;
    QStringList list;
    for(int i=0; i<Qstr.size(); i++){
        list = Qstr[i].split(',');
        pos.x = list[0].toDouble();
        pos.y = list[1].toDouble();
        pos.z = list[2].toDouble();
        pos.a = list[3].toDouble();
        pos.b = list[4].toDouble();
        pos.c = list[5].toDouble();
        path.push_back(pos);
    }
}


void MainWindow2::PosRotate(RobotPos cpos, RobotPos &pos, double agl)
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


void MainWindow2::PosRotateZW(RobotPos cpos, RobotPos &pos, double agl)
{
    cpos.a = 0;
    cpos.b = 0;
    cpos.c = 0;
    PosRotate(cpos, pos, agl);
}


void MainWindow2::Write_File(vector<vector<RobotPos>> pathLst, QString filename)
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


void MainWindow2::WritePath(vector<RobotPos> path, QString filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    for(int j=0; j<(int)path.size(); j++)
        out << path[j].x << "," << path[j].y << "," << path[j].z << ","
            << path[j].a << "," << path[j].b << "," << path[j].c <<  "\n";
}


void MainWindow2::WriteAgl(vector<RobotAxle> AglLst, QString filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    for(int j=0; j<(int)AglLst.size(); j+=10)
        out << AglLst[j].a1 << "," << AglLst[j].a2 << "," << AglLst[j].a3 << ","
            << AglLst[j].a4 << "," << AglLst[j].a5 << "," << AglLst[j].a6 <<  "\n";
}


// 焊缝由焊接方向及激光方向得出相应的轨迹数据
void MainWindow2::Weld2Path(vector<RobotPos> &path, vector<int> direction)
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
            pathHeadRotate(path,aglWeld-aglHead,3,1);
        }
        else{
            pathHeadRotate(path,aglWeld-aglHead,3,0);
        }
    }

    if(direction[3]==1){
        if(direction[1]==1){
            pathEndRotate(path,aglEnd-aglWeld,3,0);
        }
        else{
            pathEndRotate(path,aglEnd-aglWeld,3,1);
        }
    }
}


// 同条焊缝由焊接方向及激光方向得出若干组轨迹数据
void MainWindow2::pathMode(vector<RobotPos> path, vector<vector<RobotPos>> &pathlst, vector<int> direction, vector<vector<int>> &directionLst)
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
bool MainWindow2::pathModeTest(vector<vector<RobotPos>> pathlst, vector<vector<int>> directionLst, vector<int> &direction, double &evaluate)
{
    double fun, posNum;
    vector<double> flst;
    bool limit1, limit2, limit3, pass;
    vector<RobotAxle> AglLst1, AglLst2;
    pass = false;
    for(int i=0; i<pathlst.size(); i++)
    {
        fun = 0;
        posNum = 0;
        limit1 = Pos2Joint2(pathlst[i], AglLst1, agl_acl, TInF);
        limit2 = Pos2Joint2(pathlst[i], AglLst2, agl_acl, CInF);
        limit3 = PhaseJudgment(pathlst[i], posW2);
        if(!limit1)
            fun += RobotAttitudEvaluate2(AglLst1, pathlst[i], aglBest);
        if(!limit2)
            fun += RobotAttitudEvaluate2(AglLst2, pathlst[i], aglBest);
        posNum += 2*pathlst[i].size();
        fun = fun/posNum;
        AglLst1.clear();
        AglLst2.clear();
        if(!limit1 && !limit2 && limit3)
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


void MainWindow2::WeldClassify9(vector<vector<vector<RobotPos>>> pathlst_s,
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


//// 船型焊
//void MainWindow2::ShipForm_Weld(vector<vector<RobotPos>> pathlst, vector<vector<double>> region, vector<vector<double>> &StateWithSpeedLst, double &weldTime, vector<int> Variables, double WeldSpeed, int pso_Num, int time, int splitNum)
//{
//    vector<double> x, v;
//    vector<vector<double>> pso_x, pso_v;
//    vector<double> pso_f, pso_f2;
//    vector<vector<double>> pso_pbestx;
//    vector<double> pso_pbestf, pso_gbestx;
//    double pso_gbestf, pso_gbestf2;
//    double Px, Pa, Pb, Vx, Va, Vb, dpx, dpa, dpb, dvx, dva, dvb;
//    Px = region[0][0];
//    Pa = region[3][0];
//    Pb = region[4][0];
//    Vx = region[5][0];
//    Va = region[8][0];
//    Vb = region[9][0];
//    dpx = (region[0][2]-region[0][1])/Px;
//    dpa = (region[3][2]-region[3][1])/Pa;
//    dpb = (region[4][2]-region[4][1])/Pb;
//    dvx = (region[5][2]-region[5][1])/Vx;
//    dva = (region[8][2]-region[8][1])/Va;
//    dvb = (region[9][2]-region[9][1])/Vb;
//    for(int i=0; i<Px; i++){
//        for(int j=0; j<Pa; j++){
//            for(int k=0; k<Pb; k++){
//                for(int m=0; m<Vx; m++){
//                    for(int n=0; n<Va; n++){
//                        for(int nn=0; nn<Vb; nn++){
//                           x.push_back(region[0][1]+i*dpx);
//                           x.push_back(region[3][1]+j*dpa);
//                           x.push_back(region[4][1]+k*dpb);
//                           x.push_back(region[5][1]+m*dvx);
//                           x.push_back(region[8][1]+n*dva);
//                           x.push_back(region[9][1]+nn*dvb);
//                           v = {0,0,0,0,0,0};
//                           pso_x.push_back(x);
//                           pso_v.push_back(v);
//                           pso_pbestx.push_back(x);
//                           pso_f.push_back(1e10);
//                           pso_f2.push_back(1e10);
//                           pso_pbestf.push_back(1e10);
//                           x.clear();
//                           v.clear();
//                        }
//                    }
//                }
//            }
//        }
//    }
//    for(int j=0; j<10; j++)
//        pso_gbestx.push_back(0);
//    pso_gbestf = 1e10;
//    pso_gbestf2 = 1e10;

//    double c = 0.5;
//    double c1 = 0.5;
//    double c2 = 0.5;
//    vector<RobotPos> PosLst, PosLst2;
//    double index = 1e10;
//    vector<int> indexLst;
//    vector<vector<RobotAxle>> AglLst_s;
//    if(pathlst.size()==1)
//        PosLst = pathlst[0];
//    else
//    {
//        if(pathlst[0].size()>pathlst[1].size()){
//            PosLst = pathlst[0];
//            PosLst2 = pathlst[1];
//        }
//        else{
//            PosLst = pathlst[1];
//            PosLst2 = pathlst[0];
//        }
//    }
//    for(int t=0; t<1; t++)
//    {
//        RobotPos pos;
//        vector<RobotPos> PosLst_correct, PosLst_correct2;
//        vector<RobotAxle> AglLst, AglLst2;
//        bool limit, limit1, limit2;
//        double timegap = sqrt(pow(PosLst[0].x-PosLst[1].x,2)
//                     +pow(PosLst[0].y-PosLst[1].y,2)
//                     +pow(PosLst[0].z-PosLst[1].z,2))/WeldSpeed;
//        weldTime = timegap*splitNum;
//        int jj = 0;
//        pso_Num = pso_x.size();
//        for(int i=0; i<pso_Num; i++)
//        {
//            for(int j=0; j<PosLst.size(); j++)
//            {
//                pos = PosLst[j];
//                if(j>splitNum)
//                    jj = splitNum;
//                else
//                    jj = j;
//                PosRotate(cposa, pos, pso_x[i][1]+jj*timegap*pso_x[i][4]);
//                PosRotate(cposb, pos, pso_x[i][2]+jj*timegap*pso_x[i][5]);
//                pos.x -= pso_x[i][0]+jj*timegap*pso_x[i][3];
//                PosLst_correct.push_back(pos);
//            }
//            if(pathlst.size()>1)
//            {
//                for(int j=0; j<PosLst2.size(); j++){
//                    pos = PosLst2[j];
//                    PosRotate(cposa, pos, pso_x[i][1]+splitNum*timegap*pso_x[i][4]);
//                    PosRotate(cposb, pos, pso_x[i][2]+splitNum*timegap*pso_x[i][5]);
//                    pos.x -= pso_x[i][0]+splitNum*timegap*pso_x[i][3];
//                    PosLst_correct2.push_back(pos);
//                }
//            }
//            if(pathlst.size()==1)
//            {
//                limit = Pos2Joint(PosLst_correct, AglLst, agl_acl, TInF);
//                if(limit)
//                {
//                    pso_f[i] = 1e10;
//                    pso_f2[i] = 1e10;
//                }
//                else
//                {
//                    pso_f[i] = 0;
//                    pso_f2[i] = 0;
//                    for(int j=0; j<PosLst_correct.size(); j++)
//                        pso_f[i] += 180-abs(PosLst_correct[j].b);
//                    pso_f2[i] += RobotAttitudEvaluate2(AglLst, PosLst_correct, aglBest);
//                    pso_f2[i] = pso_f2[i]/PosLst_correct.size();
//                }
//            }
//            else
//            {
//                limit1 = Pos2Joint(PosLst_correct, AglLst, agl_acl, TInF);
//                limit2 = Pos2Joint(PosLst_correct2, AglLst2, agl_acl, TInF);
//                if(limit1 || limit2)
//                {
//                    pso_f[i] = 1e10;
//                    pso_f2[i] = 1e10;
//                }
//                else
//                {
//                    pso_f[i] = 0;
//                    pso_f2[i] = 0;
//                    for(int j=0; j<PosLst_correct.size(); j++)
//                        pso_f[i] += 180-abs(PosLst_correct[j].b);
//                    pso_f2[i] += RobotAttitudEvaluate2(AglLst, PosLst_correct, aglBest);
//                    pso_f2[i] += RobotAttitudEvaluate2(AglLst2, PosLst_correct2, aglBest);
//                    pso_f2[i] = pso_f2[i]/(PosLst_correct.size()+PosLst_correct2.size());
//                }
//            }
//            WritePath(PosLst_correct, "path");
//            AglLst_s.push_back(AglLst);
//            PosLst_correct.clear();
//            PosLst_correct2.clear();
//            AglLst.clear();
//            AglLst2.clear();
//        }

//        for(int i=0; i<pso_Num; i++)
//            if(pso_f[i] < pso_gbestf)
//                pso_gbestf = pso_f[i];
//        for(int i=0; i<pso_Num; i++)
//            if(pso_f[i] == pso_gbestf)
//                indexLst.push_back(i);
//        for(int i=0; i<indexLst.size(); i++)
//            if(pso_f2[indexLst[i]] < pso_gbestf2)
//            {
//                pso_gbestf2 = pso_f2[indexLst[i]];
//                index = indexLst[i];
//            }
//    }
//    if(index != 1e10)
//    {
//        WriteAgl(AglLst_s[0], "aglLst");
//        StateWithSpeedLst.push_back(pso_x[index]);
//    }
////    StateWithSpeedLst.push_back(pso_x[20]);
//}


// 船型焊
void MainWindow2::ShipForm_Weld(vector<vector<RobotPos>> pathlst, vector<vector<double>> region, vector<vector<double>> &StateWithSpeedLst, double &weldTime, vector<int> Variables, double WeldSpeed, int pso_Num, int time, int splitNumS, int splitNumE)
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
    double index = 1e10;
    vector<int> indexLst;
    vector<vector<RobotAxle>> AglLst_s;
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
        vector<RobotPos> PosLst_correct, PosLst_correct_, PosLst_correct2;
        vector<RobotAxle> AglLst, AglLst_, AglLst2;
        bool limit, limit1, limit2;
        int splitNum;
        double timegap = sqrt(pow(PosLst[0].x-PosLst[1].x,2)
                     +pow(PosLst[0].y-PosLst[1].y,2)
                     +pow(PosLst[0].z-PosLst[1].z,2))/WeldSpeed;
        splitNum = splitNumE-splitNumS;
        weldTime = timegap*(splitNumE-splitNumS);
        int jj = 0;
        pso_Num = pso_x.size();
        for(int i=0; i<pso_Num; i++)
        {
            for(int j=0; j<PosLst.size(); j++)
            {
                pos = PosLst[j];
                if(j<splitNumS)
                    jj = 0;
                else if(j>splitNumE)
                    jj = splitNumE-splitNumS;
                else
                    jj = j-splitNumS;
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
                RetracementProcess(PosLst_correct, backDistance, backPosNum);
                for(int k=0; k<10; k++)
                    PosLst_correct_.push_back(PosLst_correct[k]);
                limit1 = Pos2Joint(PosLst_correct, AglLst, agl_acl, TInF);
                limit2 = Pos2Joint(PosLst_correct_, AglLst_, agl_acl, CInF);
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
            WritePath(PosLst_correct, "path");
            AglLst_s.push_back(AglLst);
            PosLst_correct.clear();
            PosLst_correct_.clear();
            PosLst_correct2.clear();
            AglLst.clear();
            AglLst_.clear();
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
    if(index != 1e10)
    {
        WriteAgl(AglLst_s[0], "aglLst");
        StateWithSpeedLst.push_back(pso_x[index]);
    }
//    StateWithSpeedLst.push_back(pso_x[8]);
}


void MainWindow2::PostureAdjustment(vector<RobotPos> &path)
{
    bool piPei, wetherEnd;
    vector<int> piPeiLst;
    RobotPos pos, posi;
    vector<RobotPos> path2;
    Matrix4d Mt, Mti, Mc, MCInT, MTInC;
    wetherEnd = false;
    double rotate180 = 0;
    int index;
    for(int i=0; i<path.size(); i+=interval)
    {
        piPei = false;
        for(double jj=0; jj<900; jj++)
        {
            if(piPei)
                break;
            for(double j=-jj; j<=jj+1; j+=(2*jj+1))
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
        }
        if(wetherEnd)
            break;
        if(!piPei)
        {
            path2.push_back(path[i]);
            piPeiLst.push_back(i/interval);
        }
    }
    for(int i=index; i<path.size(); i+=interval)
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
void MainWindow2::PostureAdjustment2(vector<RobotPos> &path)
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
        for(double j=-200; j<200; j++)
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


//// 将轨迹转化为间隔为20mm的初步龙骨
//void MainWindow2::KeeldataReorganize(vector<RobotPos> &path)
//{
//    RobotPos pos, pos1, pos2;
//    vector<RobotPos> path2;
//    int num = 2000;
//    // 细化为间隔0.01mm的轨迹数据
//    for(int i=0; i<path.size()-1; i++)
//    {
//        pos1 = path[i];
//        pos2 = path[i+1];
//        for(int k=0; k<num; k++){
//            pos.x = pos1.x + k*(pos2.x-pos1.x)/num;
//            pos.y = pos1.y + k*(pos2.y-pos1.y)/num;
//            pos.z = pos1.z + k*(pos2.z-pos1.z)/num;
//            pos.a = pos1.a;
//            pos.b = pos1.b;
//            pos.c = pos1.c;
//            path2.push_back(pos);
//        }
//    }
//    // 摘取间隔为20mm的轨迹点
//    double dis;
//    path.clear();
//    pos = path2[0];
//    path.push_back(pos);
//    for(int i=0; i<path2.size(); i++)
//    {
//        pos1 = path2[i];
//        dis = sqrt(pow(pos1.x-pos.x,2)+pow(pos1.y-pos.y,2)+pow(pos1.z-pos.z,2));
//        if(dis>=19.99)
//        {
//            path.push_back(pos1);
//            pos = pos1;
//        }
//    }
//}


//// 将轨迹转化为间隔为20mm的初步龙骨
//void MainWindow2::KeeldataReorganize(vector<RobotPos> &path)
//{
//    RobotPos pos, pos1, pos2;
//    vector<RobotPos> path2, path3;
//    int num = 2000;
//    // 细化为间隔0.01mm的轨迹数据
//    for(int i=0; i<path.size()-1; i++)
//    {
//        pos1 = path[i];
//        pos2 = path[i+1];
//        for(int k=0; k<num; k++){
//            pos.x = pos1.x + k*(pos2.x-pos1.x)/num;
//            pos.y = pos1.y + k*(pos2.y-pos1.y)/num;
//            pos.z = pos1.z + k*(pos2.z-pos1.z)/num;
//            pos.a = pos1.a;
//            pos.b = pos1.b;
//            pos.c = pos1.c;
//            path2.push_back(pos);
//        }
//    }
//    // 摘取间隔为20mm的轨迹点
//    double dis;
//    path3 = path;
//    path.clear();
//    pos = path2[0];
//    path.push_back(pos);
//    for(int i=0; i<path2.size(); i++)
//    {
//        pos1 = path2[i];
//        dis = sqrt(pow(pos1.x-pos.x,2)+pow(pos1.y-pos.y,2)+pow(pos1.z-pos.z,2));
//        if(dis>=19.99)
//        {
//            if(path.size()<path3.size())
//            {
//                pos1.a = path3[path.size()].a;
//                pos1.b = path3[path.size()].b;
//                pos1.c = path3[path.size()].c;
//            }
//            else
//            {
//                pos1.a = path3[path3.size()-1].a;
//                pos1.b = path3[path3.size()-1].b;
//                pos1.c = path3[path3.size()-1].c;
//            }
//            path.push_back(pos1);
//            pos = pos1;
//        }
//    }
//}


// 将轨迹转化为间隔为5mm的初步龙骨
void MainWindow2::KeeldataReorganize(vector<RobotPos> &path)
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

    // 摘取间隔为5mm的轨迹点
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
void MainWindow2::CreateKeel(vector<RobotPos> &path)
{
    vector<RobotPos> path2, path3;
    RobotPos pos, pos1, pos2;

    // 焊缝点细化（5mm间隔转化为1mm间隔）
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
void MainWindow2::CreateKeel2(vector<RobotPos> &path)
{
    vector<RobotPos> path2, path3;
    RobotPos pos, pos1, pos2;

    // 焊缝点细化（5mm间隔转化为1mm间隔）
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
void MainWindow2::CreateKeelScan(vector<RobotPos> &path)
{
    vector<RobotPos> path2, path3;
    RobotPos pos, pos1, pos2;

    // 焊缝点细化（20mm间隔转化为5mm间隔）
    for(int j=0; j<path.size()-2; j++){
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


void MainWindow2::pathReverse(vector<RobotPos> &path)
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


void MainWindow2::laserReverse(vector<RobotPos> &path)
{
    for(auto &r : path)
        PosRotate(r, r, 180);
}


// 由CAD数据获取工件坐标系下的轨迹
void MainWindow2::posInTool(vector<RobotPos> &path, RobotPos posW)
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
void MainWindow2::posInBase(vector<RobotPos> &path,
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
double MainWindow2::RobotAttitudEvaluate2(vector<RobotAxle> aglLst, vector<RobotPos> path, RobotAxle aglBest)
{
    double fun = 0;
    RobotAxle agl;
    RobotPos pos;
    double k = 0.1;
    double dis = 1200;
    int posNum = backPosNum;
    Vector3d v1, v2;
    Matrix4d M;
    double a1, v;
    for(int i=posNum; i<aglLst.size()-posNum; i++)
    {
        agl = aglLst[i];
        pos = path[i];
        // 评价各个轴与焊枪位置
        fun += pow(agl.a1-aglBest.a1,2);
        fun += pow(agl.a3-aglBest.a3,2);
        fun += pow(agl.a5-aglBest.a5,2);
        fun += k*pow(abs(sqrt(pow(pos.x,2)+pow(pos.y,2))-dis),2);
        // 评价激光面与焊枪的前后位置
        a1 = agl.a1/(180/M_PI);
        pos2Matrix(pos, M);
        v1 = {sin(a1), cos(a1), 0};
        v2 = {M(0,0), M(1,0), 0};
        v = v1.dot(v2)/sqrt(v1.dot(v1)*v2.dot(v2));
        if(abs(M(2,0))<0.5) //判断是否为竖缝
        {
            fun -= 10000*v*abs(v);
        }
    }
    return fun;
}


// 路径末端姿态修改(绕y轴旋转)
void MainWindow2::pathEndRotate(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection)
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
    int startIdx = path.size()-backPosNum-posNum-1;
    // 修改焊缝尾端姿态
    for(int i=startIdx+1; i<path.size()-backPosNum; i++)
    {
        cpos = path[i];
        pos2Matrix(cpos, mc);
        mc = mc*rx;
        Matrix2pos(mc, cpos);
        PosRotate(cpos, path[i], (i-startIdx)*agl);
    }
    // 修改回撤部分姿态
    for(int i=path.size()-backPosNum; i<path.size(); i++)
    {
        cpos = path[i];
        pos2Matrix(cpos, mc);
        mc = mc*rx;
        Matrix2pos(mc, cpos);
        PosRotate(cpos, path[i], posNum*agl);
    }
}


// 路径首端姿态修改(绕y轴旋转)
void MainWindow2::pathHeadRotate(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection)
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
    int startIdx = backPosNum;
    if(posNum>path.size())
        posNum = path.size();
    // 修改焊缝首端姿态
    for(int i=startIdx; i<backPosNum+posNum; i++)
    {
        cpos = path[i];
        pos2Matrix(cpos, mc);
        mc = mc*rx;
        Matrix2pos(mc, cpos);
        PosRotate(cpos, path[i], (backPosNum+posNum-i)*agl);
    }
    // 修改回撤部分姿态
    for(int i=0; i<backPosNum; i++)
    {
        cpos = path[i];
        pos2Matrix(cpos, mc);
        mc = mc*rx;
        Matrix2pos(mc, cpos);
        PosRotate(cpos, path[i], posNum*agl);
    }
}


// 绕x轴旋转
double MainWindow2::posRotateX(vector<RobotPos> &path, double rotataAgle)
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


// 绕y轴旋转
double MainWindow2::posRotateY(vector<RobotPos> &path, double rotataAgle)
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


// 绕Z轴旋转
double MainWindow2::posRotateZ(vector<RobotPos> &path, double rotataAgle)
{
    RobotPos cpos;
    for(int i=0; i<path.size(); i++)
    {
        cpos = path[i];
        PosRotate(cpos, path[i], rotataAgle);
    }
}


// 关联处理(手动设置)
void MainWindow2::RelateProcess(vector<vector<RobotPos>> PathLst, vector<vector<int>> RelateIndex, vector<vector<vector<RobotPos>>> &RelatePathLst_s)
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


// 得到零点状态下的工具坐标系
void MainWindow2::GetToolCoordinate(RobotPos &TPos, double orbitX, double rotateAgl, double flipAgl)
{
    TPos.x += orbitX;
    PosRotate(cposb, TPos, -flipAgl);
    PosRotate(cposa, TPos, -rotateAgl);
}


// 将基坐标系下的轨迹转化为CAD中的轨迹
void MainWindow2::base2Cad(vector<RobotPos> &path,
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


void MainWindow2::Smooth(vector<RobotPos> &path, vector<RobotPos> &pathRf)
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


void MainWindow2::SeekProcess(vector<vector<int>> RelateIndex)
{
    vector<vector<RobotPos>> pathLst;
    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;
    vector<vector<int>> WeldClassifyPlan;
    Read_pathlst(pathLst, filename);

    // 绕X轴旋转
    for(int i=0; i<pathLst.size(); i++)
        posRotateX(pathLst[i], aglA);

    // 焊缝关联信息，关联分类
    RelateProcess(pathLst, RelateIndex, pathLst_s);

    // 读配置文件
    vector<vector<string>> nameLsts;
    vector<vector<double>> region;
    vector<int> direction;
    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead, directionLst, orderIDLsts, rankLsts, TracingLsts;
    vector<vector<vector<int>>> directionLst_s;
//    readConfig2(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead,
//                region, nameLsts, orderIDLsts, rankLsts, TracingLsts);
    readConfig3(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead,
                nameLsts, orderIDLsts, rankLsts, TracingLsts);
    readConfig_rgn(configPath, "region", region);
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

    // 添加回撤路径
    readConfig_Dou(configPath, "parameterAgl", "backDistance", backDistance);
    readConfig_Int(configPath, "parameterAgl", "backPosNum", backPosNum);
    if(backPosNum>0)
    {
        for(int i=0; i<pathLst_s.size(); i++)
            for(int j=0; j<pathLst_s[i].size(); j++)
                RetracementProcess(pathLst_s[i][j], backDistance, backPosNum);
    }

    // 转换为零点下状态时基标系下的位姿
    for(auto &r0 : pathLst_s)
        for(auto &r1 : r0)
            posInBase(r1, posW1, posW2);

   // 非联动处理
   WeldClassify9(pathLst_s, region, WeldClassifyPlan, directionLst_s, pathDirection, laserDirection, rotateYHead, rotateYEnd);

   // 根据焊缝附带信息调节焊缝姿态
   backPosNum = 0;
   backDistance = 0;
   for(int i=0; i<pathLst_s2.size(); i++)
   {
       for(int j=0; j<pathLst_s2[i].size(); j++)
       {
           Weld2Path(pathLst_s2[i][j], directionLst_s[i][j]);
       }
   }
   pathLst_s3 = pathLst_s2;


   // 保存轴数据
   for(int i=0; i<pathLst_s.size(); i++)
   {
       for(int j=0; j<pathLst_s[i].size(); j++)
       {
           Weld2Path(pathLst_s[i][j], directionLst_s[i][j]);
       }
   }

   vector<RobotAxle> AglLst;
   bool limit;
   QFile file("AglLst2.txt");
   if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
       return;
   QTextStream out(&file);
   for(int i=0; i<WeldClassifyPlan.size(); i++)
   {
       for(int j=5; j<WeldClassifyPlan[i].size(); j++)
       {
           if(WeldClassifyPlan[i][j]==1)
           {
               for(int k=0; k<pathLst_s[j-5].size(); k++)
                   CreateKeelScan(pathLst_s[j-5][k]);  //扫描龙骨数据
               for(int k=0; k<pathLst_s[j-5].size(); k++)
               {
                   for(auto &r : pathLst_s[j-5][k]){
                       PosRotate(cposa, r, region[3][1]+WeldClassifyPlan[i][3]*da);
                       PosRotate(cposb, r, region[4][1]+WeldClassifyPlan[i][4]*db);
                       r.x -= (region[0][1]+WeldClassifyPlan[i][0]*dx);
                   }

                   limit = Pos2Joint2(pathLst_s[j-5][k], AglLst, agl_acl, CInF);
                   out << "seamId:" <<  "\n";
                   out << "robotPosition:" << -(region[0][1]+WeldClassifyPlan[i][0]*dx) <<  "\n";
                   out << "agl1:" << region[4][1]+WeldClassifyPlan[i][4]*db <<  "\n";
                   out << "agl2:" << region[3][1]+WeldClassifyPlan[i][3]*da <<  "\n";
                   for(int ii=0; ii<(int)AglLst.size(); ii+=1)
                       out << AglLst[ii].a1 << "," << AglLst[ii].a2 << "," << AglLst[ii].a3 << ","
                           << AglLst[ii].a4 << "," << AglLst[ii].a5 << "," << AglLst[ii].a6 <<  "\n";
                   AglLst.clear();
               }
           }
       }
   }


   RepeatData Data;
   Write2Database_Seek(Data, WeldClassifyPlan, dx, da, region, nameLsts, orderIDLsts, rankLsts, TracingLsts, pathLst_s2, pathLst_s3);
   std::string ret = InputControl::CreatSerial(Data);





//   // 焊缝整理
//   RepeatData *Data = new RepeatData;
//   Zitai_Info Zitai;
//   RepeatSeamArray seam_s;
//   Seaminfo seam;
//   RepeatPos posLst1, posLst2;
//   Zitai.spacing1 = 0;
//   Zitai.spacing2 = 0;
//   Zitai.distance1 = 0;
//   Zitai.distance2 = 0;
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


// 生成多条寻位轨迹（不经限位计算）
void MainWindow2::SeekProcessByMultiple(vector<vector<int>> RelateIndex)
{
    vector<vector<RobotPos>> pathLst;
    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;
    Read_pathlst(pathLst, filename);

    // 读配置文件
    vector<vector<string>> nameLsts;
    vector<vector<double>> region;
    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead;
    readConfig(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead, region, nameLsts);

    // 绕X轴旋转
    for(int i=0; i<pathLst.size(); i++)
        posRotateX(pathLst[i], aglA);

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
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,3,0);
                }
                else{
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,3,1);
                }
            }

            if(rotateYHead[i][j]==1){
                if(laserDirection[i][j]==1){
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,3,1);
                }
                else{
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,3,0);
                }
            }
        }
    }

    // 焊缝关联信息，关联分类
    RelateProcess(pathLst, RelateIndex, pathLst_s);

    // 转换为工件坐标系下的位姿
    pathLst_s2 = pathLst_s;
    for(auto &r0 : pathLst_s2)
        for(auto &r1 : r0)
            posInTool(r1, posW1);


   // 读取配置文件相关信息
   int Multiple;
   double pos_motor, angle1_motor, angle2_motor, perAgl;
   readConfig_Int(configPath2, std::to_string(RelateIndex[0][0]), "Multiple", Multiple);
   readConfig_Dou(configPath2, std::to_string(RelateIndex[0][0]), "pos_motor", pos_motor);
   readConfig_Dou(configPath2, std::to_string(RelateIndex[0][0]), "angle1_motor", angle1_motor);
   readConfig_Dou(configPath2, std::to_string(RelateIndex[0][0]), "angle2_motor", angle2_motor);
   perAgl = -360/Multiple;

   // 旋转复制生成多条轨迹
   vector<RobotPos> path;
   RobotPos cpos = {0,0,0,0,0,0};
   path = pathLst_s2[0][0];
   for(int i=1; i<Multiple; i++)
   {
       for(auto &r : path)
           PosRotate(cpos, r, perAgl);
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
//   Zitai.spacing1 = 0;
//   Zitai.spacing2 = 0;
//   Zitai.distance1 = 0;
//   Zitai.distance2 = 0;
//   for(int i=0; i<Multiple; i++)
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
//       Zitai.pos_motor_start = pos_motor;
//       Zitai.pos_motor_stop = pos_motor;
//       Zitai.angle1_motor_start = angle1_motor;
//       Zitai.angle1_motor_stop = angle1_motor;
//       Zitai.angle2_motor_start = angle2_motor+perAgl*i;
//       Zitai.angle2_motor_stop = angle2_motor+perAgl*i;
//       Zitai.motor1speed = 0;
//       Zitai.motor2speed = 0;
//       Zitai.motor3speed = 0;
//       Data->push_back(Zitai);
//   }
//   std::string ret = InputControl::CreatSerial(*Data);
}


// 生成多条寻位轨迹（姿态经由示教数据确定）
void MainWindow2::SeekProcessByTeach(vector<vector<int>> RelateIndex)
{
    vector<RobotPos> path, path2;
    vector<vector<RobotPos>> pathLst;
    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s3;

    // 焊缝关联信息，关联分类(示教轨迹)
    Read_pathlst(pathLst, filename2);
    RelateProcess(pathLst, RelateIndex, pathLst_s);
    path = pathLst_s[0][0];
    pathLst.clear();
    pathLst_s.clear();
    for(int i=0; i<path.size(); i++)
        GetToolCoordinate(path[i], -900, 15, -90);
    base2Cad(path, posW1, posW2);

    // 焊缝关联信息，关联分类(CAD轨迹)
    Read_pathlst(pathLst, filename);
    RelateProcess(pathLst, RelateIndex, pathLst_s);
    path2 = pathLst_s[0][0];

    // 以示教轨迹调整CAD轨迹的姿态
    Smooth(path2, path);

    // 转换为工件坐标系下的位姿
    posInTool(path, posW1);
    posInTool(path2, posW1);

    pathLst_s[0][0] = path2;
    pathLst_s2.push_back(pathLst_s[0]);
    pathLst_s[0][0] = path;
    pathLst_s3.push_back(pathLst_s[0]);

    // 读取配置文件相关信息
    int Multiple = 12;
    double pos_motor, angle1_motor, angle2_motor, perAgl;
    readConfig_Int(configPath2, std::to_string(RelateIndex[0][0]), "Multiple", Multiple);
    readConfig_Dou(configPath2, std::to_string(RelateIndex[0][0]), "pos_motor", pos_motor);
    readConfig_Dou(configPath2, std::to_string(RelateIndex[0][0]), "angle1_motor", angle1_motor);
    readConfig_Dou(configPath2, std::to_string(RelateIndex[0][0]), "angle2_motor", angle2_motor);
    perAgl = -360/Multiple;

    // 旋转复制生成多条轨迹
    RobotPos cpos = {0,0,0,0,0,0};
    for(int i=1; i<Multiple; i++)
    {
        for(auto &r : path)
            PosRotate(cpos, r, perAgl);
        for(auto &r : path2)
            PosRotate(cpos, r, perAgl);
        pathLst_s[0][0] = path2;
        pathLst_s2.push_back(pathLst_s[0]);
        pathLst_s[0][0] = path;
        pathLst_s3.push_back(pathLst_s[0]);
    }

    // 焊缝整理
    AScanePos sPos;
    AFramePos fPos;
    QList<AScanePos> scanePos;
    QList<AFramePos> framePos;
    SeamInfo seam;
    QList<SeamInfo> seams;
    MachineGrond  machine;
    double db = 0;
    for(int i=0; i<Multiple; i++)
    {
        for(int k=0; k<pathLst_s2[i].size(); k++)
        {
           for(int ki=0; ki<pathLst_s2[i][k].size(); ki++)
           {
               RobotPos p = pathLst_s2[i][k][ki];
               sPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
               sPos.index1 = ki;
               sPos.enabled = 1;
               scanePos.push_back(sPos);
           }
           for(int ki=0; ki<pathLst_s3[i][k].size(); ki++)
           {
               RobotPos p = pathLst_s3[i][k][ki];
               fPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
               fPos.index1 = ki;
               framePos.push_back(fPos);
           }
           seam.scanePos = scanePos;
           seam.framePos = framePos;
           seam.seamName = "seam"+std::to_string(i+1)+"_1_0";
           seam.orderIndex = 0;
           seam.rank = 1;
           seam.tracing = 0;
           machine.pos_motor_start = pos_motor;
           machine.pos_motor_stop = pos_motor;
           machine.angle1_motor_start = angle1_motor;
           machine.angle1_motor_stop = angle1_motor;
           machine.angle2_motor_start = angle2_motor+perAgl*i;
           machine.angle2_motor_stop = angle2_motor+perAgl*i;
           seam.machine = machine;
           Data.seams.push_back(seam);
           scanePos.clear();
           framePos.clear();
        }
    }
}


void MainWindow2::TrackProcess(vector<vector<int>> RelateIndex)
{
    vector<vector<RobotPos>> pathLst;
    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s4, pathLst_s0;
    Read_pathlst(pathLst, filename);
    backDistance = 0;
    backPosNum = 0;

    // 绕X轴旋转
    for(int i=0; i<pathLst.size(); i++)
        posRotateX(pathLst[i], aglA);

    // 焊缝关联信息，关联分类
    RelateProcess(pathLst, RelateIndex, pathLst_s);
    pathLst_s0 = pathLst_s;

    // 圆拟合
//    circleFit circlefit;
//    circlefit.pointLst = pathLst_s[0][0];
//    circlefit.build();


    // 读配置文件
    vector<vector<string>> nameLsts;
    vector<vector<double>> region, region2;
    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead, orderIDLsts, rankLsts, TracingLsts;
    readConfig2(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead,
                region, nameLsts, orderIDLsts, rankLsts, TracingLsts);
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
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,3,0);
                }
                else{
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,3,1);
                }
            }

            if(rotateYHead[i][j]==1){
                if(laserDirection[i][j]==1){
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,3,1);
                }
                else{
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,3,0);
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
    int splitNumS, splitNumE, rank, rttDir;
    double offsetAgl;
    readConfig_Dou(configPath, "parameterAgl", "offsetAgl", offsetAgl);
    readConfig_Int(configPath, std::to_string(RelateIndex[0][0]), "splitNumS", splitNumS);
    readConfig_Int(configPath, std::to_string(RelateIndex[0][0]), "splitNumE", splitNumE);
    readConfig_Int(configPath, std::to_string(RelateIndex[0][0]), "rank", rank);
    readConfig_Int(configPath, std::to_string(RelateIndex[0][0]), "rttDir", rttDir);
    readConfig_Dou(configPath, "parameterAgl", "backDistance", backDistance);
    readConfig_Int(configPath, "parameterAgl", "backPosNum", backPosNum);
    double aglE, aglS, aglG, wBest;
    {
        // 找到变位机最佳起始相位角与终止相位角
        region2 = region;
        findStartAngle(pathLst_s0[0][0][splitNumE], region2[3], region2[4]);
        aglE = region2[3][1];
        findStartAngle(pathLst_s0[0][0][splitNumS], region[3], region[4]);
        aglS = region[3][1];
        aglG = aglE-aglS;
        if(rttDir)
        {
            if(region[4][1]<0)
                offsetAgl = -offsetAgl;
        }
        else{
            if(region[4][1]>0)
                offsetAgl = -offsetAgl;
        }
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
    RobotPos pos1, pos2, posi;
    {
       // 记录原始pos分界点
       pos1 = pathLst_s[0][0][splitNumS];
       pos2 = pathLst_s[0][0][splitNumE];
    }
    for(int i=0; i<pathLst_s.size(); i++)
    {
//       fit(pathLst_s[i][0]);
       KeeldataReorganize(pathLst_s[i][0]);
//       CreateKeel(pathLst_s[i][0]);
       {
           // 找到龙骨pos分界点
           double dis;
           for(int i=0; i<pathLst_s[0][0].size(); i++)
           {
               posi = pathLst_s[0][0][i];
               dis = sqrt(pow(posi.x-pos1.x,2)+pow(posi.y-pos1.y,2)+pow(posi.z-pos1.z,2));
               if(dis<5)
               {
                   splitNumS = i;
                   break;
               }
           }
           for(int i=0; i<pathLst_s[0][0].size(); i++)
           {
               posi = pathLst_s[0][0][i];
               dis = sqrt(pow(posi.x-pos2.x,2)+pow(posi.y-pos2.y,2)+pow(posi.z-pos2.z,2));
               if(dis<5)
               {
                   splitNumE = i;
                   break;
               }
           }
       }

       if(rank!=3){
           splitNumE = pathLst_s[0][0].size()-1;
       }
       {
           // 找到变位机最佳旋转速度
           wBest = aglG/((splitNumE-splitNumS)*interval/weldSpeed);
           region[8][1] = wBest;
           region[8][2] = wBest;
           double Dagl = aglG/(splitNumE-splitNumS);
       }
       if(rank==0){
           aglG = 0;
           region[3][1] -= offsetAgl;
           region[3][2] -= offsetAgl;
           region[8][1] = 0;
           region[8][2] = 0;
       }
       ShipForm_Weld(pathLst_s[i], region, StateWithSpeedLst, weldTime, Variables, weldSpeed, pso_Num, time, splitNumS, splitNumE);
//       StateWithSpeedLst.push_back(StateWithSpeed);
       weldTimeLst.push_back(weldTime);
       StateWithSpeed.clear();
       index.push_back(i);
    }


    ACad cad;
    RepeatData Data;
    cad.cadid = CadName;
    Data.cad<<cad;
    Write2Database_Track(Data, StateWithSpeedLst, index, nameLsts,
                         orderIDLsts, rankLsts, TracingLsts, weldTimeLst, pathLst_s2, pathLst_s4, aglG, splitNumS, splitNumE);
    std::string ret = InputControl::CreatSerial(Data);

}


// 生成跟踪轨迹（不经限位计算）
void MainWindow2::TrackProcessDirect(vector<vector<int>> RelateIndex)
{
    vector<vector<RobotPos>> pathLst;
    vector<vector<vector<RobotPos>>>  pathLst_s, pathLst_s2, pathLst_s4;
    Read_pathlst(pathLst, filename);

    // 焊缝关联信息，关联分类
    RelateProcess(pathLst, RelateIndex, pathLst_s);

    // 读配置文件
    vector<vector<string>> nameLsts;
    vector<vector<double>> region, region2;
    vector<vector<int>> pathDirection, laserDirection, rotateYEnd, rotateYHead;
    readConfig(configPath, RelateIndex, pathDirection, laserDirection, rotateYEnd, rotateYHead, region, nameLsts);

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
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,3,0);
                }
                else{
                    pathEndRotate(pathLst_s[i][j],aglEnd-aglWeld,3,1);
                }
            }

            if(rotateYHead[i][j]==1){
                if(laserDirection[i][j]==1){
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,3,1);
                }
                else{
                    pathHeadRotate(pathLst_s[i][j],aglWeld-aglHead,3,0);
                }
            }
        }
    }

    // 转换为工件坐标系下的位姿
    pathLst_s2 = pathLst_s;
    for(auto &r0 : pathLst_s2)
        for(auto &r1 : r0)
            posInTool(r1, posW1);
    pathLst_s4 = pathLst_s2;

//    // 焊缝处理
//    RepeatData *Data = new RepeatData;
//    Zitai_Info Zitai;
//    RepeatSeamArray seam_s;
//    Seaminfo seam;
//    RepeatPos posLst1, posLst2;
//    Zitai.spacing1 = 0;
//    Zitai.spacing2 = 0;
//    Zitai.distance1 = 0;
//    Zitai.distance2 = 0;
//    int orderIndex = 0;
//    for(int i=0; i<1; i++)
//    {
//       // 跟踪焊缝
//       KeeldataReorganize(pathLst_s2[0][0]);
//       CreateKeel2(pathLst_s2[0][0]);  //扫描数据
//       KeeldataReorganize(pathLst_s4[0][0]);
//       CreateKeel(pathLst_s4[0][0]);  //龙骨数据
//       for(int k=0; k<1; k++)
//       {
//           orderIndex++;
//           for(int ki=0; ki<pathLst_s2[0][k].size(); ki++){
//               RobotPos p = pathLst_s2[0][k][ki];
//               posLst1.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           for(int ki=0; ki<pathLst_s4[0][k].size(); ki++){
//               RobotPos p = pathLst_s4[0][k][ki];
//               posLst2.push_back(Pos(p.x, p.y, p.z, p.a, p.b, p.c));
//           }
//           seam.scanePos = posLst1;
//           seam.framePos = posLst2;
//           seam.seamName = "seam1_1_0";
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
//       Zitai.motor2speed = 0;
//       Zitai.motor3speed = 0;
//       Zitai.pos_motor_start = 0;
//       Zitai.pos_motor_stop = 0;
//       Zitai.angle1_motor_start = 0;
//       Zitai.angle1_motor_stop = 0;
//       Zitai.angle2_motor_start = 0;
//       Zitai.angle2_motor_stop = 0;
//       Data->push_back(Zitai);
//    }
//    std::string ret = InputControl::CreatSerial(*Data);
}


void MainWindow2::on_pushButton_GetWorkIfo_clicked()
{
    RelateIndexSeek.clear();
    RelateIndexTrack.clear();
    CadName = (ui->edtCad->text()).toStdString();
    configPath = ("configuration/" + ui->edtCad->text() + ".info").toStdString();
    filename = "pathData/" + ui->edtCad->text() + ".txt";
    configPath2 = ("configuration/" + ui->edtCad->text() + "2.info").toStdString();
    filename2 = "pathData/" + ui->edtCad->text() + "2.txt";
    aglBest = {-45,0,-90,0,-60,0};
    agl_acl = {-15, -36, -132, 44, -62, -66};
    readConfig_Dou(configPath, "parameterAgl", "backDistance", backDistance);
    readConfig_Int(configPath, "parameterAgl", "backPosNum", backPosNum);
    readConfig_RobotPos(configPath, "cposa", cposa);
    readConfig_RobotPos(configPath, "cposb", cposb);
    readConfig_RobotPos(configPath, "TInF", TInF);
    readConfig_RobotPos(configPath, "CInF", CInF);
    readConfig_RobotPos(configPath, "CInT", CInT);
    readConfig_RobotPos(configPath, "posW1", posW1);
    readConfig_RobotPos(configPath, "posW2", posW2);
    readConfig_agl(configPath, "parameterAgl", aglWeld, aglInclude, aglHead, aglEnd, aglA, aglT);
    readConfig_Seam(configPath, RelateIndexSeek, RelateIndexTrack);
}


void MainWindow2::on_cbxMode_activated(const QString &arg1)
{
    QString str = ui->cbxMode->currentText();
    vector<string> state, relation, seam;
//    state = {"state1","state2","state3","state4","state5","state6","state7","state8","state9","state10"};
//    relation = {"relation1","relation2","relation3","relation4","relation5",
//                "relation6","relation7","relation8","relation9","relation10"};
    for(int i=0; i<100; i++)
    {
        state.push_back("state"+std::to_string(i+1));
    }
    for(int i=0; i<100; i++)
    {
        relation.push_back("relation"+std::to_string(i+1));
    }
    for(int i=0; i<100; i++)
    {
        seam.push_back("seam"+std::to_string(i+1));
    }
//    seam = {"seam1","seam2","seam3","seam4","seam5","seam6","seam7","seam8","seam9","seam10"};

    ui->cbxState->clear();

    if(str=="寻位")
    {
        for(int i=0; i<RelateIndexSeek.size(); i++)
        {
            ui->cbxState->addItem(QString::fromStdString("state"+std::to_string(i+1)));
        }
    }
    else
    {
        for(int i=0; i<RelateIndexTrack.size(); i++)
        {
            ui->cbxState->addItem(QString::fromStdString("state"+std::to_string(i+1)));
        }
    }
}


void MainWindow2::on_cbxState_activated(const QString &arg1)
{
    QString strMode = ui->cbxMode->currentText();
    QString strState = ui->cbxState->currentText();

    ui->cbxRelate->clear();

    if(strMode=="寻位")
    {
        for(int i=0; i<1e10; i++)
        {
            if(strState==QString::fromStdString("state"+std::to_string(i+1)))
            {
                for(int j=0; j<RelateIndexSeek[i].size(); j++)
                {
                    ui->cbxRelate->addItem(QString::fromStdString("relation"+std::to_string(j+1)));
                }
                break;
            }
        }
    }

    else
    {
        for(int i=0; i<1e10; i++)
        {
            if(strState==QString::fromStdString("state"+std::to_string(i+1)))
            {
                for(int j=0; j<RelateIndexTrack[i].size(); j++)
                {
                    ui->cbxRelate->addItem(QString::fromStdString("relation"+std::to_string(j+1)));
                }
                break;
            }
        }
    }
}


void MainWindow2::on_cbxRelate_activated(const QString &arg1)
{
    QString strMode = ui->cbxMode->currentText();
    QString strState = ui->cbxState->currentText();
    QString strRelate = ui->cbxRelate->currentText();
    QStringList strSeams;
    QAbstractItemModel * model;

    if(strMode=="寻位")
    {
        for(int i=0; i<1e10; i++)
        {
            if(strSeams.size()>0)
                break;
            if(strState==QString::fromStdString("state"+std::to_string(i+1)))
            {
                for(int j=0; j<1e10; j++)
                {
                    if(strRelate==QString::fromStdString("relation"+std::to_string(j+1)))
                    {
                        for(int k=0; k<RelateIndexSeek[i][j].size(); k++)
                        {
                            strSeams.push_back(QString::fromStdString("seam"+std::to_string(RelateIndexSeek[i][j][k])));
                        }
                        model=new QStringListModel(strSeams);
                        ui->listView->setModel(model);
                        break;
                    }
                }
            }
        }
    }

    else
    {
        for(int i=0; i<1e10; i++)
        {
            if(strSeams.size()>0)
                break;
            if(strState==QString::fromStdString("state"+std::to_string(i+1)))
            {
                for(int j=0; j<1e10; j++)
                {
                    if(strRelate==QString::fromStdString("relation"+std::to_string(j+1)))
                    {
                        for(int k=0; k<RelateIndexTrack[i][j].size(); k++)
                        {
                            strSeams.push_back(QString::fromStdString("seam"+std::to_string(RelateIndexTrack[i][j][k])));
                        }
                        model=new QStringListModel(strSeams);
                        ui->listView->setModel(model);
                        break;
                    }
                }
            }
        }
    }
}




void MainWindow2::on_pushButton_goState_clicked()
{
    QString strMode = ui->cbxMode->currentText();
    QString strState = ui->cbxState->currentText();
    vector<vector<int>> RelateIndex;

    if(strMode=="寻位")
    {
        for(int i=0; i<1e10; i++)
        {
            if(strState==QString::fromStdString("state"+std::to_string(i+1)))
            {
                RelateIndex = RelateIndexSeek[i];
                break;
            }
        }

        SeekProcess(RelateIndex);
    }

    else
    {
        for(int i=0; i<1e10; i++)
        {
            if(strState==QString::fromStdString("state"+std::to_string(i+1)))
            {
                RelateIndex = RelateIndexTrack[i];
                break;
            }
        }

        TrackProcess(RelateIndex);
    }
}



void MainWindow2::on_pushButton_goRelation_clicked()
{
    QString strMode = ui->cbxMode->currentText();
    QString strState = ui->cbxState->currentText();
    QString strRelate = ui->cbxRelate->currentText();
    vector<vector<int>> RelateIndex;

    if(strMode=="寻位")
    {
        for(int i=0; i<1e10; i++)
        {
            if(RelateIndex.size()>0)
                break;
            if(strState==QString::fromStdString("state"+std::to_string(i+1)))
            {
                for(int j=0; j<1e10; j++)
                {
                    if(strRelate==QString::fromStdString("relation"+std::to_string(j+1)))
                    {
                        RelateIndex.push_back(RelateIndexSeek[i][j]);
                        break;
                    }
                }
            }
        }

        SeekProcess(RelateIndex);
    }

    else
    {
        for(int i=0; i<1e10; i++)
        {
            if(RelateIndex.size()>0)
                break;
            if(strState==QString::fromStdString("state"+std::to_string(i+1)))
            {
                for(int j=0; j<1e10; j++)
                {
                    if(strRelate==QString::fromStdString("relation"+std::to_string(j+1)))
                    {
                        RelateIndex.push_back(RelateIndexTrack[i][j]);
                        break;
                    }
                }
            }
        }

        TrackProcess(RelateIndex);
    }
}


void MainWindow2::on_listView_doubleClicked(const QModelIndex &index)
{
    dialog = new Dialog(this);
    dialog->strSelect = (ui->listView->model()->data(index).toString()).toStdString();
    QString str;
    str = QString::number(aglWeld);
    dialog->ui->edtAglO->setText(str);
    str = QString::number(aglA);
    dialog->ui->edtAglA->setText(str);
    str = QString::number(aglT);
    dialog->ui->edtAglT->setText(str);
    str = QString::number(aglHead);
    dialog->ui->edtAglS->setText(str);
    str = QString::number(aglEnd);
    dialog->ui->edtAglE->setText(str);
    dialog->setModal(false);
    dialog->show();
}


void MainWindow2::on_pushButton_goto_clicked()
{
    vector<int> index;
    vector<vector<int>> RelateIndex;
    QString strMode = ui->cbxMode->currentText();
    aglWeld = dialog->aglWeld;
    aglInclude = dialog->aglInclude;
    aglHead = dialog->aglHead;
    aglEnd = dialog->aglEnd;
    aglA = dialog->aglA;
    aglT = dialog->aglT;

    bool checkBox_Multiple = ui->checkBox_Multiple->isChecked();
    bool checkBox_posture = ui->checkBox_posture->isChecked();
    bool checkBox_track = ui->checkBox_track->isChecked();

    for(int i=0; i<1e10; i++)
    {
        if(dialog->strSelect=="seam"+std::to_string(i))
        {
            index.push_back(i);
            break;
        }
    }
    RelateIndex.push_back(index);

    if(checkBox_Multiple)
        SeekProcessByMultiple(RelateIndex);
    else if(checkBox_posture)
        SeekProcessByTeach(RelateIndex);
    else if(checkBox_track)
        TrackProcessDirect(RelateIndex);
    else
    {
        if(strMode=="寻位")
            SeekProcess(RelateIndex);
        else
            TrackProcess(RelateIndex);
    }

    // 重新恢复相关角度值
    readConfig_agl(configPath, "parameterAgl", aglWeld, aglInclude, aglHead, aglEnd, aglA, aglT);
}


void MainWindow2::on_pushButton_goMode_clicked()
{
    QString strMode = ui->cbxMode->currentText();
    vector<vector<int>> RelateIndex;

    if(strMode=="寻位")
    {
        for(int i=0; i<RelateIndexSeek.size(); i++)
        {
            RelateIndex = RelateIndexSeek[i];
            SeekProcess(RelateIndex);
        }
        std::string ret = InputControl::CreatSerial(Data);
    }

    else
    {
        for(int i=0; i<RelateIndexTrack.size(); i++)
        {
            RelateIndex = RelateIndexTrack[i];
            TrackProcess(RelateIndex);
        }
        std::string ret = InputControl::CreatSerial(Data);
    }

    Data.seams.clear();
}


//bool MainWindow2::readData(const char* fname, vector<point3dd>& ps, vector<point3dd>& pa)
//{
//    float x,y,z,a,b,c;
//    ifstream ifs(fname);
//    if ( !ifs )
//    {
//        cout << "can't read file " << fname << endl;
//        return false;
//    }
//    ps.clear();
//    pa.clear();
//    int i = 0;
//    int j = 0;
//    while ( ifs >> x && ifs >> y && ifs >> z && ifs >> a && ifs >> b && ifs >> c ) //&& ifs >> i && ifs >> j)
//    {
//        ps.push_back(point3dd(x,y,z));
//        //cout << x << "  " << y << "  " << z << endl;
//        pa.push_back(point3dd(a,b,c));
//    }
//    return true;
//}


void MainWindow2::fit(vector<RobotPos> &path)
{
    vector<point3dd> points;
    vector<point3dd> angles;
    RobotPos pos;
    for(int i=0; i<path.size(); i++)
    {
        pos = path[i];
        points.push_back(point3dd(pos.x,pos.y,pos.z));
        angles.push_back(point3dd(pos.a,pos.b,pos.c));
    }
    path.clear();

    smoothspline3d cp3d(points, 10);
    cp3d.fit();
    smoothspline3d ca3d(angles, 10);
    ca3d.fit();
    for (double i=0; i<points.size()-1; i++)
    {
        for(double j=0; j<10; j++)
        {
            point3dd p = cp3d.model(i+j*0.1);
            point3dd a = ca3d.model(i+j*0.1);
            pos = {p._x, p._y, p._z, a._x, a._y, a._z};
            path.push_back(pos);
        }
    }
}


void MainWindow2::readData(QString filename, vector<point3dd>& ps, vector<point3dd>& pa)
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

    float x,y,z,a,b,c;
    vector<RobotPos> poslst;
    QStringList list;
    for(int i=0; i<Qstr.size(); i++){
        list = Qstr[i].split(',');
        x = list[0].toDouble();
        y = list[1].toDouble();
        z = list[2].toDouble();
        a = list[3].toDouble();
        b = list[4].toDouble();
        c = list[5].toDouble();
        ps.push_back(point3dd(x,y,z));
        pa.push_back(point3dd(a,b,c));
    }
}

void MainWindow2::on_pushButton_fit_clicked()
{
    vector<point3dd> points;
    vector<point3dd> angles;
    readData("fitData/data4.txt", points, angles);
    cout << "read " <<  points.size() << " entries " << endl;
    smoothspline3d cp3d(points, 5);
    smoothspline3d ca3d(angles, 5);
    cp3d.fit();
    ca3d.fit();
    ofstream ofile("fitData/smoothpts.txt");
    RobotPos pos;
    int N = 1;
    double t = 1/N;
    for ( double i = 0; i < points.size()-1; i+=1)
    {
        for(double j=0; j<N; j++)
        {
            point3dd p = cp3d.model(i+j*t);
            point3dd a = ca3d.model(i+j*t);
            pos = {p._x, p._y, p._z, a._x, a._y, a._z};
            ofile << pos.x << "," << pos.y << "," << pos.z << ","
                  << pos.a << "," << pos.b << "," << pos.c << endl;
        }
    }
    cout << "output written to smoothpts.txt" << endl;
}


void MainWindow2::on_pushButton_circleFit_clicked()
{
    vector<vector<RobotPos>> pathLst, pathLst2;
    vector<vector<vector<RobotPos>>> pathLsts;
    ReadPathLst(pathLst, "pathData.txt");

    // 旋转复制生成多条轨迹
    int Multiple = 10;
    int perAgl = -360/Multiple;
    RobotPos cpos = {0,0,0,0,0,0};
    for(int i=0; i<pathLst.size(); i++)
    {
        for(int j=0; j<Multiple; j++)
        {
            for(auto &r1 : pathLst[i])
                PosRotate(cpos, r1, perAgl);
            pathLst2.push_back(pathLst[i]);
        }
        pathLsts.push_back(pathLst2);
        pathLst2.clear();
    }

    // 焊缝整理
    AScanePos sPos;
    AFramePos fPos;
    QList<AScanePos> scanePos;
    QList<AFramePos> framePos;
    SeamInfo seam;
    QList<SeamInfo> seams;
    MachineGrond  machine;
    double db = 0;
    for(int i=0; i<pathLsts.size(); i++)
    {
        for(int k=0; k<pathLsts[i].size(); k++)
        {
           for(int ki=0; ki<pathLsts[i][k].size(); ki++)
           {
               RobotPos p = pathLsts[i][k][ki];
               sPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
               sPos.index1 = ki;
               sPos.enabled = 1;
               scanePos.push_back(sPos);
           }
           for(int ki=0; ki<pathLsts[i][k].size(); ki++)
           {
               RobotPos p = pathLsts[i][k][ki];
               fPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
               fPos.index1 = ki;
               framePos.push_back(fPos);
           }
           seam.scanePos = scanePos;
           seam.framePos = framePos;
           seam.seamName = "seam"+std::to_string(i+1)+"_"+std::to_string(k+1)+"_0";
           seam.orderIndex = 0;
           seam.rank = 1;
           seam.tracing = 0;
           machine.pos_motor_start = 0;
           machine.pos_motor_stop = 0;
           machine.angle1_motor_start = 0;
           machine.angle1_motor_stop = 0;
           machine.angle2_motor_start = 0;
           machine.angle2_motor_stop = 0;
           seam.machine = machine;
           Data.seams.push_back(seam);
           scanePos.clear();
           framePos.clear();
        }
    }

    std::string ret = InputControl::CreatSerial(Data);


//    bool limit;
//    vector<RobotPos> PosLst_correct;
//    vector<RobotAxle> AglLst;
//    PosLst_correct.push_back({-1126.07, 302.826, 354, 74.837,135.728,-154.157});
//    limit = Pos2Joint(PosLst_correct, AglLst, agl_acl, TInF);
//    WriteAgl(AglLst, "aglLst");


//    vector<RobotPos> path, path2, path3;
//    path.push_back({0,0,0,0,135,90});
//    path2.push_back({0,0,0,0,135,90});
//    path3.push_back({0,0,0,0,135,90});
//    posRotateY(path,40);
//    posRotateY(path2,20);
//    posRotateY(path3,5);
//    path.clear();



//    vector<RobotPos> path;
//    circleFit circlefit;
//    readPath("fitData/cali.txt", path);
//    circlefit.pointLst = path;
//    circlefit.build();
//    path.clear();

//    Matrix4d M;
//    Vector3d x,y,z;
//    RobotPos cpos;
//    z << -1289.823+674.554, 760.731-335.316, 2.610-7.603;
//    y << 0,0,-1;
//    z = z/sqrt(z.dot(z));
//    x = y.cross(z);
//    M << x(0), y(0), z(0), -674.554,
//         x(1), y(1), z(1), 335.316,
//         x(2), y(2), z(2), -72.4,
//         0, 0, 0, 1;
//    Matrix2pos(M, cpos);
//    qDebug() << cpos.x << "," << cpos.y << "," << cpos.z << ","
//             << cpos.a << "," << cpos.b << "," << cpos.c <<  "\n";

//    GetToolCoordinate(cpos, -400, -60, 17);
//    qDebug() << cpos.x << "," << cpos.y << "," << cpos.z << ","
//             << cpos.a << "," << cpos.b << "," << cpos.c <<  "\n";

//    RobotPos tpos;
//    Matrix4d Ma, Mp;
//    pos2Matrix(cposa, Ma);
//    pos2Matrix(posW2, Mp);
//    M = Ma.inverse()*Mp;
//    Matrix2pos(M, tpos);
//    qDebug() << tpos.x << "," << tpos.y << "," << tpos.z << ","
//        << tpos.a << "," << tpos.b << "," << tpos.c <<  "\n";


//    vector<RobotPos> path, path2;
//    vector<vector<RobotPos>> pathLst;
//    for(int i=0; i<149; i++)
//        path.push_back({-230,60+i*5,72, 0,-125,-90});
//    for(int i=0; i<149; i++)
//        path2.push_back({-400,60+i*5,72, 0,-125,-90});
//    posRotateY(path, 10);
//    posRotateY(path2, 10);
//    CreateKeel(path);
//    CreateKeel(path2);
//    pathLst.push_back(path);
//    pathLst.push_back(path2);
//    QString filename3 = "pathlst.txt";
//    Write_File(pathLst, filename3);

//    RobotPos pos = {-230, 60, 72, -167.852, 124.393, 94.2326};
//    Matrix4d Mp, Mc, Mj;
//    pos2Matrix(pos, Mp);
//    pos2Matrix(CInT, Mc);
//    Mp = Mp*Mc;
//    Matrix2pos(Mp, pos);
//    double a, b, c, d, y;
//    a = Mp(0,0);
//    b = Mp(1,0);
//    c = Mp(2,0);
//    d = a*Mp(0,3)+b*Mp(1,3)+c*Mp(2,3);
//    y = (d-a*(-230)-c*72)/b;
//    pos = {-230,y,72,0,0,0};
//    pos2Matrix(pos, Mj);
//    Mj = Mp.inverse()*Mj;
//    Matrix2pos(Mj, pos);
//    qDebug() << pos.x << "," << pos.y << "," << pos.z << ","
//        << pos.a << "," << pos.b << "," << pos.c <<  "\n";





//    RobotPos tpos;
//    RobotPos pos = {-0.588581,1.06407,-179.942,-177.179,0.0800888,174.204};
//    Matrix4d Ma, Mp, M;
//    pos2Matrix(cposa, Ma);
//    pos2Matrix(pos, Mp);
//    M = Ma*Mp;
//    Matrix2pos(M, tpos);
//    qDebug() << tpos.x << "," << tpos.y << "," << tpos.z << ","
//        << tpos.a << "," << tpos.b << "," << tpos.c <<  "\n";

//    bool limit;
//    vector<RobotPos> path;
//    vector<RobotAxle> AglLst1;
//    readPath("data.txt",path);

//    Eigen::Matrix4d mw1, mw2, mr, mt;
//    pos2Matrix(posW2, mw2);

//    for(auto &r : path)
//    {
//        pos2Matrix(r, mr);
//        mt = mw2*mr;
//        Matrix2pos(mt, r);
//    }

//    for(int i=0; i<path.size(); i++)
//    {
//        PosRotate(cposa, path[i], -147.5);
//        PosRotate(cposb, path[i], 0);
//        path[i].x += 325;
//    }
//    limit = Pos2Joint2(path, AglLst1, agl_acl, TInF);
//    path.clear();
//    RobotPos pos = cposa;
//    PosRotate(cposa, pos, -147.5);
//    PosRotate(cposb, pos, 0);
//    pos.x += 325;
//    pos = {0,0,0,0,0,0};






//    vector<RobotAxle> AglLst;

//    for(int ii=0; ii<1000; ii++)
//        AglLst.push_back({0,0,0,0,0,0});
//    QFile file("AglLst2.txt");
//    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
//        return;
//    QTextStream out(&file);
//    out << "position:" <<  "\n";
//    for(int ii=0; ii<(int)AglLst.size(); ii+=10)
//        out << AglLst[ii].a1 << "," << AglLst[ii].a2 << "," << AglLst[ii].a3 << ","
//            << AglLst[ii].a4 << "," << AglLst[ii].a5 << "," << AglLst[ii].a6 <<  "\n";
//    AglLst.clear();



//    Matrix4d M, M2;
//    RobotPos pos,pos2;
//    pos = {-54.765,0.456,-472.643,-1.368,16.239,89.099};
//    pos2Matrix(pos,M);
//    M = M.inverse().eval();

//    pos2 = {-11.876,1.339,97.279,0,0,0};
//    pos2Matrix(pos2,M2);
//    M = M*M2;

//    Matrix2pos(M,pos);
//    Matrix2pos(M,pos);



//    bool limit;
//    vector<RobotPos> PosLst_correct;
//    vector<RobotAxle> AglLst;
//    PosLst_correct.push_back({-1032.189, 865.031, 151.182, 0,180,0});
//    limit = Pos2Joint(PosLst_correct, AglLst, agl_acl, TInF);
//    WriteAgl(AglLst, "aglLst");


//    RobotPos cpos = {-303.918, 796.849, -170.738, 172.854, -99.214, 0};
//    RobotPos cpos = {-1348.984, 717.274, -84.032, 82.955, 90.242, 90.478};
//    cposInZ(cpos, 847.477);
//    cpos.a = 0;

//    vector<RobotPos> path;
//    circleFit circlefit;
//    readPath("fitData/cali.txt", path);
//    circlefit.pointLst = path;
//    circlefit.build();
//    path.clear();

//    circlefit.interpolation(0, 10, path);
//    WritePath(path, "fitData/smoothpts2.txt");
//    readPath("fitData/smoothpts2.txt", path);
//    circlefit.pointLst = path;
//    circlefit.build();
//    path.clear();

//    readPath("fitData/smoothpts.txt", path);
//    circlefit.pointLst = path;
//    circlefit.build();
//    path.clear();






//    // 插入过渡轨迹得到新的轨迹
//    vector<RobotPos> path, path2;
//    vector<vector<RobotPos>> pathLst;
//    Read_pathlst(pathLst, "huaGuangO2.txt");

//    // 修改姿态（绕世界坐标系旋转）
//    for(int j=0; j<pathLst[0].size(); j++)
//    {
//        PosRotateZW(pathLst[0][j],pathLst[0][j],-7*(6-j)+28);
////        if(j>=8)
////        {
////            PosRotate(pathLst[0][j],pathLst[0][j],20*(j-8));
////        }
//    }

//    for(int i=1; i<pathLst.size(); i++)
//    {
//        for(int j=0; j<pathLst[i].size(); j++)
//        {
//            pathLst[i][j].a = pathLst[0][j].a;
//            pathLst[i][j].b = pathLst[0][j].b;
//            pathLst[i][j].c = pathLst[0][j].c;
//        }
//    }

//    // 生成龙骨数据
//    for(int i=0; i<pathLst.size()-1; i++)
//    {
//        for(int j=0; j<pathLst[i].size(); j++)
//            path.push_back(pathLst[i][j]);
//        creatTransPos(pathLst[i][pathLst[i].size()-1], pathLst[i+1][0], path2);
//        for(int j=0; j<path2.size(); j++)
//            path.push_back(path2[j]);
//    }
//    for(int j=0; j<pathLst[pathLst.size()-1].size(); j++)
//        path.push_back(pathLst[pathLst.size()-1][j]);

//    // 将新的轨迹写入文件
////    ofstream ofs("huaGuang.txt");
////    for(int j=0; j<(int)path.size(); j++)
////        ofs << "P00"+std::to_string(j+1) << "," << path[j].x << "," << path[j].y << "," << path[j].z << ","
////            << path[j].a << "," << path[j].b << "," << path[j].c <<  "\n";


//    // 转至虚拟坐标系下
//    RobotPos posW1 = {0,1355,0,90,90,-90};
//    posInTool(path, posW1);


//    // 寻找可行解
//    vector<RobotPos> clst;
//    huaGuang(path, clst);
//    ofstream ofs("clst.txt");
//    for(int j=0; j<(int)clst.size(); j++)
//        ofs << clst[j].x << "," << clst[j].y << "," << clst[j].z << ","
//            << clst[j].a << "," << clst[j].b << "," << clst[j].c <<  "\n";


//    // 填入DB
//    RepeatData Data;
//    AScanePos sPos;
//    AFramePos fPos;
//    QList<AScanePos> scanePos;
//    QList<AFramePos> framePos;
//    SeamInfo seam;
//    for(int ki=0; ki<path.size(); ki++){
//        RobotPos p = path[ki];
//        sPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
//        sPos.index1 = ki;
//        if(ki==0 || ki==path.size()-1)
//            sPos.enabled = 0;
//        scanePos.push_back(sPos);
//    }
//    for(int ki=0; ki<path.size(); ki++){
//        RobotPos p = path[ki];
//        fPos.pos = Pos(p.x, p.y, p.z, p.a, p.b, p.c);
//        fPos.index1 = ki;
//        framePos.push_back(fPos);
//    }
//    seam.scanePos = scanePos;
//    seam.framePos = framePos;
//    Data.seams.push_back(seam);

//    std::string ret = InputControl::CreatSerial(Data);
}


void MainWindow2::huaGuang(vector<RobotPos> path, vector<RobotPos> &clst)
{
    bool limit;
    Matrix4d Mc, Mt, M;
    RobotPos pos, cpos;
    vector<RobotPos> path2;
    vector<RobotAxle> aglLst;
//    cpos = {-48.037, 1429.959, -142.201, -34.288, 89.870, -101.366};
    cpos = {-34.701, 1405.586, 125.570, -34.768, 87.100, -90.440};
    pos2Matrix(cpos, Mc);
    TInF = {-2.5, 87.8, 540.3, 85.6, 13.9,-175.3};
    vector<int> idLst;
    int idx = 0;
    int id;
    for(int z=250; z<260; z+=10)
    {
        for(int y=0; y<1; y+=20)
        {
            for(int agl=0; agl<1; agl+=3)
            {
                aglLst.clear();
                path2.clear();
                path2 = path;
                pos = {0,y,z,0,0,agl};
                pos2Matrix(pos, Mt);
                for(auto &r : path2)
                {
                    pos2Matrix(r, M);
                    M = Mt*Mc*M;
                    Matrix2pos(M, r);
                }
                limit = Pos2Joint(path2, aglLst, agl_acl, TInF);
                if(aglLst.size()>=89){
                    clst.push_back(pos);
                    idLst.push_back(int(aglLst.size()));
                }
            }
        }
    }
    for(int i=0; i<idLst.size(); i++)
    {
        if(idx<idLst[i])
        {
            idx = idLst[i];
            id = i;
        }
    }
    path2.clear();
}


void MainWindow2::creatTransPos(RobotPos Ps, RobotPos Pe, vector<RobotPos> &path)
{
    double len, dx, dy, dz;
    RobotPos pos;
    vector<RobotPos> path2;
    len = sqrt(pow(Pe.x-Ps.x,2)+pow(Pe.y-Ps.y,2)+pow(Pe.z-Ps.z,2));
    dx = (Pe.x-Ps.x)/len;
    dy = (Pe.y-Ps.y)/len;
    dz = (Pe.z-Ps.z)/len;
//    int Num = len/5;
    int Num = 7;
    len = len/Num;
    path.clear();
    path.push_back(Ps);
    path.push_back(Pe);
    path2.push_back(Ps);
    for(int i=1; i<Num; i++)
    {
        pos.x = Ps.x + i*len*dx;
        pos.y = Ps.y + i*len*dy;
        pos.z = Ps.z + i*len*dz;
        path2.push_back(pos);
    }
    path2.push_back(Pe);
    interpolation(path2, path);
    path.clear();
    for(int i=1; i<path2.size()-1; i++)
        path.push_back(path2[i]);
}
