#ifndef COORATION_H
#define COORATION_H

#include "fit/sepmodel3d.h"
#include "walgo/d2seamdetector.h"
#include "tools.h"
#include "tools/matrix.h"
#include "cworkpiecedata.h"

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace walgo;

#define TRACK_RANK 2
#define START_POINT_LIMIT 10
#define START_LEN_LIMIT 4
#define LINE_2_LINE 5
#define FIT_SPAN 6
#define TRACK_POINT_LIMIT 5
#define START_POINT_SPEED 100
#define START_SCAN_SPEED 10

class  MultiSepModel
{
public:
     MultiSepModel(){}
     point3dd at(double t);
     bool build(vector<RobotPos> data,double rank=2);
     double getRMS() const;
     double distense;
     vector<vector<int>> lastTvec; /// 最後保留的有效点序列(多段)
     vector<vector<vector<int>>> cutVecs;  /// 多段 多次 剔除的点序列
private:
     double rms;     // 平均rms
     vector<SepModel3D> sepModels; /// 分段函数列表
};

class CircleModel : MultiSepModel
{
public:
    CircleModel(){}
    point3d<double> at(double t);
    point3d<double> abc(double angle, double t);
    bool build(vector<RobotPos> data);
    double getMis();
    double getRMS() const;
    void paramInit(const double* x);
    double getLen();
    double getR();
    string printCPos();
private:
    void buildEnds();
    point3d<double> pabc;
    double minDis(RobotPos p) const;
    vector<RobotPos> init_points; /// init
    vector<RobotPos> points;
    vector<RobotPos> ty_points;
    RobotPos cPos;
    double r;
};

class UV2path
{
public:
    UV2path();
    void setMUV(map<int,Point2d>  v);
    void setLUV(map<int,Point2d>  v);
    void setRUV(map<int,Point2d>  v);
    void setPos(map<int,RobotPos> p);
    void setSeamName(string seamName);
    void setCurCad(string cadName);
    void setDbInfos(map<string,string> db);
    string getInfos(string key);
    void setCurTime(string time);
    bool get3dPath(vector<RobotPos> & path);
private:
    map<string,string> dbInfos;
    map<int,Point2d> pts;
    map<int,Point2d> lts;
    map<int,Point2d> rts;
    map<int,RobotPos> rbts;
    string curSeam;
    string curCad;
    string currTime;
};

UV getJUV(cv::Mat& m,int p,int thresh,string time);
point3d<double> getDir(RobotPos target, MultiSepModel msm, double & t);
RobotPos getModelP(RobotPos target, MultiSepModel msm, double t);
bool fitSeamPath(string path, vector<RobotPos> & seam0,int rank0=2);
bool correlationBySep(string logPath, vector<RobotPos> & seam0, int rank0, RelateInfo rif);
bool tracing_end_correlation(string name, std::vector<RobotPos> & oriPath,
                  std::vector<RobotPos> seam2,double l2,double gap2,
                  std::vector<RobotPos> & weldVec,const double p2p_len);
///初始跟踪轨迹拟合
bool initStartPath(string logPath, std::vector<RobotPos> oriPath,
                            std::vector<RobotPos> seamKeel,
                            std::vector<RobotPos>& weldVec,const double p2p_len,
                   RelateInfo rif);
/// 从龙骨数据中提取，替代图像异常的pos
bool getPosFromKeel(RobotPos & keel, std::vector<RobotPos> oriPath,
                    std::vector<RobotPos> weldVec,
                    std::vector<RobotPos> seamKeel,const double p2p_len);
///  p3d 一簇点 p0 要找的中间点
bool getCenter(vector<RobotPos> p3d, RobotPos & p0);
/// 跟踪因图像异常退出前，将轨迹拟合结束
void fitTracingErrEnd(ofstream & ofs,std::vector<RobotPos>& oriPath,
                      const std::vector<RobotPos> seamKeel,
                      std::vector<RobotPos>& weldVec,const double p2p_len,double endlen=10);
/// 将图像或龙骨数据融入到原始数据队列，并拟合扩展焊接轨迹
bool appendWeldPath(string logPath, RobotPos ap,ofstream & ofs,std::vector<RobotPos> & oriPath
                             ,const std::vector<RobotPos> seamKeel,std::vector<RobotPos> & weldVec,const double p2p_len,
                    RelateInfo rif,double endlen=10,double c2tx=33, double fit_ps=4);
/// 从曲线方程中获取前一个点后新的一个点;sm 曲线方程;pos 曲线上的pos;pte 要计算的点前一个pos
bool getModelPos(SepModel3D sm, point3dd & pos,point3dd pte,point3dd ppte,const double p2p_len);
bool getCirclePos(CircleModel cm, point3dd & pos,point3dd pte,point3dd ppte,const double p2p_len);
/// weld params init
void weldEmpi();

/// 自适应焊接参数
void weldAdaption(double gap, RobotPos & p, weldInfo & wf);

/// 檢查UV有效性
bool checkUV(UV tar,UV center);

/// 自动扩展轨迹算法
void externPath3(vector<RobotPos> path,
                 vector<RobotPos> keel0,
                 vector<RobotPos> & keel1,
                 vector<RobotPos> & keel2,
                 vector<RobotPos> & keel3);

/// 根据时间推算各个变位机角度 以后换成高速编码器的值，可以弃用
MOTORDEGRE getAngleByTime(double curTime, WELDANGLEINFO angleInfo);
#endif // COORATION_H
