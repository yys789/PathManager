#ifndef CWorkPieceData_H
#define CWorkPieceData_H
#include "base/data_types.h"

using namespace std;
using namespace cv;

//0.未处理 1.扫描 2.图像处理 3.焊接 4.焊接完成 5.焊接失败
enum SEAMSTATUS{NOTSCANNED = 0,SCANING,IMAGEPROCESSING,STARTWELDING,WELDINGSUCCESS,WELDINGFAILED};
///焊縫的標志信息
struct seamMark
{
    int seamID;
    int orderID;
    string name;//焊縫名字
};

bool comp(seamMark& sm1,seamMark& sm2);

struct SensorDataPack
{
    Mat m;         /// 图像
    RobotPos r;    /// 机器人pos
    double time;   /// 记录时间
    int cameraMode; /// 传感器状态 0 寻位状态 1跟踪状态
};

typedef struct RELATEDSEAMINFO
{
    int seamId;
    double spacing;
    double distance;
}relatedInfo;

struct WELDINFO
{
    RobotPos offset;
    RobotPos base_offset;
    weldInfo WI;
};

typedef struct RELATEINFO
{
    int beginId;         /// 起头焊缝编号　0 无关联
    int beginExtern;     /// 起头延展
    int beginDistance;   /// 起头空间距离
    double beginRank;    /// 起头轨迹特征
    vector<RobotPos> beginPath;/// 起头轨迹
    int endId;           /// 结尾焊缝编号　0 无关联
    int endExtern;       /// 结尾延展
    int endDistance;     /// 结尾空间距离
    double endRank;      /// 结束轨迹特征
    vector<RobotPos> endPath;/// 起头轨迹
    double L;            /// 长度
}RelateInfo;  /// 关联信息

class CWorkPieceData
{
public:
    CWorkPieceData();
    ~CWorkPieceData();
    void init();
    void weldList_sort();
    void release_pair_vec();
    void release_map_pair();
    void release_AllSeamPathMap();
    void saveImagesAll();
    void saveImages();
public:
    vector<SensorDataPack> pair_vec;///單條焊縫的掃描信息
    std::map<int,vector<SensorDataPack>> map_pair;///主焊縫以及關聯焊縫的掃描信息
    std::map<int,std::vector<RobotPos>> AllSeamPathMap;///所有焊縫的焊接路徑
    string m_sCADID;///当前工件名
    seamMark sMark;///当前焊缝的标识结构体
    std::map<int,string> seamMap;///seamid 和seamName的对应关系
    string sCurrTime;///当前时间
};

#endif // CWorkPieceData_H
