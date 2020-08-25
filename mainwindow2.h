#ifndef MAINWINDOW2_H
#define MAINWINDOW2_H

#include <QMainWindow>
#include <vector>
#include <QFile>
#include <QTextStream>
#include <Eigen/Dense>
#include<time.h>
#include <QDebug>
#include "Proto/DataControl/baseTypes.h"
#include "Proto/DataControl/InputControl.h"
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include<io.h>
#include<fstream>
#include<iostream>
#include <sstream>
#include<string>
#include <QStringListModel>
#include "dialog.h"
#include "include/curveFit.h"
#include "include/circleFit.h"
#include <fitHead/smoothspline3d.h>

using namespace std;
using namespace Eigen;
using std::vector;
using namespace walgo;

//struct RobotPos
//{
//    double x, y, z, a, b, c;
//};
//struct RobotAxle
//{
//    double a1,a2,a3,a4,a5,a6;
//};

namespace Ui {
class MainWindow2;
}

class MainWindow2 : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow2(QWidget *parent = 0);
    ~MainWindow2();
    void readConfig(string configPath,
                                vector<vector<int>> RelateIndex,
                                vector<vector<int>> &pathDirection,
                                vector<vector<int>> &laserDirection,
                                vector<vector<int>> &rotateYEnd,
                                vector<vector<int>> &rotateYHead,
                                vector<vector<double>> &region, vector<vector<string> > &nameLsts);
    void readConfig2(string configPath,
                                vector<vector<int>> RelateIndex,
                                vector<vector<int>> &pathDirection,
                                vector<vector<int>> &laserDirection,
                                vector<vector<int>> &rotateYEnd,
                                vector<vector<int>> &rotateYHead,
                                vector<vector<double>> &region,
                                vector<vector<string>> &nameLsts,
                                 vector<vector<int>> &orderIDLsts,
                                 vector<vector<int>> &rankLsts,
                                 vector<vector<int>> &TracingLsts);
    void readConfig3(string configPath,
                     vector<vector<int>> RelateIndex,
                     vector<vector<int>> &pathDirection,
                     vector<vector<int>> &laserDirection,
                     vector<vector<int>> &rotateYEnd,
                     vector<vector<int>> &rotateYHead,
                     vector<vector<string>> &nameLsts,
                     vector<vector<int>> &orderIDLsts,
                     vector<vector<int>> &rankLsts,
                     vector<vector<int>> &TracingLsts);
    void readConfig_RobotPos(string configPath, string name, RobotPos &pos);
    void readConfig_agl(string configPath, string name, double &aglWeld, double &aglInclude, double &aglHead, double &aglEnd, double &aglA, double &aglT);
    void readConfig_Str(string configPath, string name, string petName, string &Str);
    void readConfig_Dou(string configPath, string name, string petName, double &Dou);
    void readConfig_Int(string configPath, string name, string petName, int &Int);
    void readConfig_Find(string configPath, string name, string petName);
    void readConfig_Seam(string configPath,
                         vector<vector<vector<int>>> &RelateIndexSeek, vector<vector<vector<int>>> &RelateIndexTrack);
    void readConfig_rgn(string configPath, string name, vector<vector<double>> &region);
    void findStartAngle(RobotPos pos, vector<double> &rotateRegion, vector<double> &flipRegion);
    void RetracementProcess(vector<RobotPos> &path, double distance, int backPosNum);
    void interpolation(vector<RobotPos> &seam, vector<RobotPos> pos_set);
    void Matrix2pos(Eigen::Matrix4d &m,  RobotPos &pos);
    void pos2Matrix(const RobotPos &pos, Eigen::Matrix4d &m);
    bool Pos2Joint(vector<RobotPos> Pos_set, vector<RobotAxle> &agl_set, RobotAxle agl_acl, RobotPos AInB);
    bool Pos2Joint2(vector<RobotPos> Pos_set, vector<RobotAxle> &agl_set, RobotAxle agl_acl, RobotPos AInB);
    void Read_pathlst(vector<vector<RobotPos>> &PathLst, QString filename);
    void ReadPathLst(vector<vector<RobotPos>> &PathLst, QString filename);
    void PosRotate(RobotPos cpos, RobotPos &pos, double agl);
    void PosRotateZW(RobotPos cpos, RobotPos &pos, double agl);
    void Write_File(vector<vector<RobotPos>> pathLst, QString filename);
    void WritePath(vector<RobotPos> path, QString filename);
    void WriteAgl(vector<RobotAxle> AglLst, QString filename);
    void Weld2Path(vector<RobotPos> &path, vector<int> direction);
    void pathMode(vector<RobotPos> path, vector<vector<RobotPos>> &pathlst, vector<int> direction, vector<vector<int>> &directionLst);
    bool pathModeTest(vector<vector<RobotPos>> pathlst, vector<vector<int>> directionLst, vector<int> &direction, double &evaluate);
    void WeldClassify9(vector<vector<vector<RobotPos>>> pathlst_s,
                                   vector<vector<double>> region,
                                   vector<vector<int>> &WeldClassifyPlan,
                                   vector<vector<vector<int>>> &directionLst_s,
                                   vector<vector<int>> pathDirection,
                                   vector<vector<int>> laserDirection,
                                   vector<vector<int>> rotateYHead,
                                   vector<vector<int>> rotateYEnd);
    void ShipForm_Weld(vector<vector<RobotPos>> pathlst, vector<vector<double>> region, vector<vector<double> > &StateWithSpeedLst, double &weldTime, vector<int> Variables, double WeldSpeed, int pso_Num, int time, int splitNumS, int splitNumE);
    void PostureAdjustment(vector<RobotPos> &path);
    void PostureAdjustment2(vector<RobotPos> &path);
    void KeeldataReorganize(vector<RobotPos> &path);
    void CreateKeel(vector<RobotPos> &path);
    void CreateKeel2(vector<RobotPos> &path);
    void CreateKeelScan(vector<RobotPos> &path);
    void pathReverse(vector<RobotPos> &path);
    void laserReverse(vector<RobotPos> &path);
    void posInTool(vector<RobotPos> &path, RobotPos posW);
    void posInBase(vector<RobotPos> &path,
                               RobotPos posW1,  //CAD工件坐标系
                               RobotPos posW2  //实际工件坐标系
                               );
    double RobotAttitudEvaluate2(vector<RobotAxle> aglLst, vector<RobotPos> path, RobotAxle aglBest);
    void pathEndRotate(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection);
    void pathHeadRotate(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection);
    double posRotateX(vector<RobotPos> &path, double rotataAgle);
    double posRotateY(vector<RobotPos> &path, double rotataAgle);
    double posRotateZ(vector<RobotPos> &path, double rotataAgle);
    void RelateProcess(vector<vector<RobotPos>> PathLst, vector<vector<int>> RelateIndex, vector<vector<vector<RobotPos>>> &RelatePathLst_s);
    void SeekProcess(vector<vector<int>> RelateIndex);
    void TrackProcess(vector<vector<int>> RelateIndex);
    void GetToolCoordinate(RobotPos &TPos, double orbitX, double rotateAgl, double flipAgl);
    void base2Cad(vector<RobotPos> &path,
                  RobotPos posW1,  //CAD工件坐标系
                  RobotPos posW2  //实际工件坐标系
                               );
    void SeekProcessByMultiple(vector<vector<int>> RelateIndex);
    void SeekProcessByTeach(vector<vector<int>> RelateIndex);
    void TrackProcessDirect(vector<vector<int>> RelateIndex);
    void Smooth(vector<RobotPos> &path, vector<RobotPos> &pathRf);
    void Write2Database_Seek(RepeatData &Data, vector<vector<int>> WeldClassifyPlan,
                             double dx, double da,
                             vector<vector<double>> region, vector<vector<string>> nameLsts,
                             vector<vector<int>> &orderIDLsts,
                             vector<vector<int>> &rankLsts,
                             vector<vector<int>> &TracingLsts,
                             vector<vector<vector<RobotPos>>> pathLst_sS,
                             vector<vector<vector<RobotPos>>> pathLst_sK);
    void Write2Database_Track(RepeatData &Data, vector<vector<double>> StateWithSpeedLst,
                              vector<int> index, vector<vector<string>> nameLsts,
                              vector<vector<int>> &orderIDLsts,
                              vector<vector<int>> &rankLsts,
                              vector<vector<int>> &TracingLsts,
                              vector<double> weldTimeLst,
                              vector<vector<vector<RobotPos>>> pathLst_sS,
                              vector<vector<vector<RobotPos>>> pathLst_sK,
                              double aglG, int splitNumS, int splitNumE);
    void Write2Database_FangGuan(RepeatData &Data, vector<vector<double>> StateWithSpeedLst,
                              vector<int> index, vector<vector<string>> nameLsts,
                              vector<vector<int>> &orderIDLsts,
                              vector<vector<int>> &rankLsts,
                              vector<vector<int>> &TracingLsts,
                              vector<double> weldTimeLst,
                              vector<vector<vector<RobotPos>>> pathLst_sS,
                              vector<vector<vector<RobotPos>>> pathLst_sK,
                              double aglG, int splitNumS, int splitNumE);
    void readData(QString filename, vector<point3dd>& ps, vector<point3dd>& pa);
    void readPath(QString filename, vector<RobotPos> &path);
    void fit(vector<RobotPos> &path);
    void cposInZ(RobotPos &cpos, double posX);
    void creatTransPos(RobotPos Ps, RobotPos Pe, vector<RobotPos> &path);
    void huaGuang(vector<RobotPos> path, vector<RobotPos> &clst);
    bool PhaseJudgment(vector<RobotPos> path, RobotPos cpos);


private slots:
    void on_pushButton_GetWorkIfo_clicked();

    void on_cbxMode_activated(const QString &arg1);

    void on_cbxState_activated(const QString &arg1);

    void on_cbxRelate_activated(const QString &arg1);

    void on_pushButton_goState_clicked();

    void on_pushButton_goRelation_clicked();

    void on_listView_doubleClicked(const QModelIndex &index);

    void on_pushButton_goto_clicked();

    void on_pushButton_goMode_clicked();

    void on_pushButton_fit_clicked();

    void on_pushButton_circleFit_clicked();

private:
    Ui::MainWindow2 *ui;
    RobotPos cposa;
    RobotPos cposb;
    RobotPos TInF;
    RobotPos CInF;
    RobotPos CInT;
    RobotPos posW1; // CAD工件坐标系
    RobotPos posW2; // 实际工件坐标系
    RobotAxle aglBest;
    RobotAxle agl_acl;
    int backPosNum; //回撤pos数目
    double backDistance;
    string configPath, configPath2;
    QString filename, filename2;
    double aglWeld, aglA, aglT, aglInclude, aglEnd, aglHead;
    string CadName;
    vector<vector<vector<int>>> RelateIndexSeek;
    vector<vector<vector<int>>> RelateIndexTrack;
    Dialog *dialog;
    int interval; // 龙骨间隔
    RepeatData Data;
};

#endif // MAINWINDOW2_H
