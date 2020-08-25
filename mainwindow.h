#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector>
using std::vector;
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
#include "include/defines.h"

#include<io.h>
#include<fstream>
#include<iostream>
#include <sstream>
#include<string>

using namespace std;
using namespace Eigen;

//struct RobotPos
//{
//    double x, y, z, a, b, c;
//};
//struct RobotAxle
//{
//    double a1,a2,a3,a4,a5,a6;
//};
struct point3d
{
    double _x, _y, _z;
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void Matrix2pos(Eigen::Matrix4d &m,  RobotPos &pos);
    void pos2Matrix(const RobotPos &pos, Eigen::Matrix4d &m);
    bool Pos2Joint(vector<RobotPos> Pos_set, vector<RobotAxle> &agl_set, RobotAxle agl_acl, RobotPos AInB);
    bool Pos2Joint2(vector<RobotPos> Pos_set, vector<RobotAxle> &agl_set, RobotAxle agl_acl, RobotPos AInB);
    void Read_File(vector<RobotPos> &Pos_set, QString filename, QString KeyWord);
    void Read_pathlst(vector<vector<RobotPos>> &PathLst, QString filename);
    void Read_pathlst2(vector<vector<RobotPos>> &PathLst, QString filename);
    void Write_Filec(vector<vector<RobotPos>> pathLst, string filename);
    void Write_File(vector<vector<RobotPos>> pathLst, QString filename);
    void Write_File2(vector<vector<vector<RobotPos>>> pathLst_s, vector<vector<double> > platform, QString filename);
    void Write_File3(vector<vector<vector<RobotPos>>> pathLst_s,
                                 vector<vector<vector<RobotPos>>> pathLst_s2,
                                 vector<vector<vector<RobotAxle>>> AglLst_ss,
                                 vector<vector<double>> platform,
                                 QString filename);
    double Evaluation_function(vector<RobotAxle> agllst, vector<double> Operating_agl, vector<double> Walking_agl);
    void PosRotate(RobotPos cpos, RobotPos &pos, double agl);
    void WeldClassify7(vector<vector<vector<RobotPos>>> pathlst_s, vector<vector<double>> region, vector<vector<int>> &WeldClassifyPlan);
    void WeldClassify8(vector<vector<vector<RobotPos>>> pathlst_s, vector<vector<double>> region, vector<vector<int>> &WeldClassifyPlan, vector<vector<int> > &laserDirection, vector<vector<int> > rotateY);
    void PostureAdjustment(vector<RobotPos> &path);
    void PostureAdjustment2(vector<RobotPos> &path);
    void pathReverse(vector<RobotPos> &path);
    void laserReverse(vector<RobotPos> &path);
    void interpolation(vector<RobotPos> &seam, vector<RobotPos> pos_set);
    void ShipForm_Weld(vector<vector<RobotPos>> pathlst, vector<vector<double>> region, vector<double> &StateWithSpeed, double &weldTime, vector<int> Variables, double WeldSpeed, int pso_Num, int time, int splitNum);
    void ShipForm_Weld2(vector<vector<RobotPos>> pathlst, vector<vector<double>> region, vector<double> &StateWithSpeed, double &weldTime, vector<int> Variables, double WeldSpeed, int pso_Num, int time, int splitNum);
    void RelateProcess3(vector<vector<RobotPos>> PathLst, vector<vector<int>> RelateIndex, vector<vector<vector<RobotPos>>> &RelatePathLst_s);

    void CreateKeel(vector<RobotPos> &path);
    void CreateKeel2(vector<RobotPos> &path);
    void CreateKeelScan(vector<RobotPos> &path);
    void KeeldataReorganize(vector<RobotPos> &path);
    void WorkpieceCoordinates(RobotPos posO, RobotPos posX, RobotPos &posW);
    void posInTool(vector<RobotPos> &path, RobotPos posW);
    void posInBase(vector<RobotPos> &path, RobotPos posW1, RobotPos posW2);
    double RobotAttitudEvaluate2(vector<RobotAxle> aglLst, vector<RobotPos> path, RobotAxle aglBest);
    void pathEndRotate(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection);
    void pathHeadRotate(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection);
    double posRotateY(vector<RobotPos> &path, double rotataAgle);
    double posRotateX(vector<RobotPos> &path, double rotataAgle);
    void pathEndRotateX(vector<RobotPos> &path, double rotataAgle, int posNum, int rotateDirection);
    void RetracementProcess(vector<RobotPos> &path, double distance, int backPosNum);
    void WeldClassify9(vector<vector<vector<RobotPos>>> pathlst_s,
                                   vector<vector<double>> region,
                                   vector<vector<int>> &WeldClassifyPlan,
                                   vector<vector<vector<int>>> &directionLst_s,
                                   vector<vector<int> > pathDirection,
                                   vector<vector<int> > laserDirection,
                                   vector<vector<int>> rotateYHead,
                                   vector<vector<int>> rotateYEnd);
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
    void readConfig_RobotPos(string configPath, string name, RobotPos &pos);
    void readConfig_agl(string configPath, string name, double &aglWeld, double &aglInclude, double &aglHead, double &aglEnd);
    void readConfig_Str(string configPath, string name, string petName, string &Str);
    void readConfig_Dou(string configPath, string name, string petName, double &Dou);
    void readConfig_Int(string configPath, string name, string petName, int &Int);
    void readConfig_Find(string configPath, string name, string petName);
    void readConfig_Seam(string configPath,
                         vector<vector<vector<int>>> &RelateIndexSeek, vector<vector<vector<int>>> &RelateIndexTrack);
    void Weld2Path(vector<RobotPos> &path, vector<int> direction);
    void pathMode(vector<RobotPos> path, vector<vector<RobotPos>> &pathlst, vector<int> direction, vector<vector<int>> &directionLst);
    bool pathModeTest(vector<vector<RobotPos>> pathlst, vector<vector<int>> directionLst, vector<int> &direction, double &evaluate);
    void findStartAngle(RobotPos pos, vector<double> &rotateRegion, vector<double> &flipRegion);
    void GetToolCoordinate(RobotPos &TPos, double orbitX, double rotateAgl, double flipAgl);

    void getSeamType(const string file, vector<string> &seamTypeLst);
    void getSeamData(const string file, string seamType, vector<vector<RobotPos>> &pathLst);
    int getSeamTypeFromFolder(string path, vector<string> &seamTypeLst);
    int getSeamDataFromFolder(string path, string seamType, vector<vector<RobotPos>> &pathLst);
    void base2Cad(vector<RobotPos> &path,
                  RobotPos posW1,  //CAD工件坐标系
                  RobotPos posW2  //实际工件坐标系
                               );
    void cmove(vector<RobotPos> path, int n1, int n2, vector<RobotPos> &path1, vector<RobotPos> &path2);
    void getKeelPos(vector<RobotPos> &path);
    void getScanPos(vector<RobotPos> &path);
    void Smooth(vector<RobotPos> &path, vector<RobotPos> &pathRf);
    void Write2Database_Seek(RepeatData &Data, vector<vector<int>> WeldClassifyPlan, double dx, double da,
                                    vector<vector<double>> region, vector<vector<string>> nameLsts,
                                    vector<vector<vector<RobotPos>>> pathLst_sS, vector<vector<vector<RobotPos>>> pathLst_sK);
    void Write2Database_Track(RepeatData &Data, vector<vector<double>> StateWithSpeedLst,
                                    vector<int> index, vector<vector<string>> nameLsts, vector<double> weldTimeLst,
                                    vector<vector<vector<RobotPos>>> pathLst_sS, vector<vector<vector<RobotPos>>> pathLst_sK);

private slots:
    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_UnLinkage_clicked();

    void on_Linkage_clicked();

    void on_pushButton_6_clicked();

private:
    Ui::MainWindow *ui;
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
    string configPath;
    QString filename;
    double aglWeld, aglInclude, aglEnd, aglHead;
    string CadName;
};

#endif // MAINWINDOW_H
