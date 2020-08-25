#ifndef _TOOLS_H
#define _TOOLS_H

#include <cmath>

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/exception/all.hpp>
#include <boost/signals2/signal.hpp>

#include "tools/singleton.h"
#include "tools/laser_yz.h"
#include "tools/coord.h"

#include "base/config.h"
#include "opencv2/opencv.hpp"

#include "walgo/point.h"

using namespace Eigen;
using namespace std;

/// int转换字符串正数正常返回，负数或0返回空字符串
std::string int2str(int m);

/// 字符串转RobotPos
bool str2Robot(std::string str, RobotPos & f);

std::vector<float> fitPath(const std::vector<RobotPos> , std::vector<RobotPos> &) ;

std::vector<UV> findUV(std::vector<UV> seamUV) ;

extern boost::signals2::signal<UV (cv::Mat)> manaualDetectUV_sig ;
/// 手动检测图片UV
UV manaualDetectUV( const cv::Mat &image ) ;

std::string getCurrentTime();

std::string getDate();
/// 保存图片到硬盘
/// @param [in] path 保存路徑，默认为[picture]目录
/// @param [in] name 保存的文件名，默认为当前时间点
/// @param [in] mat 需要保存的图片
void saveImg( const cv::Mat &mat, const std::string &path = "", const std::string &name = "") ;

/// 转3D的模式
void UV2Point3D(const RobotPos &currPos, struct UV uv, RobotPos &posOut,int mode=0) ;

double stringToNum(const std::string& str);

std::string DecToBin(int n);

int BinToDec(std::string str);

UV strToUV(std::string str);

void sPosTodVec(std::string sPos,std::vector<double>& vec);

void sPosTodPos(std::string sPos,RobotAxle& axle);

void sPosTodPos(std::string sPos,RobotPos& pos);

std::string AscToString(std::vector<int>& vec);

std::string AscToChar(int n);

void stringToAsc(std::vector<int>& vec,std::string str);

void charToAsc(int& n,std::string str);

RobotAxle mapToRobotAxle(std::map<std::string,std::string>& m);

RobotPos mapToRobotPos(std::map<std::string,std::string>& m);

void IntToSeamName(int n,std::string& str);

int SeamNameToInt(std::string& seamname);

/// 获取系统UNIX时间（精确到ms）
double getMicTime();


template<typename T>
inline T labs( const T & x ){return x<0?-x:x;}
template<typename T>
inline int sgn( const T & x ){return x<0?-1:(x?1:0);}
/// double 四舍五入 成 int
inline int d_round( const double & x ){return (int)(sgn(x)*(labs(x)+0.50001));}

void itranf(const cv::Mat &img,cv::Mat &red,int w= 900,int h= 1080);
void ptranf(cv::Point2d &uv,int w= 900);

#endif
