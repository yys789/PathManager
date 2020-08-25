#ifndef DBINTERFACE_H
#define DBINTERFACE_H
#include <map>
#include <vector>
#include <string>
#include "base/data_types.h"
#include <iostream>
#include "base/cworkpiecedata.h"

using namespace std;

void getExternInfo(int seamID, map<string,string>& mExternInfo);
void getSeamInfo(int seamID,std::map<std::string,std::string>& mSeamInfo);
void getComInfo(std::map<std::string,std::string>& mComInfo);
void getScanPos(int seamId,std::vector<RobotPos>& scanPath);
void getFramePos(int seamId,std::vector<RobotPos>& framePos);
void getRelatedSeamsInfo(int seamId,RelateInfo & rif);
void getMidPointById(int seamId,std::string& midpoint,RobotAxle& axle);
void getSeamWeldInfo(int seamId,std::vector<WELDINFO>& weldInfo);
void getSeamMotorAngleInfo(int seamId,WELDANGLEINFO& angleInfo);
void getCadInfo(std::string cadId,std::map<std::string,std::string>& cadInfo,RobotPos& pos);
void getCadInfo(std::string cadId,RobotPos& pos);


void updateSeamStatus(int seamId,int goodSid,SEAMSTATUS st);
void updateComInfo(std::string columnName,std::string value);



#endif // DBINTERFACE_H
