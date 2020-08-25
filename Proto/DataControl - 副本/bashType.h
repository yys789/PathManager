#ifndef BASHTYPE_H
#define BASHTYPE_H

#include <iostream>
#include <QList>
#include <fstream>
#include <strings.h>




struct Pos{
    Pos(){}
    Pos(double _x,double _y,double _z,double _a,double _b,double _c){
        x = fabs(_x) < 0.001 ? 0: _x;
        y = fabs(_y) < 0.001 ? 0: _y;
        z = fabs(_z) < 0.001 ? 0: _z;
        a = fabs(_a) < 0.001 ? 0: _a;
        b = fabs(_b) < 0.001 ? 0: _b;
        c = fabs(_c) < 0.001 ? 0: _c;
    }

    Pos(double _x,double _y,double _z,double _a,double _b,double _c,bool test){
        x =  _x;
        y = _y;
        z =  _z;
        a =  _a;
        b =  _b;
        c =  _c;
    }
    double   x = 0;
    double   y  = 0;
    double   z = 0;
    double   a  = 0;
    double   b = 0;
    double   c = 0;
};

typedef  QList<Pos>  RepeatPos;



struct Seaminfo{
    RepeatPos  scanePos;
    RepeatPos  framePos;
    int     seamIndex = 0;      //当前索引
    int     bind1	 ;      //焊缝关联索引1
    int     bind2	  ;     //焊缝关联索引2
    std::string   seamName ;	//焊缝名称
    int    orderIndex;          ///焊接次序
    bool         isScan = false;              //是否扫描
};

typedef  QList<Seaminfo>  RepeatSeamArray;

typedef struct Zitai_Info{

    double   spacing1 = 0	; //伸缩距离1
    double   distance1 = 0  ; //空间距离1
    double   spacing2 = 0	;
    double   distance2 = 0 ;
    double   pos_motor_start;		//地轨开始位置
    double   pos_motor_stop ;		//地轨停止位置

    double    angle1_motor_start ;	//变位机开始角度1
    double    angle1_motor_stop ;	//变位机停止角度1

    double    angle2_motor_start;	//变位机开始角度2
    double    angle2_motor_stop ;	//变位机停止角度2

    double    motor1speed	= 0;
    double    motor2speed	= 0;
    double    motor3speed	= 0;

    std::string  cadid =  "custom";
    Pos          cadMidPos = Pos(0,0,0,0,0,0);         //Cad Mid Point
    Pos          cadBasePos= Pos(0,0,0,0,0,0);        //Cad Base Point
    //bool         isScan = false;              //是否扫描
    RepeatSeamArray posArray;       /// 一组焊缝

}ST_ZT_INFO;

typedef  QList<Zitai_Info> RepeatData;

//inline std::string transString(double d,int len) {
//    int first = round(d * pow(10, len)) / pow(10, len);
//    int last = round(d * pow(10, len)) - (first * pow(10, len));
//    return std::to_string(first) + "." + std::to_string(abs(last));
//}


inline std::string pos2Str(Pos p){
    std::string s_pos = "";
//    s_pos+= transString(p.x,3);
//    s_pos+= ",";
//    s_pos+= transString(p.y,3);
//    s_pos+= ",";
//    s_pos+= transString(p.z,3);
//    s_pos+= ",";
//    s_pos+= transString(p.a,3);
//    s_pos+= ",";
//    s_pos+= transString(p.b,3);
//    s_pos+= ",";
//    s_pos+= transString(p.c,3);

    char buf[64];
     ::sprintf(buf,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",p.x,p.y,p.z,p.a,p.b,p.c);
    return std::string(buf);

}

//inline std::string pos2Str(Pos p){
//    std::string s_pos = "";
//    s_pos+= std::to_string(p.x);
//    s_pos+= ",";
//    s_pos+= std::to_string(p.y);
//    s_pos+= ",";
//    s_pos+= std::to_string(p.z);
//    s_pos+= ",";
//    s_pos+= std::to_string(p.a);
//    s_pos+= ",";
//    s_pos+= std::to_string(p.b);
//    s_pos+= ",";
//    s_pos+= std::to_string(p.c);
//    return s_pos;
//}

#endif // BASHTYPE_H
