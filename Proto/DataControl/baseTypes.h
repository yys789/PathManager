#ifndef BASETYPES_H
#define BASETYPES_H


#include <iostream>
#include <QList>
#include <fstream>
#include <strings.h>
#include <math.h>
#include <sstream>




using namespace std;
typedef  int int32;

// 基础数据结构 Pos单元
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
    double   x = 0;
    double   y  = 0;
    double   z = 0;
    double   a  = 0;
    double   b = 0;
    double   c = 0;


    Pos instance(string str)
    {
        Pos p = {0,0,0,0,0,0};
        if(str == "")
            return p;
        char split[3] = {',',' ','\t'};
        std::vector<double> ds;
        for(int i=0;i<3;i++)
        {
            ds.resize(0);
            ds.clear();
            stringstream ss(str);
            while(ss.good())
            {
                string substr;
                getline( ss, substr,split[i]);
                stringstream sd(substr);
                double di;
                sd>>di;
                ds.push_back(di);
            }
            if(ds.size() >= 6)
                break;
        }
        if(ds.size() < 6)
            return p;
        p.x = ds[0];
        p.y = ds[1];
        p.z = ds[2];
        p.a = ds[3];
        p.b = ds[4];
        p.c = ds[5];
        return p;
    }

    std::string toStr(char sep=','){
        char buf[64];
        ::memset(buf,0,64);
        ::sprintf(buf,"%.3f%c%.3f%c%.3f%c%.3f%c%.3f%c%.3f",x,sep,y,sep,z,sep,a,sep,b,sep,c);
        return std::string(buf);
    }
};


// 一个龙骨数据
struct AFramePos{
    Pos pos = Pos(-1,-1,-1,-1,-1,-1);             // 工件坐标

    double angle1 = 0.0;
    double angle2 = 0.0;
    double angle3 = 0.0;

    /* 可能不需要填写的数据 */
    int index1 = 0;          //区分顺序
    int weld   = 1;      // 焊接方式（默认为1）
    int height = 4;      // 焊缝的高度(自适应用)
    int enabled = 1;         // 是否需要焊接
};

// 一个扫描数据
struct AScanePos{
    Pos pos;             // 工件坐标  //

    /* 可能不需要填写的数据 */
    int index1      = -1;          //区分顺序
    int camera   = 0;      // 焊接方式（默认为1）
    int enabled = 1;         // 是否需要焊接

    double angle1 = 0.0;
    double angle2 = 0.0;
    double angle3 = 0.0;
};

// 一个CAD信息
struct ACad{
    string    cadid       = "custom";
    Pos       cadBasePos  = Pos(0,0,0,0,0,0);

    /* 可能不需要填写的数据 */
    int32     cadMidPoint   = 0;
    double    safeAngle1  = -45;		// 安全角度
    int32     safeLoc     = 450;    // 安全地柜位置
};

// 焊缝关联信息
struct ASeamRelate{
    double    spacing1	=   0; 	//伸缩距离
    double    distance1  =  0; 	//空间距离
    double    spacing2	=   0;
    double    distance2  =  0;	//
    int32     bind1	  =  0 ; 	//焊缝关联索引1
    int32     bind2	  =  0 ; 	//焊缝关联索引2

    int32     seFlag  = 0;      //默认0
};


// 焊接信息-------暂时先不填
struct SeamWeldInfo{
    int      weldorder	= 0;
    Pos    	 offset		= Pos(0,0,0,0,0,0);
    Pos      baseoffset	= Pos(0,0,0,0,0,0);
    int32    enabled = 1;   // 默认1
};

// 焊接参数-------暂时先不填
struct WeldParament{
    int32    s_w	= 30;
    int32    i_w	= 160;
    int32    v_w	= 21;
    int32    t_end	= 0;
    int32    i_end	= 0;
    int32    v_end	= 0;
    int32    range	= 0;
    int32    axis_x	= 6;
    int32    axis_y	= 6;
};

// 变位机及地轨参数
struct MachineGrond{
    double    pos_motor_start 	= 0;		//地轨开始位置
    double    pos_motor_stop 	= 0;		//地轨开始位置
    double    angle1_motor_start 	= 0;	//变位机开始角度
    double    angle1_motor_stop	= 0;	//变位机停止角度
    double    angle2_motor_start	= 0;	//变位机开始角度
    double    angle2_motor_stop 	= 0;	//变位机停止角度
    double    motor1speed		= 0;
    double    motor2speed		= 0;
    double    motor3speed		= 0;
};



// 焊缝信息及参数
struct SeamInfo{
    QList<AFramePos>    framePos ;		// 龙骨数据点
    QList<AScanePos>    scanePos ;		// 扫描数据点

    ASeamRelate   relate	;		// 关联信息
    SeamWeldInfo weldinfo;		// 焊接参数
    WeldParament weldPara;
    string    seamName = "DEFAULT";			// 焊缝名称
    int32     seamID = -1 ;         // 焊缝索引

    int32     orderIndex = -1;       // 焊缝次序
    string    cadid   = "";         // 当前的Cadid
    MachineGrond  machine;      // 变位机及地轨数据

    /* 可能不需要填写的数据 */
    int32       main = 1;       // 默认为1
    int32       rank = 0;       // 直线为0，曲线为1
    string      seamType = "V";       //焊缝类型---------V SE  J
    double       DEPARTLEN = 100;       //回撤距离?默认100
    int32         midPoint = 0;
    int32       MIDDLESTART = 0;
    string      option  = "00000000";
    int32       autoFit = 0;        //自适应
    int32       tracing = 0;        //0.寻位   1.跟踪  2.半寻位  3.龙骨

    double       width   = 0;
    double       height  = 0;

    int32       CORNERLEN   =  0;
    int32       CORNERANGLE =  0;

    int32       enabled = 1;
};


struct RepeatData{
    QList<SeamInfo> seams;  // 许多组焊缝
    QList<ACad>  cad;
    string       date ;			// 定义创建时间
};

#endif // BASETYPES_H
