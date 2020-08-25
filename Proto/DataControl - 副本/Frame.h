#ifndef FRAME_H
#define FRAME_H

#include <QtGlobal>

#ifdef Q_OS_WIN
    #include "Proto/Google/win/Frame.pb.h"
#elif defined (Q_OS_LINUX)
    #include "Proto/Google/linux/Frame.pb.h"
#endif

#include <QDebug>
#include "bashType.h"




// 数据基类，定义了一些插入方式
class Frame
{
public:
    Frame();
    ~Frame();

    static Pos  creatPos(double x,double y,double z,double a,double b,double c);
    static std::string proto2PosStr(frame::Pos *p);

//    /// 插入子路径集
//    static void appendZitai(frame::FrameChildren* obj,frame::PosArray *list);

//    /// 创建单个龙骨数据
//    static frame::FrameChildren* creatZitai(double x,double y,double z,double angle1,double angle2);


};

#endif // FRAME_H
