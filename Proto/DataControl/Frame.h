#ifndef FRAME_H
#define FRAME_H

#include <QtGlobal>

#ifdef Q_OS_WIN

#elif defined (Q_OS_LINUX)

#endif

#include "Proto/Google/Frame.pb.h"

#include <QDebug>
#include "baseTypes.h"




// 数据基类，定义了一些插入方式
class Frame
{
public:
    Frame();
    ~Frame();
    static std::string N_double(double a);
    static double F_double(double a);

    static std::string proto2PosStr(frame::Pos *p);


};

#endif // FRAME_H
