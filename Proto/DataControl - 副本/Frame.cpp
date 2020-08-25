#include "Frame.h"
#include <fstream>
#include <iostream>
Frame::Frame()
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;



}

Frame::~Frame()
{
    google::protobuf::ShutdownProtobufLibrary();
}


Pos Frame::creatPos(double x, double y, double z, double a, double b, double c)
{
    return  Pos(x,y,z,a,b,c);
}



std::string Frame::proto2PosStr(frame::Pos *p)
{
    std::string s_pos = "";

    char buf[64];
     ::sprintf(buf,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",p->x(),p->y(),p->z(),p->a(),p->b(),p->c());
    return std::string(buf);
//    s_pos+= transString(p->x(),3);
//    s_pos+= ",";
//    s_pos+= transString(p->y(),3);
//    s_pos+= ",";
//    s_pos+= transString(p->z(),3);
//    s_pos+= ",";
//    s_pos+= transString(p->a(),3);
//    s_pos+= ",";
//    s_pos+= transString(p->b(),3);
//    s_pos+= ",";
//    s_pos+= transString(p->c(),3);

//    s_pos+= std::to_string(p->x());
//    s_pos+= ",";
//    s_pos+= std::to_string(p->y());
//    s_pos+= ",";
//    s_pos+= std::to_string(p->z());
//    s_pos+= ",";
//    s_pos+= std::to_string(p->a());
//    s_pos+= ",";
//    s_pos+= std::to_string(p->b());
//    s_pos+= ",";
//    s_pos+= std::to_string(p->c());
    return s_pos;
}

/// 增加龙骨数据组
//void Frame::appendZitai(frame::FrameChildren* obj,frame::PosArray *list)
//{
//    auto posList = obj->mutable_poslist();
//    /// 插入Map键值对的方法，注释的也是一种方法
//    //(*posList)[posList->size()] = *list;
//    posList->operator[](posList->size()) = *list;
//}

///// 创建新增的龙骨对象，并赋予值
//frame::FrameChildren* Frame::creatZitai(double x, double y, double z, double angle1, double angle2)
//{
//    auto obj =new  frame::FrameChildren ;
//    obj->set_x(x);
//    obj->set_y(y);
//    obj->set_z(z);
//    obj->set_angle1(angle1);
//    obj->set_angle2(angle2);
//    return obj;
//}



