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

std::string Frame::N_double(double a)
{
    char buf[20];
    ::sprintf(buf,"%.3f",a);
    string f(buf);
    a = atof(buf);
    return f;
}

double Frame::F_double(double a)
{
    char buf[20];
    ::sprintf(buf,"%.3f",a);
    string f(buf);
    return atof(buf);
}


std::string Frame::proto2PosStr(frame::Pos *p)
{
    std::string s_pos = "";

    char buf[64];
     ::sprintf(buf,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",p->x(),p->y(),p->z(),p->a(),p->b(),p->c());
    return std::string(buf);
    return s_pos;
}



