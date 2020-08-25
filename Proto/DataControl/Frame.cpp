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




std::string Frame::proto2PosStr(frame::Pos *p)
{
    std::string s_pos = "";

    char buf[64];
     ::sprintf(buf,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",p->x(),p->y(),p->z(),p->a(),p->b(),p->c());
    return std::string(buf);
    return s_pos;
}



