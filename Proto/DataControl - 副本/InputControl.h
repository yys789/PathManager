#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "bashType.h"
#include "Frame.h"
#include "DecodeControl.h"

class InputControl
{
public:
    InputControl();

    static RepeatData& TestCreat();

    static std::string CreatSerial(RepeatData &data);


};

#endif // INPUTCONTROL_H
