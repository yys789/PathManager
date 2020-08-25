#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H


#include "baseTypes.h"
#include "Frame.h"
#include "DecodeControl.h"

class InputControl
{
public:
    InputControl();

    static std::string CreatSerial(RepeatData &data);

    static RepeatData TestCreat();

};

#endif // INPUTCONTROL_H
