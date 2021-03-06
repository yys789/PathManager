﻿#ifndef ERRORCODE_H
#define ERRORCODE_H
#include <string>
#include <map>
#include <cstdio>


enum ReturnStatus
{
    RESULT_OK = 0,
    //DB
    ERR_DB_OPEN_FAILED,
    ERR_DB_NO_DATA_FOUND,
    ERR_DB_DATA_ALREADY_EXISTS,
    ERR_DB_DATA_ERROR,
    ERR_EXEC_SQL_GET_RESULT_FAILED,


    //PLC-MODBUS
    ERR_MODBUS_CREATE_CONTEXT_FAILED,
    ERR_MODBUS_INVALID_SLAVEID,
    ERR_MODBUS_CONNECT_FAILED,
    ERR_MODBUS_WRITE_REGISTERS_FAILED,
    ERR_MODBUS_READ_REGISTERS_FAILED,
    ERR_MODBUS_PUSH_MESSAGE_FAILD,
    ERR_MODBUS_INVALID_DATA,
    ERR_MODBUS_FEEDBACK_RESULT_FAILED,
    ERR_MODBUS_FEEDBACK_ERRORCODE_FAILED,
    ERR_MODBUS_LASER_CONTROL_FAILED,
    ERR_MODBUS_AIRVALUE_CONTROL_FAILED,
    ERR_MODBUS_MOTOR_CONTROL_FAILED,
    ERR_MODBUS_MSGQUEUE_SEND_IS_EMPTY,
    ERR_MODBUS_CHECKCODE_ERROR,

    //UI-DB
    ERR_FUNC_DISPLAY_CADS_LIST_FAILED,
    ERR_FUNC_DISPLAY_SEAMINFO_FAILED,
    ERR_FUNC_UPDATE_PROGRESS_FAILED,

    //FUNC
    ERR_FUNC_SERVICE_PROCESS_RUN_ERROR,
    ERR_FUNC_CLEAN_TORCH_FAILED,
    ERR_FUNC_INITIAL_RUN_DATA_FAILED,
    ERR_FUNC_ADJUST_PLANE_FAILED,
    ERR_FUNC_RESET_FAILED,
    ERR_FUNC_SAVE_IMAGES_FAILED,
    ERR_FUNC_WELDING_FAILED,
    ERR_FUNC_CORRELATION_FAILED,
    ERR_FUNC_AUTO_SCAN_WELDING_FAILED,
    ERR_FUNC_GET_SEAMINFO_FAILED,
    ERR_FUNC_GET_COMINFO_FAILED,
    ERR_FUNC_GET_RELATED_SEAMS_FAILED,
    ERR_FUNC_GET_SEAMS_LIST_FAILED,
    ERR_FUNC_DEAL_IMAGES_FAILED,
    ERR_FUNC_DEAL_SEAM_FAILED,
    ERR_FUNC_EXCEPTION,

    //MODULE
    ERR_MODULE_ADD_IMAGES_FAILED,
    ERR_MODULE_GET_UV_FAILED,
    ERR_MODULE_CONFIRM_IMAGES_FAILED,

    //ROBOT
    ERR_ROBOT_MOVEING_EXCEPTION,
    ERR_ROBOT_RESET_FAILED,
    ERR_ROBOT_POS_NOT_SAFE,

    //MOTOR
    ERR_MOTOR_ANGLE_ABNORMAL,
    ERR_MOTOR_SYNC_FAILED,
    ERR_MOTOR_ROTATE_FAILED,

    //CAMERA
    ERR_CAMERA_TAKE_PICTURES_FAILED,

    //CHECK
    ERR_CHECK_EXCESSIVE_DEVIATION,
    ERR_CHECK_SENSOR_FAILED,
    ERR_CHECK_BOOL_SENSOR_FAILED,
    ERR_CHECK_TCP_FAILED,
    ERR_CHECK_SEAMINFO_FAILED,
    ERR_CHECK_BRIGHTNESS_FAILED,

    //DEVICE
    ERR_DEVICE_DISCONNECTED,
    ERR_DEVICE_MODBUS_DISCONNECTED,
    ERR_DEVICE_CAMERA_DISCONNECTED,


    //SYSINFO
    ERR_SYSINFO_DATA_ERROR,

    //NORMAL
    ERR_DATA_INVALID,

    ERR_END

};

const char* getErrString(int err_code);

#endif // ERRORCODE_H
