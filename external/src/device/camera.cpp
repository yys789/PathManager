#include "device/camera.h"
#include <iostream>
#include <thread>
//#include <QDebug>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include "base/data_types.h"
#include "walgo/histimage.h"
#include "base/config.h"

using namespace std;
using namespace chrono;
using namespace cv;
using namespace walgo;

#define MAX_BUF_SIZE (720*540*3)
#define _LOG_ cout<<"["<<__FILE__<<"]["<<__LINE__<<"]["<<__FUNCTION__<<"]"<<endl;


Camera::Camera():
    _status{DISCONNECTED},
    m_hDevHandle{nullptr}
{
    connectDevice();
}

Camera::~Camera()
{
    disconnectDevice();
}

void Camera::connectDevice()
{
    _LOG_;
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    try
    {
        if( DISCONNECTED == _status)
        {
            unsigned nIndex = 0;
            int nRet = MV_OK;
            MV_CC_DEVICE_INFO_LIST m_stDevList;
            memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
            // ch:枚举子网内所有设备
            nRet = EnumDevices(MV_USB_DEVICE, &m_stDevList);
            if (MV_OK != nRet)
            {
                cout<<"Enum Devices ,nRet = "<<nRet<<endl;
                return;
            }

            cout<<"camera device count : "<< m_stDevList.nDeviceNum <<endl;

            if(!IsDeviceAccessible(m_stDevList.pDeviceInfo[nIndex], MV_ACCESS_Control))
            {
                cout<<"Devices Accessible false ."<<endl;
                return;
            }
            nRet = Open(m_stDevList.pDeviceInfo[nIndex]);
            if (MV_OK != nRet)
            {
                cout<<"open camera failed, nRet = "<<nRet<<endl;
                return;
            }
            m_ptrFrameBuf = nullptr;
            m_ptrFrameBuf = (unsigned char*)malloc(MAX_BUF_SIZE);
            memset(m_ptrFrameBuf,0,sizeof(MAX_BUF_SIZE));
            nRet = SetEnumValue("PixelFormat",PixelType_Gvsp_RGB8_Packed);
            if (MV_OK != nRet)
            {
                cout<<"set PixelFormat failed, nRet = "<<nRet<<endl;
                return;
            }
            nRet = SetEnumValue("TriggerMode",MV_TRIGGER_MODE_OFF);
            if (MV_OK != nRet)
            {
                cout<<"set trigger mode, nRet = "<<nRet<<endl;
                return;
            }
            nRet = SetEnumValue("ExposureAuto",MV_EXPOSURE_AUTO_MODE_OFF);
            if (MV_OK != nRet)
            {
                cout<<"set ExposureAuto failed, nRet = "<<nRet<<endl;
                return;
            }
            //设置自动白平衡模式
            unsigned int nValue = MV_BALANCEWHITE_AUTO_OFF; //一次白平衡模式
            nRet = MV_CC_SetBalanceWhiteAuto(m_hDevHandle, nValue);
            if (MV_OK != nRet)
            {
                cout<<"set BalanceWhiteAuto OFF failed, nRet = "<<nRet<<endl;
                return;
            }

            nRet = MV_CC_SetImageNodeNum(m_hDevHandle,3);
            if (MV_OK != nRet)
            {
                cout<<"set image node num, nRet = "<<nRet<<endl;
                return;
            }

            Config _sys_config("etc/sys.info");
            setExposure(_sys_config.get<int>("camera.exposure"));
            nRet = StartGrabbing();
            if (MV_OK != nRet)
            {
                cout<<"StartGrabbing failed, nRet = "<<nRet<<endl;
                return;
            }
            _status = CONNECTED;
        }
    }
    catch(...)
    {
        cout<<"camera connect faield" <<endl;
    }

}

void Camera::disconnectDevice()
{
    _LOG_;
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    if(CONNECTED == _status)
    {
        StopGrabbing();
    }
    if(m_ptrFrameBuf)
    {
        free(m_ptrFrameBuf);
        delete m_ptrFrameBuf;
    }
    Close();
    _status = DISCONNECTED;
}


cv::Mat Camera::takePicture()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx);
    uint nRet = MV_OK;
    cv::Mat matRet;
    if( DISCONNECTED == _status || nullptr == m_hDevHandle)
    {
        cout<<"camera handle nullptr"<<endl;
        return matRet;
    }
    try
    {
        memset(m_ptrFrameBuf,0,sizeof(MAX_BUF_SIZE));

        MV_FRAME_OUT_INFO_EX pFrameInfo = {0};
        memset(&pFrameInfo,0,sizeof(MV_FRAME_OUT_INFO_EX));

        nRet = GetOneFrameTimeout(m_ptrFrameBuf,MAX_BUF_SIZE,&pFrameInfo,1000);
        if(0x80000007 == nRet || 0x80000006 == nRet)
        {
            cout<<"img err:"<<nRet<<endl;
        }
        if(MV_OK != nRet && 0x80000007 != nRet && 0x80000006 != nRet)
        //if(MV_OK != nRet)
        {
            cout<<"Get One Frame Timeout, nRet = "<<nRet<<endl;
            _status = ERR;
            throw "get oneFrame fialed!";
        }
        RGB2BGR(m_ptrFrameBuf, pFrameInfo.nWidth, pFrameInfo.nHeight);
        matRet = cv::Mat(pFrameInfo.nHeight, pFrameInfo.nWidth, CV_8UC3, m_ptrFrameBuf);
        if(matRet.data == NULL)
        {
            cout<<"get no image"<<endl;
            return _mat;
        }
        {
            std::lock_guard<std::recursive_mutex> lck(_mtxMat);
            matRet.copyTo(_mat);
        }
        _timePicture = steady_clock::now() ;
        memset(m_ptrFrameBuf,0,sizeof(MAX_BUF_SIZE));

    }catch(const char * errInfo)
    {
        resetCamera();
    }
    return _mat;
}

void Camera::resetCamera()
{
    _LOG_;
    disconnectDevice();
    connectDevice();
}

int Camera::setGain(float fValue)
{
    _LOG_;
    return MV_CC_SetGain(m_hDevHandle,fValue);
}

int Camera::setExposure(float fValue)
{
    _LOG_;
    int nRet=MV_CC_SetEnumValue(m_hDevHandle,"ExposureAuto",MV_EXPOSURE_AUTO_MODE_OFF);
    if(MV_OK != nRet)
    {
        return nRet;
    }
    return MV_CC_SetFloatValue(m_hDevHandle,"ExposureTime",fValue);
}

void Camera::setRoi(int witdh,int height,int offsetX,int offsetY)
{
    _LOG_;
    if(witdh > 0)
    {
        MV_CC_SetIntValue(m_hDevHandle,"Width",witdh);
    }
    if(height > 0)
    {
        MV_CC_SetIntValue(m_hDevHandle,"Height",height);
    }
    if(offsetX > 0)
    {
        MV_CC_SetIntValue(m_hDevHandle,"OffsetX",offsetX);
    }
    if(offsetY > 0)
    {
        MV_CC_SetIntValue(m_hDevHandle,"OffsetY",offsetY);
    }
}

int Camera::EnumDevices(unsigned int nTLayerType, MV_CC_DEVICE_INFO_LIST* pstDevList)
{
    return MV_CC_EnumDevices(nTLayerType, pstDevList);
}

bool Camera::IsDeviceAccessible(MV_CC_DEVICE_INFO* pstDevInfo, unsigned int nAccessMode)
{
    return MV_CC_IsDeviceAccessible(pstDevInfo, nAccessMode);
}

int Camera::Open(MV_CC_DEVICE_INFO* pstDeviceInfo)
{
    if (MV_NULL == pstDeviceInfo)
    {
        return MV_E_PARAMETER;
    }

    if (m_hDevHandle)
    {
        return MV_E_CALLORDER;
    }

    int nRet  = MV_CC_CreateHandle(&m_hDevHandle, pstDeviceInfo);
    if (MV_OK != nRet)
    {
        return nRet;
    }

    nRet = MV_CC_OpenDevice(m_hDevHandle);
    if (MV_OK != nRet)
    {
        MV_CC_DestroyHandle(m_hDevHandle);
        m_hDevHandle = MV_NULL;
    }

    return nRet;
}

int Camera::Close()
{
    if (MV_NULL == m_hDevHandle)
    {
        return MV_E_HANDLE;
    }

    MV_CC_CloseDevice(m_hDevHandle);

    int nRet = MV_CC_DestroyHandle(m_hDevHandle);
    m_hDevHandle = MV_NULL;

    return nRet;
}

bool Camera::IsDeviceConnected()
{
    return MV_CC_IsDeviceConnected(m_hDevHandle);
}

int Camera::StartGrabbing()
{
    return MV_CC_StartGrabbing(m_hDevHandle);
}

int Camera::StopGrabbing()
{
    return MV_CC_StopGrabbing(m_hDevHandle);
}

int Camera::GetOneFrameTimeout(unsigned char* pData, unsigned int nDataSize, MV_FRAME_OUT_INFO_EX* pFrameInfo, int nMsec)
{
    return MV_CC_GetOneFrameTimeout(m_hDevHandle, pData, nDataSize, pFrameInfo,nMsec);
}

int Camera::SetImageNodeNum(unsigned int nNum)
{
    return MV_CC_SetImageNodeNum(m_hDevHandle, nNum);
}

int Camera::GetDeviceInfo(MV_CC_DEVICE_INFO* pstDevInfo)
{
    return MV_CC_GetDeviceInfo(m_hDevHandle, pstDevInfo);
}

int Camera::GetIntValue(IN const char* strKey, OUT MVCC_INTVALUE_EX *pIntValue)
{
    return MV_CC_GetIntValueEx(m_hDevHandle, strKey, pIntValue);
}

int Camera::SetIntValue(IN const char* strKey, IN int64_t nValue)
{
    return MV_CC_SetIntValueEx(m_hDevHandle, strKey, nValue);
}

int Camera::GetEnumValue(IN const char* strKey, OUT MVCC_ENUMVALUE *pEnumValue)
{
    return MV_CC_GetEnumValue(m_hDevHandle, strKey, pEnumValue);
}

int Camera::SetEnumValue(IN const char* strKey, IN unsigned int nValue)
{
    return MV_CC_SetEnumValue(m_hDevHandle, strKey, nValue);
}

int Camera::SetEnumValueByString(IN const char* strKey, IN const char* sValue)
{
    return MV_CC_SetEnumValueByString(m_hDevHandle, strKey, sValue);
}

int Camera::GetFloatValue(IN const char* strKey, OUT MVCC_FLOATVALUE *pFloatValue)
{
    return MV_CC_GetFloatValue(m_hDevHandle, strKey, pFloatValue);
}

int Camera::SetFloatValue(IN const char* strKey, IN float fValue)
{
    return MV_CC_SetFloatValue(m_hDevHandle, strKey, fValue);
}

int Camera::GetBoolValue(IN const char* strKey, OUT bool *pbValue)
{
    return MV_CC_GetBoolValue(m_hDevHandle, strKey, pbValue);
}

int Camera::SetBoolValue(IN const char* strKey, IN bool bValue)
{
    return MV_CC_SetBoolValue(m_hDevHandle, strKey, bValue);
}

int Camera::GetStringValue(IN const char* strKey, MVCC_STRINGVALUE *pStringValue)
{
    return MV_CC_GetStringValue(m_hDevHandle, strKey, pStringValue);
}

int Camera::SetStringValue(IN const char* strKey, IN const char* strValue)
{
    return MV_CC_SetStringValue(m_hDevHandle, strKey, strValue);
}

int RGB2BGR( unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight )
{
    if ( NULL == pRgbData )
    {
        return MV_E_PARAMETER;
    }

    for (unsigned j = 0; j < nHeight; j++)
    {
        for (unsigned int i = 0; i < nWidth; i++)
        {
            unsigned char red = pRgbData[j*(nWidth*3)+i*3];
            pRgbData[j*(nWidth*3)+i*3] = pRgbData[j*(nWidth*3)+i*3+2];
            pRgbData[j*(nWidth*3)+i*3+2] = red;
        }
    }
    return MV_OK;
}

const cv::Mat Camera::getImg() const
{
    Millsecond timeNow = steady_clock::now() ;
    // cout<<"(timeNow - _timePicture)::"<<(timeNow - _timePicture)<<endl;
    if  ((timeNow - _timePicture)  > MillDiff(20)) {
        const_cast<Camera*>(this)->takePicture() ;
    }
    return _mat ;
}

