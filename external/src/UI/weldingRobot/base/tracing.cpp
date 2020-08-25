#include "tracing.h"
#include "ui_tracing.h"
#include "base/errorcode.h"
#include <QMessageBox>
#include <future>
#include "common/dbinterface.h"

Tracing::Tracing(QWidget *parent) :
    QWidget(parent),
    _mb_ptr(Singleton<mbControl>::get())
  ,_sqlite_ptr(Singleton<mySqliteApi>::get())
  ,_camera_ptr(Singleton<Camera>::get())
  ,m_bTaking{false}
  ,ui(new Ui::Tracing)
{
    ui->setupUi(this);
    Config cali("etc/calibration.info");
    ct[0]= cali.get<UV>("cameraCenterUV");  /// 传感器寻位中心点
    ct[1]= cali.get<UV>("cameraCenterUV1"); /// 传感器跟踪中心点
    initUI();
    weldEmpi();
}

Tracing::~Tracing()
{
    delete ui;
}

/////////////////上方代码为跟踪准备部分，下方代码是跟踪的具体逻辑实现//////////////////////
/// 跟踪开始前的参数、容器初期化
bool Tracing::initParams(string seamId,RelateInfo rif,bool wflag)
{
}

/// 图像 时间 位置采集至dataset
int Tracing::takePictures()
{
    ofstream ofs( logPath + "/path0.log") ;//打开记录机器人坐标位置文件
    try
    {
        if(!_mb_ptr || !_camera_ptr)
        {
            throw ERR_CAMERA_TAKE_PICTURES_FAILED;
        }
        ofs<<"照片序号	机器人反馈的pos	拍照时间 相机状态"<<endl;
        int count = 0;
        std::queue<SensorDataPack>().swap(dataSet);/// 清空图像位置容器
        while(m_bTaking)
        {
            if(waitMode)  /// 起头有一个TCP运动到焊接起点的过程，这个过程无需拍照
            {
                std::this_thread::sleep_for(10_ms);
                continue;
            }
            Mat mat;/// 当前图像
            _camera_ptr->takePicture().copyTo(mat);
            auto cur = _robot->getCurrPos( RobotTCP::UPLOAD );
            auto time_pic = getMicTime();
            SensorDataPack sdp = {mat,cur,time_pic,cameraMode};
            {
                std::lock_guard<std::mutex> lck(_mtx);
                dataSet.push(sdp);
            }
            std::this_thread::sleep_for(100_ms);
            char str[256];
            ::sprintf(str,"%d\t%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\ttime\t%.3f\t"
                      ,count,cur.x,cur.y,cur.z,cur.a,cur.b,cur.c,time_pic);
            ofs <<str<<cameraMode<<endl;
            count++;
        }
        char msg[256];
        sprintf(msg,"takePictures end：%.3f",getMicTime());
        ofs<<msg<<endl;
        ofs.close();//关闭坐标位置文件
        return RESULT_OK;
    }catch(ReturnStatus error_code)
    {
        throw;
    }
    catch(...)
    {
        cout<<__FUNCTION__<<endl;
        char msg[256];
        sprintf(msg,"takePictures err end：%.3f",getMicTime());
        ofs<<msg<<endl;
        m_bTaking = false;
        return ERR_FUNC_SERVICE_PROCESS_RUN_ERROR;
    }
}

void Tracing::posCheck(SensorDataPack mpa,double & x)
{
}

/// 只负责机器人运动控制,记录:
/// 拟合转换到base的pos及速度序列
/// 记录变位机坐标系下的原始pos序列
/// udp反馈的pos序列及时间
int Tracing::tracking()
{
}

/// 开始跟踪逻辑
void Tracing::workTracing()
{
    Config _sys_config(SYS);
    imageLevel = _sys_config.get<double>("imageLevel",1);
    maxGap =  _sys_config.get<double>("fit.gap",50);
    endKeelIndex = _sys_config.get<double>("fit.endKeelIndex",10);
    double movespeed = _sys_config.get<double>("robot.moveSpeed",300);
    double scanspeed = _sys_config.get<double>("robot.tracingScan",10);
    double motor1steps = _sys_config.get<double>("motor.motor1steps",10);
    double motor2steps = _sys_config.get<double>("motor.motor2steps",11100);
    double motor3steps = _sys_config.get<double>("motor.motor3steps",12100);
    double departLen = stringToNum(seamInfo["DEPARTLEN"]);
    angleInfo.angleSpeed1 = angleInfo.motor1speed/motor1steps;
    angleInfo.angleSpeed2 = 360*angleInfo.motor2speed/motor2steps;
    angleInfo.angleSpeed3 = 360*angleInfo.motor3speed/motor3steps;
    string posName;
    RobotAxle midpos;
    getMidPointById(stoi(curSeam),posName,midpos);
    p2p_len = 0;
    for(uint i=0;i<seamKeel.size()-1;i++)
    {
        point3dd k0 = {seamKeel[i].x,seamKeel[i].y,seamKeel[i].z};
        point3dd k1 = {seamKeel[i+1].x,seamKeel[i+1].y,seamKeel[i].z};
        p2p_len += k1.dist(k0); ///  按约定，龙骨都是等间距的
    }
    p2p_len /= (seamKeel.size()-1);
    _robot->setRotateFalse();  /// 通过机器人，关闭变位机转动IO口
    _sys_config.put("robot.speed", movespeed) ;
    _sys_config.sync();
    _robot->absoluteMove(midpos);  /// 机器人运动到安全位置
    if(abs(gMotorposs-angleInfo.motors.angle1) > 0.1
     ||abs(gMotorangle1s-angleInfo.motors.angle2) > 0.1
     ||abs(gMotorangle2s-angleInfo.motors.angle3) > 0.1)
    {
        if(gMotorposs > loc)
        {
            _mb_ptr->rotateNew(loc,30,gMotorangle1s,10,gMotorangle2s,10,1);
        }
        if(gMotorangle1s < safeA1)
        {
            _mb_ptr->rotateNew(gMotorposs,30,safeA1,10,gMotorangle2s,10,1);
        }
        _mb_ptr->rotateNew(angleInfo.motors.angle1,30,angleInfo.motors.angle2,10,angleInfo.motors.angle3,10,1);
    }
    _sqlite_ptr->selectSQLOrder({"SCANEPOS"},{"INDEX1","POS","ENABLED"},{"SEAMID",curSeam},{"INDEX1","ASC"});
    if(_sqlite_ptr->m_iRows <= 0)
    {
        QMessageBox::about(NULL,"提示","焊缝信息缺失,请检查DB!");
        _robot->absoluteMove(midpos);//回到安全位置
        return;
    }
    vector<string> pindex;
    vector<RobotPos> poslst;
    needPick = _sqlite_ptr->m_vResult[0]["ENABLED"] == "2"?true:false;
    for(auto& v:_sqlite_ptr->m_vResult)
    {
        RobotPos r = RobotPos::instance(v["POS"]);
        posInBase(r,tool,getPosition(angleInfo.motors));   /// 将抽象的扫描起点位置转换到base下
        poslst.push_back(r);
        pindex.push_back(v["INDEX1"]);
    }
    _mb_ptr->closeAirValve();   /// 关闭气阀，传感器变成寻位模式
    bool errEnd = false;
    switch(trackMode)  /// 焊缝的tracing属性1表示跟踪，3直接走龙骨
    {
    case 1:
    {
        /// 跟踪逻辑
        _mb_ptr->openLaser();  /// 打开激光
        if(poslst.size() < 2)
        {
            QMessageBox::about(NULL,"提示","初始图像采集路径缺失!");
            _robot->absoluteMove(midpos);//回到安全位置
            return;
        }
        RobotPos p0 = poslst[0];
        p0 = p0>CAMERA;  /// 转成相机中心点对准焊缝位置
        _robot->absoluteMove(p0,true,true); /// 扫描起点位置
        _sys_config.put("robot.speed", scanspeed) ;
        _sys_config.sync();
        RobotPos p1 = poslst[1];
        if(_gxA && needPick)
        {
            RobotPos mis;
            findRealStart(p0,mis);
            p0 = CAMERA>p0;
            p1 = p1<<mis;
            RobotPos p0_ = p0;
            posInTool(p0_,tool,getPosition(angleInfo.motors));   /// 将抽象的扫描起点位置转换到base下
            _sqlite_ptr->updateSQL({"SCANEPOS"},{"POS","'"+p0_.toStr()+"'","ENABLED","1"},{"SEAMID",curSeam,"INDEX1",pindex[0]});
            RobotPos p1_ = p1;
            posInTool(p1_,tool,getPosition(angleInfo.motors));   /// 将抽象的扫描起点位置转换到base下
            _sqlite_ptr->updateSQL({"SCANEPOS"},{"POS","'"+p1_.toStr()+"'","ENABLED","1"},{"SEAMID",curSeam,"INDEX1",pindex[1]});
            p0 = p0>CAMERA;
            _robot->absoluteMove(p0,true,true); /// 扫描起点位置
        }
        scaneEnd = p1>CAMERA;  /// 转成相机中心点对准焊缝位置
        m_bTaking = true;  /// 进入拍照状态
        std::future<int> f1 = std::async(std::launch::async,&Tracing::takePictures,this);           /// 拍照线程
        std::future<int> f12 = std::async(std::launch::async,&Tracing::imgProcess,this);            /// 图像处理线程
        std::future<int> f13 = std::async(std::launch::async,&Tracing::tracking,this); /// 机器人跟踪控制线程
        try
        {
            if(f1.get())
            {
                cout<<"图像采集异常结束!"<<endl;
            }
            if(f12.get())
            {
                cout<<"图像算法及轨迹拟合异常结束!"<<endl;
            }
            if(f13.get())
            {
                cout<<"机器人跟踪程序异常结束!"<<endl;
            }
            if(oriPath.size() > 0)
            {
                auto oriIter = oriPath.end()-1;
                while(oriIter>=oriPath.begin() && to_corra.size()<50)    /// 将原始点列最后50个点转到base坐标系下，放入to_corra容器，用于其他焊缝的拟合关联逻辑
                {
                    auto p = *oriIter;
                    posInBase(p,tool,getPosition(angleInfo.motore));
                    to_corra.push_back(p);
                    oriIter--;
                }
            }
        }
        catch(...)
        {
            errEnd = true;
        }
        _sys_config.put("robot.speed", 100);
        _sys_config.sync();
        RobotPos curpos = _robot->getCurrPos(RobotTCP::UPLOAD);
        retractMove(curpos,departLen);
        curpos.v = movespeed;
        _robot->absoluteMove(curpos);
        _robot->absoluteMove(midpos);//回到安全位置
//        if(abs(gMotorposs-angleInfo.motors.angle1) > 0.1
//         ||abs(gMotorangle1s-angleInfo.motors.angle2) > 0.1
//         ||abs(gMotorangle2s-angleInfo.motors.angle3) > 0.1)
//        {
//            if(gMotorposs > loc)
//            {
//                _mb_ptr->rotateNew(loc,30,gMotorangle1s,10,gMotorangle2s,10,1);
//            }
//            if(gMotorangle1s < safeA1)
//            {
//                _mb_ptr->rotateNew(gMotorposs,30,safeA1,10,gMotorangle2s,10,1);
//            }
//            _mb_ptr->rotateNew(angleInfo.motors.angle1,30,angleInfo.motors.angle2,10,angleInfo.motors.angle3,10,1);
//        }
        ofstream origin_ofs( logPath + "/path3.log") ;//变位机坐标系内的原始pos及角度
        origin_ofs<<"变位机坐标系内的原始pos"<<endl;   /// 打印变位机坐标系下的原始位置信息
        for(auto & p:oriPath)
        {
            origin_ofs<<p.toStr()<<endl;
        }
        origin_ofs.close();
        _mb_ptr->closeLaser();
    }
        break;
    case 3:/// 直接走龙骨
    {
        _mb_ptr->openLaser();  /// 打开激光
        auto s0 = seamKeel[0];
        posInBase(s0,tool,getPosition(angleInfo.motors));
        _sys_config.put("robot.speed", 100);
        _sys_config.sync();
        scaneEnd = s0;
        _robot->absoluteMove(s0>CAMERA);
        weldVec = seamKeel;
        std::future<int> f13 = std::async(std::launch::async,&Tracing::tracking,this); /// 机器人跟踪控制线程
        try
        {
            int result3 = f13.get();
            if(result3)
            {
                cout<<"机器人跟踪程序异常结束!"<<endl;
            }
        }
        catch(...)
        {
            errEnd = true;
        }
        _sys_config.put("robot.speed", 100);
        _sys_config.sync();
        _mb_ptr->closeLaser();
        RobotPos curpos = _robot->getCurrPos(RobotTCP::UPLOAD);
        retractMove(curpos,departLen);
        _robot->absoluteMove(curpos);//back 200
        _robot->absoluteMove(midpos);//回到安全位置
    }
        break;
    }
}

bool Tracing::findRealStart(RobotPos & p0,RobotPos & mis)
{
    RobotPos tar = p0;
    ofstream ofs("testLog/mo/findRealStart.txt");
    ofs<<getMicTime()<<endl;
    ofs<<"in pos :"<<p0.x<<","<<p0.y<<","<<p0.z<<endl;
    _robot->absoluteMove(tar);
    for(int t=0;t<=2;t++)
    {
        vector<RobotPos> p3d;
        for(int i=0;i<=4;i++)
        {
            for(int j=0;j<=4;j++)
            {
                RobotPos offset = RobotPos::instance();
                offset.x = -t*5;
                if(i%2)
                    offset.y = 16-j*8;
                else
                    offset.y = j*8-16;
                offset.z = 16-i*8;
                RobotPos ti = tar<<offset;
                _robot->absoluteMove(ti);
                Mat mat;/// 当前图像
                _camera_ptr->takePicture().copyTo(mat);
                _gxA->AddImg(mat);
                imwrite("testLog/mo/"+to_string(i)+"_"+to_string(j)+".png",mat);
                cv::Point2d p2d = _gxA->getuv();
                UV tz = {p2d.x,p2d.y};
                ofs<<ti.toStr()<<" "<<tz.u<<","<<tz.v;
                if(checkUV(tz,ct[0]))
                {
                    RobotPos pi;
                    UV2Point3D(ti,tz,pi,0);
                    p3d.push_back(pi);
                    ofs<<" "<<pi.toStr();
                }
                ofs<<endl;
            }
        }
        RobotPos realPos = p0;
        if(getCenter(p3d,realPos))
        {
            tar = realPos>CAMERA;
            ofs<<"real pos :"<<tar.x<<","<<tar.y<<","<<tar.z<<endl;
            mis = (!p0)<<tar;
            p0 = tar;
            break;
        }
    }
    ofs.close();
    return true;
}
