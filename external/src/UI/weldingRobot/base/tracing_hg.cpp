#include "tracing.h"
#include "ui_tracing.h"
#include "base/errorcode.h"
#include <QMessageBox>
#include <future>
#include "fitCorrelation.h"
#include "getuvonshell/self_adaption.h"
#include "common/dbinterface.h"

void Tracing::initUI()
{
    _sqlite_ptr->selectSQL({"CAD"});
    if(_sqlite_ptr->m_iRows == 0)
        throw ERR_DB_NO_DATA_FOUND;
    auto vec = _sqlite_ptr->m_vResult;
    for(auto& v:vec)
        ui->cbxCAD->addItem(QString::fromStdString(v["CADID"]));
    ui->dig->setText(QString::number(0));
}

void Tracing::on_btnPosition3_clicked()
{
    CircleModel cmdel;
    vector<RobotPos> data;
    ifstream ifs;
    ifs.open("circleModel3.txt");
    string s;
    while(getline(ifs,s))
    {
        RobotPos r;
        if(str2Robot(s,r))
        {
            data.push_back(r);
        }
    }
    cmdel.build(data);
    cout<<"cpos:"<<cmdel.printCPos()<<"R:"<<cmdel.getR()<<"rms:"<<cmdel.getRMS()<<endl;
}

void Tracing::on_fitHGBase_clicked()
{
    vector<RobotPos> points;
    ifstream ifs;
    ifs.open("circleModel6.txt");
    string s;
    while(getline(ifs,s))
    {
        RobotPos r;
        if(str2Robot(s,r))
        {
            points.push_back(r);
        }
    }
    ifs.close();
    if(points.size() < 2)
        return;
    RobotPos pos0 = points[0];
    RobotPos pos_y = points[1];
    point3dd axis_y = {pos_y.x-pos0.x,pos_y.y-pos0.y,pos_y.z-pos0.z};
    point3dd axis_z = {0,0,1};
    point3dd axis_x = axis_y.cross(axis_z);
    axis_x.normalize();
    axis_y = axis_z.cross(axis_x);
    Matrix4d mHG;
    mHG<<axis_x._x,axis_y._x,axis_z._x,pos0.x,
            axis_x._y,axis_y._y,axis_z._y,pos0.y,
            axis_x._z,axis_y._z,axis_z._z,pos0.z,
            0,        0,        0,     1;
    RobotPos cpos;
    Matrix2pos(mHG,cpos,zyz);
    cout<<"cpos:"<<cpos.toStr()<<endl;
    RobotPos tool0 = RobotPos::instance();
    cout<<"in position:"<<cpos.toStr()<<endl;
}

/// 拟合基本点坐标系
void Tracing::on_fitBase_clicked()
{
    vector<RobotPos> circle4;
    vector<RobotPos> face5;
    ifstream ifs;
    ifs.open("circleModel4.txt");
    string s;
    while(getline(ifs,s))
    {
        RobotPos r;
        if(str2Robot(s,r))
        {
            circle4.push_back(r);
            face5.push_back(r);
        }
    }
    ifs.close();
    CircleModel cmdel;
    cmdel.build(circle4);
    cout<<"cpos circle:"<<cmdel.printCPos()<<"R:"<<cmdel.getR()<<"rms:"<<cmdel.getRMS()<<endl;
    RobotPos cpos = RobotPos::instance(cmdel.printCPos());
    ifstream ifs2;
    ifs2.open("circleModel5.txt");
    while(getline(ifs2,s))
    {
        RobotPos r;
        if(str2Robot(s,r))
        {
            face5.push_back(r);
        }
    }
    ifs2.close();
    int fConts = face5.size();
    Vector3d abc;
    points2face(face5,abc);
    point3dd axis_z = {abc(0,0),abc(1,0),abc(2,0)};
    axis_z.normalize();
    point3dd axis_x = {face5[fConts-1].x-cpos.x,face5[fConts-1].y-cpos.y,face5[fConts-1].z-cpos.z};
    point3dd axis_y = axis_z.cross(axis_x);
    axis_y.normalize();
    axis_x = axis_y.cross(axis_z);
    axis_x.normalize();
    Matrix4d oat;
    oat<<axis_x._x,axis_y._x,axis_z._x,cpos.x,
            axis_x._y,axis_y._y,axis_z._y,cpos.y,
            axis_x._z,axis_y._z,axis_z._z,cpos.z,
            0,        0,        0,     1;
    Matrix2pos(oat,cpos,zyz);
    cout<<"cpos:"<<cpos.toStr()<<endl;
}

void Tracing::on_home_2_clicked()
{
    _robot->setRotateFalse();
}

///变位机回到初始跟踪状态
void Tracing::on_home_clicked()
{
    std::thread([this]()mutable{
        double t1 = getMicTime();
        double digs = ui->dig->text().toDouble();
        _robot->setRotateFalse();
        _mb_ptr->rotateMotor(ROTATINGDISK,digs,200,2);
        this_thread::sleep_for(std::chrono::microseconds(400));
        _robot->setRotateTrue();
        this_thread::sleep_for(std::chrono::milliseconds(200));
        imwrite("picture/"+to_string(getMicTime())+".png",_camera_ptr->takePicture());
        double t2 = getMicTime();
        cout<<"rotate time:"<<t2-t1<<endl;
        _mb_ptr->openLaser();
    }).detach();
}

void Tracing::on_home_3_clicked()
{
    static bool kg = false;
    kg = !kg;
    std::thread([this]
    {
        int count = 0;
        while(kg)
        {
            cout<<count;
            _robot->setRotateTrue();
            cout<<"\t信号打开";
            this_thread::sleep_for(std::chrono::seconds(3));
            _robot->setRotateFalse();
            cout<<"\t信号关闭"<<endl;
            this_thread::sleep_for(std::chrono::seconds(3));
            count++;
        }
    }).detach() ;
}

void Tracing::on_rotateTrack_clicked()
{
    uint tc = std::thread::hardware_concurrency();
    cout<<"hardware_concurrency:"<<tc<<endl;
    auto trd = std::thread([this]{
        curSeam = seamMap[ui->cbxSEAM->currentText().toStdString()];
        vector<RobotPos> empty1;
        vector<RobotPos> empty2;
        ifstream ifs;
        ifs.open("corrationTest.txt");
        string line;
        while(getline(ifs,line))
        {
            RobotPos pos;
            if(str2Robot(line,pos))
            {
                empty2.push_back(pos);
            }
        }
        ifs.close();
        RelateInfo rif;
        if(initParams(curSeam,rif,ui->ckxWeld->isChecked()))
            workTracing();
    });
    trd.detach();
}

void Tracing::on_signal51_clicked()
{
    _robot->setRotateTrue();
}

/// 验证龙骨数据，机器人空走
void Tracing::on_verifiKeel_clicked()
{
    std::thread([this]()mutable{
        while(1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            _sqlite_ptr->selectSQL("COMPARAMS");
            auto vec = _sqlite_ptr->m_vResult;
            double angle = 0;
            for(int i=0;i<vec.size();i++)
            {
                auto kv = vec[i];
                if(kv["COMNAME"] == "CURMOTORPOS")
                {
                    angle = stringToNum(kv["COMVALUE"]);
                    break;
                }
            }
            if(abs(angle) < 0.01)
                break;
        }
        Config _sys_config(SYS);
        _sys_config.put("robot.speed",100) ;
        _sys_config.sync();
        map<string,RobotAxle> savePosMap;
        _sqlite_ptr->selectSQL({"SAFEPOS"});
        for(auto p : _sqlite_ptr->m_vResult)
        {
            RobotAxle midpos;
            sPosTodPos(p["POS"],midpos);
            savePosMap[p["POSID"]] = midpos;
        }
        curCAD = ui->cbxCAD->currentText().toStdString();
        string curSeamNAME = ui->cbxSEAM->currentText().toStdString();
        _sqlite_ptr->selectSQL2({"CAD"},{"BASEPOINT","SAFELOC","SAFEANGLE1"},{"CADID","'"+curCAD+"'"});
        if(_sqlite_ptr->m_vResult.size() == 0)
            return;
        double loc = stringToNum(_sqlite_ptr->m_vResult[0]["SAFELOC"]);
        double safeA1 = stringToNum(_sqlite_ptr->m_vResult[0]["SAFEANGLE1"]);
        tool = RobotPos::instance(_sqlite_ptr->m_vResult[0]["BASEPOINT"]);
        _sqlite_ptr->selectSQLOrder({"SEAMINFO"},
        {"SEAMID","MIDPOINT","DEPARTLEN",
         "MOTORPOSS","MOTORANGLE1S","MOTORANGLE2S",
         "MOTORPOSE","MOTORANGLE1E","MOTORANGLE2E",
         "MOTOR1SPEED","MOTOR2SPEED","MOTOR3SPEED","TRACING"},
        {"CADID","'"+curCAD+"'","SEAMNAME","'"+curSeamNAME+"'","ENABLED","1"},{"ORDERID","ASC"});
        auto vecseam = _sqlite_ptr->m_vResult;
        for(auto r : vecseam)
        {
            double departLen = stringToNum(r["DEPARTLEN"]);
            double m1s = stringToNum(r["MOTORPOSS"]);
            double m2s = stringToNum(r["MOTORANGLE1S"]);
            double m3s = stringToNum(r["MOTORANGLE2S"]);
            double m1e = stringToNum(r["MOTORPOSE"]);
            double m2e = stringToNum(r["MOTORANGLE1E"]);
            double m3e = stringToNum(r["MOTORANGLE2E"]);
            double v1 = stringToNum(r["MOTOR1SPEED"]);
            double v2 = stringToNum(r["MOTOR2SPEED"]);
            double v3 = stringToNum(r["MOTOR3SPEED"]);
            RobotAxle safe = savePosMap[r["MIDPOINT"]];
            int trackMode = atoi(r["TRACING"].c_str());
            _robot->absoluteMove(safe);
            _sqlite_ptr->selectSQLOrder({"FRAMEPOS"},{"POS"},{"SEAMID","'"+r["SEAMID"]+"'","ENABLED","1"},{"INDEX1","ASC"});
            vector<RobotPos> posList;
            int ks = _sqlite_ptr->m_vResult.size();
            int index = 0;
            for(auto s : _sqlite_ptr->m_vResult)
            {
                RobotPos keel = RobotPos::instance(s["POS"]);
                MOTORDEGRE degree = {m1s+index*(m1e-m1s)/ks,m2s+index*(m2e-m2s)/ks,m3s+index*(m3e-m3s)/ks};
                posInBase(keel,tool,getPosition(degree));
                keel.v = 10;
                if(trackMode != 1)
                {
                    keel = keel>CAMERA;
                }
                keel.v = 50;
                posList.push_back(keel);
                index++;
            }
            if(posList.size() < 3)
                continue;

            _sqlite_ptr->selectSQL("COMPARAMS");
            auto vec = _sqlite_ptr->m_vResult;
            double angle1 = 0,angle2 = 0,angle3 = 0;
            for(uint i=0;i<vec.size();i++)
            {
                auto kv = vec[i];
                if(kv["COMNAME"] == "CURMOTORANGLE2")
                {
                    angle1 = stringToNum(kv["COMVALUE"]);
                    break;
                }
            }
            if(abs(angle1-m1s) > 0.1 ||abs(angle2-m2s) > 0.1  ||abs(angle3-m3s) > 0.1)
            {
                if(angle1 > loc)
                {
                    _mb_ptr->rotateMotor(ROTATINGDISK,loc);
                }
                if(angle2 < safeA1)
                {
                    _mb_ptr->rotateMotor(FLIPDISK,safeA1);
                }
                _mb_ptr->rotateMotor(ROTATINGDISK,m3s);
                _mb_ptr->rotateMotor(FLIPDISK,m2s);
                _mb_ptr->rotateMotor(GROUNDRAIL,m1s);
            }
            if(abs(v1)> 1)
                _mb_ptr->rotateMotor(GROUNDRAIL,m1e,abs(v1),2);
            if(abs(v2)> 1)
                _mb_ptr->rotateMotor(FLIPDISK,m2e,abs(v2),2);
            if(abs(v3)> 1)
                _mb_ptr->rotateMotor(ROTATINGDISK,m3e,abs(v3),2);
            _sys_config.put("robot.speed",100) ;
            _sys_config.sync();
            RobotPos begin = posList[0];
            retractMove(begin,departLen);
            _robot->absoluteMove(begin,true,true);
            _robot->absoluteMove(posList[0],true,true);
            _robot->setRotateTrue();
            _robot->absoluteMove(posList);
            _robot->setRotateFalse();
            RobotPos end = posList[posList.size()-1];
            retractMove(end,departLen);
            _robot->absoluteMove(end,true,true);
            _robot->absoluteMove(safe);
        }
    }).detach();
}

void Tracing::on_testRotate_clicked()
{
    std::thread([this]()mutable{
        try
        {
            _mb_ptr->rotateMotor(GROUNDRAIL,0,10,1);
            _mb_ptr->rotateMotor(FLIPDISK,5,36,1);
            _mb_ptr->rotateMotor(ROTATINGDISK,-60,200,1);
            _mb_ptr->rotateMotor(GROUNDRAIL,100,10,2);
            _mb_ptr->rotateMotor(FLIPDISK,-5,10,2);
            _mb_ptr->rotateMotor(ROTATINGDISK,-40,10,2);
            cout<<"ROTATE OK!"<<endl;
        }catch(...)
        {
            cout<<"ROTATE err!"<<endl;
        }
    }).detach();
}

void Tracing::on_pushButton_2_clicked()
{
    vector<RobotPos> seam0;
    ifstream ifs;
    ifs.open("keel.txt");
    string line;
    while(getline(ifs,line))
    {
        RobotPos pos;
        if(str2Robot(line,pos))
        {
            seam0.push_back(pos);
        }
    }
    ifs.close();
    fitSeamPath("test",seam0);
    ofstream ofs;
    ofs.open("keel2.txt");
    for(auto p : seam0)
    {
        ofs<<p.toStr()<<endl;
    }
    ofs.close();
}

void Tracing::on_pushButton_3_clicked()
{
    vector<RobotPos> seam0;
    ifstream ifs;
    ifs.open("keel.txt");
    string line;
    while(getline(ifs,line))
    {
        RobotPos pos;
        if(str2Robot(line,pos))
        {
            int ss = seam0.size();
            if(ss > 0)
            {
                auto se = seam0[ss-1];
                if(se.dis(pos) > 1)
                {
                    RobotPos pos_ = {(se.x+pos.x)/2,(se.y+pos.y)/2,(se.z+pos.z)/2,
                                     (se.a+pos.a)/2,(se.b+pos.b)/2,(se.c+pos.c)/2,UPLOAD};
                    seam0.push_back(pos_);
                }
            }
            seam0.push_back(pos);
        }
    }
    ifs.close();
    ofstream ofs;
    ofs.open("keel2.txt");
    for(auto p : seam0)
    {
        ofs<<p.toStr()<<endl;
    }
    ofs.close();
}

void Tracing::on_cbxCAD_currentIndexChanged(const QString &arg1)
{
    curCAD = arg1.toStdString();
    _sqlite_ptr->selectSQL2({"SEAMINFO"},{"SEAMID","SEAMNAME"},{"CADID","'"+curCAD+"'"});
    if(_sqlite_ptr->m_iRows == 0)
        return;
    auto vecseam = _sqlite_ptr->m_vResult;
    ui->cbxSEAM->clear();
    map<string,string> empty;
    seamMap.swap(empty);
    for(auto& v:vecseam)
    {
        seamMap[v["SEAMNAME"]] = v["SEAMID"];
        ui->cbxSEAM->addItem(QString::fromStdString(v["SEAMNAME"]));
    }
}

void Tracing::on_btnPosition1_clicked()
{
    vector<RobotPos> data;
    ifstream ifs;
    ifs.open("circleModel1.txt");
    string s;
    while(getline(ifs,s))
    {
        RobotPos r;
        if(str2Robot(s,r))
        {
            data.push_back(r);
        }
    }
    if(data.size()<2)
    {
        QMessageBox::about(NULL,"提示","计算坐标系1的数据不足,检查circleModel1.txt!");
    }
    point3dd dir_x = {data[1].x-data[0].x,data[1].y-data[0].y,data[1].z-data[0].z};
    point3dd dir_z = {0,0,1};
    point3dd dir_y = dir_z.cross(dir_x);
    dir_z = dir_x.cross(dir_y);
    double xm = dir_x.norm();
    double ym = dir_y.norm();
    double zm = dir_z.norm();
    Matrix4d mr;
    mr<<dir_x._x/xm,dir_x._y/xm,dir_x._z/xm,0,
            dir_y._x/ym,dir_y._y/ym,dir_y._z/ym,0,
            dir_z._x/zm,dir_z._y/zm,dir_z._z/zm,0,
            0,          0,          0,1;
    RobotPos cpos;
    Matrix2pos(mr,cpos,zyz);
    cout<<"cpos:0,0,0,"<<cpos.a<<","<<cpos.b<<","<<cpos.c<<endl;
}

void Tracing::on_btnPosition2_clicked()
{
    CircleModel cmdel;
    vector<RobotPos> data;
    ifstream ifs;
    ifs.open("circleModel2.txt");
    string s;
    while(getline(ifs,s))
    {
        RobotPos r;
        if(str2Robot(s,r))
        {
            data.push_back(r);
        }
    }
    ifs.close();
    cmdel.build(data);
    cout<<"cpos:"<<cmdel.printCPos()<<"R:"<<cmdel.getR()<<"rms:"<<cmdel.getRMS()<<endl;
}

void Tracing::on_testFit_clicked()
{
    ifstream ifs;
    ifs.open("fit/path.txt");
    point3dd pte;
    point3dd ppte;
    string s;
    getline(ifs,s);
    RobotPos r;
    if(str2Robot(s,r))
    {
        pte = {r.x,r.y,r.z};
    }
    getline(ifs,s);
    if(str2Robot(s,r))
    {
        ppte = {r.x,r.y,r.z};
    }
    p2p_len = pte.dist(ppte);
    vector<double> _x[3];
    while(getline(ifs,s))
    {
        if(str2Robot(s,r))
        {
            _x[0].push_back(r.x);
            _x[1].push_back(r.y);
            _x[2].push_back(r.z);
        }
    }
    ifs.close();
    SepModel3D smd;
    smd.setParams({TRACK_RANK});
    smd.setRealSE(0,_x[0].size());
    smd.setData(_x[0],_x[1],_x[2]);
    point3dd smdpos;
    try
    {
        smd.build();
        double t = 0;
        while(t<smd.length)
        {
            point3dd pt = smd.model(t);
            cout<<pt._x<<","<<pt._y<<","<<pt._z<<endl;
            t+=1;
        }
        cout<<"target:"<<endl;
        if(getModelPos(smd,smdpos,pte,ppte,p2p_len))
        {
            cout<<smdpos._x<<","<<smdpos._y<<","<<smdpos._z<<endl;
        }
    }catch(...)
    {
    }
}

/// 只负责机器人运动控制,记录:
/// 拟合转换到base的pos及速度序列
/// 记录变位机坐标系下的原始pos序列
/// udp反馈的pos序列及时间
int Tracing::tracking_hg(string seamId)
{
    static uint weldPoints = 0;  /// 已焊接点数
    int ret = RESULT_OK;
    /// 处理原始点列,准备给机器人发送位置运动指令
    ofstream track_ofs( logPath + "/path5.log") ;//给机器人发送的点列信息
    track_ofs<<"跟踪时发送给机器人运动的pos序列	运动速度	变位机转动角度（弧度）	期望到达的时间"<<endl;
    try
    {
        Config _sys_config("etc/sys.info");
        double maxSpeed = _sys_config.get<double>("tracing.maxSpeed",20);
        cout<<"发送变位机转动信号，待转模式,angle:"<<angleInfo.motore.angle3<<endl;
        double scanspeed = _sys_config.get<double>("robot.tracingScan",10);
        _sys_config.put<double>("robot.speed",scanspeed);
        _sys_config.sync();
        _robot->absoluteMove(scaneEnd);  /// 运动到扫描终点
        waitMode = true; /// 等待，直到机器人运动到焊接起点位置了
        _mb_ptr->openAirValve();   /// 打开气阀，传感器进入跟踪状态
        int wvCheck = 3000;
        while(wvCheck--)
        {
            std::this_thread::sleep_for(1_ms);  /// 等待3秒
            if(weldVec.size()>=3)
                break;
        }
        if(weldVec.size() < 3)  /// 3秒内无焊接轨迹，起头失败，跟踪逻辑异常退出
        {
            track_ofs<<"起头3秒内无轨迹!"<<endl;
            throw -1;
        }
        _mb_ptr->rotateNew(angleInfo.motore.angle1,abs(angleInfo.angleSpeed1),
                           angleInfo.motore.angle2,abs(angleInfo.angleSpeed2),
                           angleInfo.motore.angle3,abs(angleInfo.angleSpeed3),2);  /// 发送变位机转动信号，待转模式
        //_mb_ptr->rotateNew(-35,25,-5,8,-8,8,2);  /// 发送变位机转动信号，待转模式
        auto p = weldVec[0];
        posInBase(p,tool,getPosition(angleInfo.motors));
        double curTime = 0;
        _sys_config.put("robot.speed",START_POINT_SPEED) ; /// 运动到焊接起点，固定速度
        _sys_config.sync();
        char str[256];
        _robot->absoluteMove(p);
        _robot->pathLog = true;
        waitMode = false; /// 到达起点，无需等待
        cameraMode = 1; /// 跟踪状态
        weldStatus = false;
        weldPoints = 0;
        double p2pTime = p2p_len*6/(wf.S_W);   /// 根据焊接速度，确定机器人跟踪点到点的运动时间
        int rotateIndex = 2;   /// 从准备发送第3个点开始转动
        int cp = 2; /// cp move point mis = 2
        uint udp12 = 0;
        while(1)
        {
            std::this_thread::sleep_for(1_ms);  /// 等待
            if(weldVec.size()<=weldPoints+1)
                break;
            if(weldPoints == rotateIndex)
            {
                _robot->setRotateTrue();
                angleInfo.rotateTime = getMicTime()+0.01;
                track_ofs<<"开始转动时间:"<<angleInfo.rotateTime<<endl;
            }
            auto p = weldVec[weldPoints];
            curTime = getMicTime();
            MOTORDEGRE dd = {0,0,0};
            if(angleInfo.rotateTime > 1)
            {
                dd.angle1 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed1;
                dd.angle2 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed2;
                dd.angle3 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed3;
                if(abs(dd.angle1)>abs(angleInfo.motore.angle1-angleInfo.motors.angle1))
                {
                    dd.angle1 = angleInfo.motore.angle1-angleInfo.motors.angle1;
                }
                if(abs(dd.angle2)>abs(angleInfo.motore.angle2-angleInfo.motors.angle2))
                {
                    dd.angle2 = angleInfo.motore.angle2-angleInfo.motors.angle2;
                }
                if(abs(dd.angle3)>abs(angleInfo.motore.angle3-angleInfo.motors.angle3))
                {
                    dd.angle3 = angleInfo.motore.angle3-angleInfo.motors.angle3;
                }
            }
            double targetTime;
            if(weldPoints >= rotateIndex)
                targetTime = angleInfo.rotateTime+p2pTime*(weldPoints-rotateIndex+cp);
            else
                targetTime = 0;
            MOTORDEGRE degree = angleInfo.motors;
            degree.angle1 += dd.angle1;
            degree.angle2 += dd.angle2;
            degree.angle3 += dd.angle3;
            posInBase(p,tool,getPosition(degree));
            auto cur = _robot->getCurrPos( RobotTCP::UPLOAD );
            double distence = cur.dis(p);
            if(targetTime < 1)
            {
                p.v = wf.S_W/6;
            }
            else
            {
                if(targetTime-curTime > 0.01)
                {
                    p.v = distence/(targetTime-curTime);
                }
                else
                {
                    p.v = maxSpeed;
                }
            }
            p.v = p.v<maxSpeed?p.v:maxSpeed;
            p.v = p.v>1?p.v:1;
            if(weldFlag)
            {
                p.v *= 6;   /// 焊接状态下，速度单位是cm/min 空走时单位 mm/sec
            }
            vector<RobotPos> vmi;
            vmi.push_back(p);
            auto w = weldVec[weldPoints];
            ::sprintf(str,"%.3f,%.3f,%.3f,%.1f,%.1f,%.1f %.3f,%.3f,%.3f,%.1f,%.1f,%.1f %.1f angle3 %.1f cur %.3f tar %.3f "
                      ,w.x,w.y,w.z,w.a,w.b,w.c,p.x,p.y,p.z,p.a,p.b,p.c,p.v,degree.angle3,curTime,targetTime);
            track_ofs <<str;
            if(Pos2Joint2(vmi))   /// 检查出目标位置机器人运动不到
            {
                ret = 1;
                track_ofs<<"机器人位置不可达"<<endl;
                throw -1;
            }
            auto next = weldVec[weldPoints+1];
            if(weldFlag)
            {
                if(udp12 == 0 && next.weld)
                {
                    if(!weldStatus)
                    {
                        _robot->setWeldingTrue(wf);
                        weldStatus = true;
                    }
                }
                if((weldStatus && next.weld == false) || (!weldStatus && next.weld == true)) /// 状态切换
                {
                    udp12 = 9999;
                }
                _robot->sendWeldPoss(udp12++,p,offset,base_offset,default_h,autoFit,wf,(distence > 0.1));
                if(udp12 == 10000)
                {
                    udp12 = 0;
                    if(weldStatus)
                    {
                        _robot->setWeldingFalse();
                        weldStatus = false;
                    }
                }
            }
            else
            {
                _robot->sendWeldPoss(udp12++,p,offset,base_offset,default_h,autoFit,wf,(distence > 0.1));
            }
            backPath.push_back(p);
            track_ofs<<_robot->weldIndex;
            curTime = getMicTime();
            double timewait = targetTime-curTime-p2pTime;
            track_ofs<<" "<<timewait<<endl;
            if(timewait>0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(d_round(timewait*1000)));
            }
            weldPoints++;
        }
        auto endp = weldVec[weldPoints];
        auto endpre = weldVec[weldPoints-1];
        double endp2p = endp.dis(endpre)/p2p_len;
        curTime = getMicTime();
        MOTORDEGRE dd = angleInfo.motors;
        dd.angle1 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed1;
        dd.angle2 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed2;
        dd.angle3 = (weldPoints-rotateIndex+cp)*p2pTime*angleInfo.angleSpeed3;
        if(abs(dd.angle1)>abs(angleInfo.motore.angle1-angleInfo.motors.angle1))
        {
            dd.angle1 = angleInfo.motore.angle1-angleInfo.motors.angle1;
        }
        if(abs(dd.angle2)>abs(angleInfo.motore.angle2-angleInfo.motors.angle2))
        {
            dd.angle2 = angleInfo.motore.angle2-angleInfo.motors.angle2;
        }
        if(abs(dd.angle3)>abs(angleInfo.motore.angle3-angleInfo.motors.angle3))
        {
            dd.angle3 = angleInfo.motore.angle3-angleInfo.motors.angle3;
        }
        MOTORDEGRE degree = angleInfo.motors;
        degree.angle1 += dd.angle1;
        degree.angle2 += dd.angle2;
        degree.angle3 += dd.angle3;
        posInBase(endp,tool,getPosition(degree));
        auto cur = _robot->getCurrPos( RobotTCP::UPLOAD );
        double targetTime = angleInfo.rotateTime+p2pTime*(weldPoints-1-rotateIndex+cp)+p2pTime*endp2p;
        if(targetTime-curTime > 0.01)
        {
            endp.v = cur.dis(endp)/(targetTime-curTime);
        }
        else
        {
            endp.v = maxSpeed;
        }
        endp.v = endp.v<maxSpeed?endp.v:maxSpeed;
        endp.v = endp.v>1?endp.v:1;
        if(weldFlag)
        {
            endp.v *= 6;   /// 焊接状态下，速度单位是cm/min 空走时单位 mm/sec
        }
        auto ew = weldVec[weldPoints];
        vector<RobotPos> vmi;
        vmi.push_back(endp);
        ::sprintf(str,"%.3f,%.3f,%.3f,%.1f,%.1f,%.1f %.3f,%.3f,%.3f,%.1f,%.1f,%.1f %.1f angle3 %.1f cur %.3f tar %.3f end"
                  ,ew.x,ew.y,ew.z,ew.a,ew.b,ew.c,endp.x,endp.y,endp.z,endp.a,endp.b,endp.c,endp.v,degree.angle3,curTime,targetTime);
        track_ofs <<str;
        if(Pos2Joint2(vmi))
        {
            ret = 2;
            track_ofs<<"机器人位置不可达"<<endl;
            throw -1;
        }
        _robot->setRotateFalse();
        _robot->sendWeldPoss(9999,endp,offset,base_offset,default_h,autoFit,wf);
        track_ofs<<_robot->weldIndex<<endl;
        char msg[64];
        sprintf(msg,"tracking end：%.3f",getMicTime());
        track_ofs<<msg<<endl;
    }catch(...)
    {
        ret = ERR_FUNC_SERVICE_PROCESS_RUN_ERROR;
        char msg[64];
        sprintf(msg,"tracking err end：%.3f",getMicTime());
        track_ofs<<msg<<endl;
        /// 跟踪异常退出
    }
    if(weldStatus)
        _robot->setWeldingFalse();
    if(weldVec.size() < seamKeel.size()-10)
    {
        Config _sys_config(SYS);
        _sys_config.put<int>("weld.lastEnd",weldVec.size());
        _sys_config.sync();
    }
    m_bTaking = false;
    _robot->pathLog = false;
    track_ofs.close();
    return ret;
}

int Tracing::imgProcess_hg(string seamId)
{
    string option = seamInfo["OPTION"];
    int tc = 0;
    for(uint i=0;i<option.length();i++)
    {
        tc *= 2;
        if(option.substr(i,1) == "1")
        {
            tc+=1;
        }
    }
    SelfAdaption * _hgA = new SelfAdaption();
    if(tc&0x01)
        _hgA->SetSide(true);
    else
        _hgA->SetSide(false);

    Config _sys_config(SYS);
    double endlen = _sys_config.get<double>("fit.endlen",10);
    double c2tx = _sys_config.get<double>("fit.c2tx",33);
    double fitps = _sys_config.get<double>("fit.fitps",3);
    ofstream imglog(logPath + "/path2.log") ;//打开记录机器人坐标位置文件
    imglog<<"照片序号	uv值	时间	UV23D计算出的pos	变位机角度变化（弧度）	已知原始点容器大小/待处理容器大小	原始点数据用于拟合的序号区间"<<endl;
    int count = 0;
    int exitCount = 0;
    int IMG_ERR_LIMIT = d_round(10*maxGap*6/wf.S_W); // 每秒10张
    int ss = seamKeel.size();
    static double preTime = getMicTime();
    bool saveFlag = false;
    UV pre_uv = {0,0};  /// 前一次算法的uv
    try
    {
        while(m_bTaking || !dataSet.empty())
        {
            double inTime = getMicTime();
            std::this_thread::sleep_for(1_ms);
            Mat m;
            if(dataSet.empty())
                continue;
            SensorDataPack mpa;
            {
                std::lock_guard<std::mutex> lck(_mtx);
                mpa = dataSet.front();
                mpa.m.copyTo(m);
                dataSet.pop();
            }
            cv::Point2d puv;
            cout<<"img algo in!"; //cv::flip(m,m,1);
            if(weldStatus)
            {
                _gxA->set_weld_on();
            }
            else
            {
                _gxA->set_weld_off();
            }
            switch(imageLevel)
            {
            case 0:
                break;
            case 1:
                imwrite(logPath+"/"+to_string(count)+".png",m);
                break;
            }
            if(getMicTime() > preTime+1)
            {
                preTime = getMicTime();
                if(imageLevel == 2)
                {
                    imwrite(logPath+"/"+to_string(count)+".png",m);
                    saveFlag = true;
                }
            }
            _gxA->AddImg(m);
            puv = _gxA->getuv(); //puv.x= m.cols-puv.x;
            cout<<"img algo ok!"<<endl;
            char stime[256];
            ::sprintf(stime,"inTime:%.3fuse%.3fs ",inTime,getMicTime()-inTime);
            imglog<<stime<<" "<<count;
            UV tar = {puv.x,puv.y};
            RobotPos r;
            int ws = weldVec.size();
            imglog<<" 当前点数"<<ws<<"龙骨点数"<<ss<<" ";
            if(abs(ss-ws) <= 1 && rInfos.endPath.size() > 0)/// 进入结束点关联逻辑，停止拍照
            {
                if(tracing_end_correlation(logPath+"/correlation_end.log",oriPath,
                                           rInfos.endPath,rInfos.endExtern,rInfos.endDistance,weldVec,p2p_len))
                {
                    m_bTaking = false;
                    break;
                }
            }
            double x;
            MOTORDEGRE magl = getAngleByTime(mpa.time, angleInfo);
            imglog<<mpa.cameraMode<<"angle:"<<magl.angle1<<"/"<<magl.angle2<<"/"<<magl.angle3<<" ";
            posCheck(mpa,x);
            if(x > 0)
            {
                imglog<<x<<"\tscaned pos!"<<endl;
            }
            bool uvOk = checkUV(tar,ct[1]);
            int ks = seamKeel.size();
            bool toEnd = ws >= ks-endKeelIndex;
            imglog<<toEnd<<"/"<<end_err_limit<<"/"<<IMG_ERR_LIMIT<<"/"<<exitCount<<"/";
            vector<double> lruv;
            _gxA->GetWhatYouWant(lruv);
            if(lineType == LT_WW)
            {
                if(lruv.size() < 4)
                {
                    uvOk = false;
                }
                else
                {
                    float py,pz,ly,lz,ry,rz;
                    float fu = (float)ct[1].u;
                    float fv = (float)ct[1].v;
                    laser_yz((float)(puv.x)-fu,(float)(puv.y)-fv,py,pz,1);
                    laser_yz((float)lruv[0]-fu,(float)lruv[1]-fv,ly,lz,1);
                    laser_yz((float)lruv[2]-fu,(float)lruv[3]-fv,ry,rz,1);
                    double dw = (ly-py)*(ly-py)+(lz-pz)*(lz-pz);
                    double dh = (ry-py)*(ry-py)+(rz-pz)*(rz-pz);
                    if(dw > 4*default_w*default_w
                            || dh > 4*default_h*default_h
                            || dw < 0.25*default_w*default_w
                            || dh < 0.25*default_h*default_h)
                    {
                        imglog<<"dw:"<<dw<<"/dh:"<<dh<<" "
                             <<puv.x << "," <<puv.y << "/" <<lruv[0] << "," <<lruv[1]
                              << "/" <<lruv[2] << "," <<lruv[3]<< endl;
                        uvOk = false;
                    }
                }
            }
            if(!uvOk) /// 跟踪过程中图像异常
            {
                exitCount++;
                imglog<<count<<"\terr"<<endl;
                int limit = toEnd?end_err_limit:IMG_ERR_LIMIT; /// 靠近结尾图像异常相对严格控制
                if(exitCount > limit) /// 连续IMG_ERR_LIMIT张图像异常，退出跟踪逻辑
                {
                    vector<RobotPos> cleanPath;
                    int os = oriPath.size();
                    for(int e=0;e<os-exitCount;e++)
                    {
                        cleanPath.push_back(oriPath[e]);
                    }
                    fitTracingErrEnd(imglog,cleanPath,seamKeel,weldVec,p2p_len, endlen);
                    throw -1;
                }
                if(abs(pre_uv.u) < 1 || abs(pre_uv.v)<1)
                {
                    count ++;
                    continue;
                }
                /// getPosFromKeel(r,oriPath,weldVec,seamKeel,p2p_len); 从龙骨序列中提取未来的pos
                tar = pre_uv;
            }
            /// 图像无异常
            UV2Point3D(mpa.r,tar,r,mpa.cameraMode);
            emit printUV(tar.u,tar.v);
            char str[256];
            ::sprintf(str,"%duv\t%.3f,%.3f\tt\t%.3f 23d ",count,tar.u,tar.v,mpa.time);
            imglog <<str<<r.toStr();
            RobotPos rTool = r;
            posInTool(rTool,tool,getPosition(magl));
            imglog<<" "<<oriPath.size()<<"/"<<dataSet.size()<<" ";
            if(appendWeldPath(logPath,rTool,imglog,oriPath,seamKeel,weldVec,p2p_len, rInfos,endlen,c2tx,fitps))
            {
                pre_uv = tar;
                if(uvOk)
                {
                    exitCount --;
                    exitCount = exitCount<0?0:exitCount;
                }
            }
            else
            {
                if(uvOk)
                {
                    exitCount ++;
                }
                uvOk = false;
                int ws = weldVec.size();
                int ks = seamKeel.size();
                int limit = ws >= ks-endKeelIndex?end_err_limit:IMG_ERR_LIMIT; /// 靠近结尾图像异常相对严格控制
                if(exitCount > limit) /// 连续IMG_ERR_LIMIT张图像异常，退出跟踪逻辑
                {
                    vector<RobotPos> cleanPath;
                    int os = oriPath.size();
                    for(int e=0;e<os-exitCount;e++)
                    {
                        cleanPath.push_back(oriPath[e]);
                    }
                    fitTracingErrEnd(imglog,cleanPath,seamKeel,weldVec,p2p_len, endlen);
                    throw -1;
                }
                /// getPosFromKeel(r,oriPath,weldVec,seamKeel,p2p_len); 从龙骨序列中提取未来的pos
                tar = pre_uv;
            }
            if(imageLevel == 2 && !uvOk && !saveFlag)
                imwrite(logPath+"/"+to_string(count)+".png",m);
            imglog<<endl;
            count++;
        }
        delete _gxA;
        char msg[64];
        sprintf(msg,"imgProcess end：%.3f",getMicTime());
        imglog<<msg<<endl;
    }
    catch(...)
    {
        char msg[64];
        sprintf(msg,"imgProcess err end：%.3f",getMicTime());
        imglog<<msg<<endl;
        m_bTaking = false;
        return ERR_FUNC_SERVICE_PROCESS_RUN_ERROR;
    }
    imglog.close();
    return RESULT_OK;
}

/// PATH3 离线拟合
void Tracing::on_simuFit_clicked()
{
    char timeStr[256];
    sprintf(timeStr,"in time %.3f",getMicTime());
    cout<<timeStr<<endl;
    ifstream ifs;
    ifs.open("fit/simulate.txt");
    p2p_len = 5;
    string s;
    vector<RobotPos> ori;
    uint rec = 0;
    while(getline(ifs,s))
    {
        rec++;
        RobotPos r;
        if(str2Robot(s,r))
        {
            //int turb = rand(); /// 随机因子
            //if(rec<1000 || rec>1030)
                ori.push_back(r);
//            else
//            {
//                RobotPos off = RobotPos::instance();
//                off.y = rand()%11-5;
//                off.z = rand()%11-5;
//                ori.push_back(r<<off);
//            }
        }
    }
    ifs.close();
    int os = ori.size();
    vector<RobotPos> fitPs; /// 拟合片段
    ofstream ofs;
    ofs.open("fit/simulateOut.txt");
    /// 前一个pos
    point3dd pte;
    point3dd ppte;
    RobotPos p0 = ori[0];
    char str[128];
    int initpx=0;
    for(;initpx<os;initpx++)
    {
        RobotPos pi = ori[initpx];
        fitPs.push_back(pi);
        if(pi.dis(p0) >= START_LEN_LIMIT*p2p_len)
        {
            SepModel3D smd;
            smd.setParams({TRACK_RANK});
            vector<double> _x[3];
            for(auto & pt : fitPs)
            {
                _x[0].push_back(pt.x);
                _x[1].push_back(pt.y);
                _x[2].push_back(pt.z);
            }
            smd.setRealSE(0,fitPs.size());
            smd.setData(_x[0],_x[1],_x[2]);
            point3dd smdpos;
            try
            {
                smd.build();
                /// 放入第0个pos
                pte = smd.model(0);
                ::sprintf(str,"%.3f,%.3f,%.3f",pte._x,pte._y,pte._z);
                ofs<<str<<endl;
                double len = smd.length;
                double cur = 0.001;
                int sk = 1;
                while(cur < len)
                {
                    smdpos = smd.model(cur);
                    if(abs(smdpos.dist(pte)-p2p_len) < 0.01)
                    {
                        ppte = pte;
                        pte = smdpos;
                        sk++;
                        ::sprintf(str,"%.3f,%.3f,%.3f",pte._x,pte._y,pte._z);
                        ofs<<str<<endl;
                    }
                    cur += 0.001;
                }
            }catch(...)
            {
            }
            break;
        }
    }
    int px = initpx;
    int fitIndex = 0;
    while(px < os)
    {
        point3dd opx = {ori[px].x,ori[px].y,ori[px].z};
        if(opx.dist(pte) < 2*p2p_len)
        {
            px++; // no need to fit
            continue;
        }
        vector<RobotPos>().swap(fitPs);
        int fi= fitIndex;
        fi = fi>20?fi-20:fi;
        fi = fi<0?0:fi;
        bool updateFitIndex = false;
        for(;fi<=px;fi++)
        {
            auto p = ori[fi];
            point3dd pp = {p.x,p.y,p.z};
            if(pte.dist(pp)<p2p_len*FIT_SPAN)
            {
                fitPs.push_back(p);
                if(!updateFitIndex)
                {
                    fitIndex = fi-10;
                    updateFitIndex = true;
                }
            }
        }
        if(fitPs.size() < TRACK_POINT_LIMIT)
        {
            return;
        }
        SepModel3D smd;
        smd.setParams({TRACK_RANK});
        vector<double> _x[3];
        for(auto & p : fitPs)
        {
            _x[0].push_back(p.x);
            _x[1].push_back(p.y);
            _x[2].push_back(p.z);
        }
        smd.setRealSE(0,fitPs.size());
        smd.setData(_x[0],_x[1],_x[2]);
        point3dd smdpos;
        try
        {
            smd.build();
            if(getModelPos(smd,smdpos,pte,ppte,p2p_len))
            {
                ppte = pte;
                pte = smdpos;
                ::sprintf(str,"%.3f,%.3f,%.3f",pte._x,pte._y,pte._z);
                ofs<<str<<endl;
            }
        }catch(...)
        {
        }
        px ++;
    }
    ofs.close();
    sprintf(timeStr,"out time %.3f",getMicTime());
    cout<<timeStr<<endl;
}

/// 开始跟踪逻辑
void Tracing::workTracing_hg()
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

void Tracing::on_testRotate_2_clicked()
{
    vector<RobotPos> vec;
    getFramePos(587,vec);
    RobotPos c2t = RobotPos::instance("-32.822,5.341,1.476,-23.604,0.21,23.431");
    for(int i=0;i<vec.size();i++)
    {
        RobotPos p1 = vec[i];
        RobotPos p2 = vec[i+1];
        RobotPos dir = p2-p1;
        point3dd d3d = {dir.x,dir.y,dir.z};
        point3dd dz = {0,0,1};
        point3dd dy = dz.cross(d3d);
        dy.normalize();
        point3dd dx = dy.cross(dz);
        dx.normalize();
        Matrix4d m4d;
        m4d<<dx._x,dy._x,dz._x,0,
             dx._y,dy._y,dz._y,0,
             dx._z,dy._z,dz._z,0,
             0    ,    0,    0,1;
        RobotPos ptool = RobotPos::instance();
        Matrix2pos(m4d,ptool,zyz);

    }
}

