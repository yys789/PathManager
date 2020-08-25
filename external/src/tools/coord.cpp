#include "tools/coord.h"
#include "tools/matrix.h"
#include "base/config.h"
#include "tools.h"

#include <vector>
using namespace Eigen ;
using namespace std ;

/// 计算某个点绕基坐标每个轴旋转后的欧拉角度
/// @param [in] flangePos   当前法兰坐标
/// @param [in] pointInTCP  要计算的旋转点在法兰中的位置
/// @param [in] rotation_vec  旋转的坐标轴, 每次只能传一个
/// @param [out] newPos 计算出的最终法兰欧拉角
void TCPtrans(const RobotPos &oriKCS, const RobotPos &TCP, array<double, 3> rotAngle, RobotPos &New_KCS)
{
     using namespace Eigen;
     auto KCS = oriKCS ;

     Config sys_config ;
     Matrix3d  RKCS, RrotAngle, RTCP, RNew_KCS;
     pos2Matrix(KCS, RKCS, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
     pos2Matrix(TCP, RTCP, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
     RobotPos rot{0, 0, 0, rotAngle[0], rotAngle[1], rotAngle[2]};
     pos2Matrix(rot, RrotAngle, EULERTYPE::zyx);
     RNew_KCS = RrotAngle*RKCS;
     Matrix2pos(RNew_KCS, New_KCS, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));

     MatrixXd A(3, 1),B(4,1);

     A(0,0) = TCP.x;
     A(1,0) = TCP.y;
     A(2,0) = TCP.z;
     B = (RNew_KCS-RKCS)*A;
     New_KCS.x = KCS.x - B(0,0);
     New_KCS.y = KCS.y - B(1,0);
     New_KCS.z = KCS.z - B(2,0);
}

/// 计算某个点绕基坐标每个轴旋转后的欧拉角度
/// @param [in] flangePos   当前法兰坐标
/// @param [in] pointInTCP  要计算的旋转点在法兰中的位置
/// @param [in] rotation_vec  旋转的坐标轴次序[z, y, x]
/// @param [out] newPos 计算出的最终法兰欧拉角
void pointRotate( const RobotPos &flangePos, const RobotPos &pointInFlange, const vector<RobotPos> &rotation_vec, RobotPos &newPos )
{
    auto currPos = flangePos ;
    for ( auto &rotation : rotation_vec ) {
        TCPtrans(currPos, pointInFlange, {rotation.a, 0, 0}, newPos ) ;
        TCPtrans(newPos, pointInFlange, {0, rotation.b, 0}, newPos ) ;
        TCPtrans(newPos, pointInFlange, {0, 0, rotation.c}, newPos ) ;
        newPos.x += rotation.x ;
        newPos.y += rotation.y ;
        newPos.z += rotation.z ;
        currPos = newPos ;
    }
    newPos.tcp = flangePos.tcp ;
}

/// 坐标系变化
/// @param [in] currPos point 在坐标系A中的坐标
/// @param [in] rotatePos 坐标系A旋转, point在空间中的位置不变
/// @param [out] outPos point 在坐标系旋转后的坐标
void coordinateRotate( const RobotPos &inPos, const RobotPos &rotatePos, RobotPos &outPos )
{
   using namespace Eigen ;
   Matrix4d  RKCS, RrotAngle, RTCP, RNew_KCS;
   Vector4d Xb(4, 1), Xt(4, 1), B(4,1);

   auto currPos = inPos ;

   Xb(0,0) = currPos.x;
   Xb(1,0) = currPos.y;
   Xb(2,0) = currPos.z;
   Xb(3,0) = 1;

   //pos2Matrix(currPos, RKCS);
   //求姿态信息
   Matrix3d Rb, Rt, Rkcs;
   Config sys_config ;

   pos2Matrix(currPos, Rb, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   pos2Matrix(rotatePos, Rkcs, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   Rt = Rkcs.inverse()*Rb;
   Matrix2pos(Rt, outPos, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));

   //求位置信息
   pos2Matrix(rotatePos, RKCS, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   Xt = RKCS.inverse()*Xb;
   outPos.x = Xt(0,0);
   outPos.y = Xt(1,0);
   outPos.z = Xt(2,0);
}

/// 计算两点的旋转矩阵
/// @param [in] posA
/// @param [in] posB
Matrix4d getRotateMatrix( const RobotPos &posA, const RobotPos &posB )
{
   Matrix4d  RKCS, RNew_KCS;
   MatrixXd Xb(4, 1), Xt(4, 1);
   RobotPos outPos;


   Xb(0,0) = posB.x;
   Xb(1,0) = posB.y;
   Xb(2,0) = posB.z;
   Xb(3,0) = 1;

   //求姿态信息
   Matrix3d Rb, Rt, Rkcs;
   Config sys_config ;


   pos2Matrix(posB, Rb, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   pos2Matrix(posA, Rkcs, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   Rt = Rkcs.inverse()*Rb;
   Matrix2pos(Rt, outPos, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));

   //求位置信息
   pos2Matrix(posA, RKCS, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   Xt = RKCS.inverse()*Xb;
   outPos.x = Xt(0,0);
   outPos.y = Xt(1,0);
   outPos.z = Xt(2,0);

   pos2Matrix(outPos, RNew_KCS, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
   return RNew_KCS;
}

/// 根据两点间的旋转矩阵求另一点的坐标
/// @param [in] posA
/// @param [in] m
/// @param [out] posB
void rotateMatrix2Pos( const RobotPos &posA, const Eigen::Matrix4d &m, RobotPos &posB )
{
  Matrix4d RKCS, Rb;
  Config sys_config ;
  pos2Matrix(posA, RKCS, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
  Rb = RKCS*m;
  Matrix2pos(Rb, posB, static_cast<EULERTYPE>(sys_config.get<int>("robot.eulerType")));
}

void laser2Base(double TCP[], const RobotPos &Pos, RobotPos &outPos,int mode)
{
    RobotPos cameraPos;
    Config config("etc/calibration.info") ;
    cameraPos = config.get<RobotPos>("CINT"+int2str(mode)) ;
    RobotPos tempPos{TCP[0], TCP[1], TCP[2],0,0,0};
    outPos = Pos<<cameraPos<<tempPos;
}

void posInBase(RobotPos & p, RobotPos tool, RobotPos p2b)
{
    p = p>>tool>>p2b;
}

void posInTool(RobotPos & p, RobotPos tool, RobotPos p2b)
{
    p = p>>(!p2b)>>(!tool);
}

/// 将一串点往一个法平面上投影
void shadow(point3d<double> pabc, vector<RobotPos> & points)
{
    RobotPos zero = RobotPos::instance();
    for(auto & r : points)
    {
        if(r.dis(zero) > 1)
        {
            point3d<double> p ={r.x,r.y,r.z};
            double t = (1-p.inner(pabc))/pabc.inner(pabc);
            r.x += t*pabc._x;
            r.y += t*pabc._y;
            r.z += t*pabc._z;
        }
    }
}

/// 根据龙骨确定曲线方向
point3dd getKeelDir(point3dd cur, point3dd keelCur, point3dd keelNext)
{
    auto ck = cur.cross(keelCur);
    double xita = asin(ck.norm()/(cur.norm()*keelCur.norm()));
    ck.normalize();
    Vector3d v3d = {ck._x,ck._y,ck._z};
    AngleAxisd anglev(xita,v3d);
    Vector3d vnext = {keelNext._x,keelNext._y,keelNext._z};
    auto vn = anglev.matrix().inverse()*vnext;
    return {vn(0,0),vn(1,0),vn(2,0)};
}

/// 回撤一个距离
void retractMove(RobotPos & s,int len,int dy,int dx)
{
    RobotPos re = RobotPos::instance();
    re.x -= dx;
    re.y -= dy;
    re.z -= len;
    s = s<<re;
}

/// 返回两串点形成的曲线的空间距离
bool checkPath(vector<RobotPos> target, vector<RobotPos> bme,double & sumlen)
{
    if(target.size() < 3)
        return false;
    if(bme.size() < 1)
        return false;
    vector<point3dd> t3d;
    for(auto t : target)
    {
        t3d.push_back({t.x,t.y,t.z});
    }
    int ts = t3d.size();
    int bs = bme.size();
    double sum = 0;
    for(auto b : bme)
    {
        point3dd b3d = {b.x,b.y,b.z};
        int index = 0;
        double maxangle = -999;
        for(int i=0;i<ts-1;i++)
        {
            point3dd b1 = t3d[i]-b3d;
            point3dd b2 = t3d[i+1]-b3d;
            double inner = b1.inner(b2)/(b1.norm()*b2.norm());
            if(maxangle>inner)
            {
                maxangle = inner;
                index = i;
            }
        }
        point3dd bm1 = t3d[index]-b3d;
        point3dd bm2 = t3d[index+1]-b3d;
        point3dd bc12 = bm1.cross(bm2);
        point3dd b12 = t3d[index+1]-t3d[index];
        sum += bc12.norm()/b12.norm();
    }
    sum /= bs;
    cout<<"曲线跟龙骨的平均误差距离："<<sum<<endl;
    Config _sys_config(SYS);
    if(sum > _sys_config.get<int>("fit.PATHDIS",20))
        return false;
    for(int i=0;i<ts-1;i++)
    {
        point3dd t12 = t3d[i+1]-t3d[i];
        sumlen += t12.norm();
    }
    return true;
}

/// lr 地轨位置
/// angle1 翻转角度
/// angle2 旋转角度
RobotPos getPosition(MOTORDEGRE degrees)
{
    Config conf("etc/calibration.info");
    RobotPos pos0 = conf.get<RobotPos>("position2base0"); ///
    RobotPos r1 = RobotPos::instance();
    r1.x = degrees.angle1;
    RobotPos pos1 = conf.get<RobotPos>("position2base1"); /// 变位机在BASE坐标系下的表达
    pos1.c += degrees.angle2;
    RobotPos pos2 = conf.get<RobotPos>("position2base2"); /// 变位机在BASE坐标系下的表达
    pos2.c -= degrees.angle3;
    return pos0<<r1<<pos1<<pos2;
}

void mergePath(vector<RobotPos> & path,int tracing,vector<RobotPos> keelKeel)
{
    vector<RobotPos> weldKeel;
    for(auto & p : path)
    {
        weldKeel.push_back(p);
    }
    uint ws = weldKeel.size();
    if(ws < 3)
    {
        cout<<"原始点数太少，无法延展！"<<endl;
        return;
    }
    double sum = 0;
    for(uint i=1;i<ws;i++)
    {
        sum += weldKeel[i].dis(weldKeel[i-1]);
    }
    double wSpan = sum/(ws-1); /// 原始点平均间距
    uint ks = keelKeel.size();
    if(ks < 3)
    {
        cout<<"龙骨点数太少，无法延展！"<<endl;
        return;
    }
    sum = 0;
    for(uint i=1;i<ks;i++)
    {
        sum += keelKeel[i].dis(keelKeel[i-1]);
    }
    double keelSpan = sum/(ks-1); /// 龙骨点平均间距
    uint rate = d_round(keelSpan/wSpan);  /// 龙骨间距跟path间距的比值  例：5mm/1mm=5
    vector<RobotPos>().swap(path); /// 清空原有軌跡
    uint wkmax = ws>ks*rate?ws:ks*rate;  /// merge后的轨迹点数
    ofstream ofs;
    ofs.open("testLog/pathCompare.log",ios_base::app);  // 记录实际轨迹与龙骨的差异对比
    ofs<<fixed<<setprecision(3)<<getMicTime()<<endl;
    for(uint t=0;t<ws;t++)
    {
        uint i_ = t<ks*rate?t/rate:ks-1;
        RobotPos next = keelKeel[i_];
        RobotPos & wk = weldKeel[t];
        double wx = wk.x;
        double wy = wk.y;
        double wz = wk.z;
        wk = next;
        wk.x = wx;
        wk.y = wy;
        wk.z = wz;
        ofs<<next.toStr()<<"\t"<<wk.toStr()<<endl;
        continue;
    }
    if(tracing == 2)
    {
        for(uint t=ws;t<wkmax;t+=rate)
        {
            uint wsn = weldKeel.size();
            /// 扩展keelPath
            int tr = d_round(t/rate);
            int times = 1;
            point3dd avg = {0,0,0};
            RobotPos wr = weldKeel[wsn-1];
            RobotPos kp = keelKeel[tr];
            while(times < tr-1 && times*rate < wsn-1)
            {
                RobotPos wl = weldKeel[wsn-1-times*rate];
                point3dd wE = {wr.x-wl.x,wr.y-wl.y,wr.z-wl.z};
                RobotPos kl = keelKeel[tr-1-times];
                RobotPos kr = keelKeel[tr-1];
                point3dd kE = {kr.x-kl.x,kr.y-kl.y,kr.z-kl.z};
                point3dd kEp = {kp.x-kr.x,kp.y-kr.y,kp.z-kr.z};
                point3dd wEp = getKeelDir(wE,kE,kEp);
                avg._x += wEp._x;
                avg._y += wEp._y;
                avg._z += wEp._z;
                times++;
            }
            avg = {avg._x/times,avg._y/times,avg._z/times};
            for(uint q=1;q<=rate;q++)
            {
                RobotPos next = kp;
                next.x = wr.x+avg._x*q/rate;
                next.y = wr.y+avg._y*q/rate;
                next.z = wr.z+avg._z*q/rate;
                ofs<<kp.toStr()<<"\t"<<next.toStr()<<endl;
                weldKeel.push_back(next);
            }
        }
    }
    for(auto & p : weldKeel)
    {
        path.push_back(p);
    }
    ofs.close();
}


/// ok return false , bad pos return true
bool Pos2Joint2(vector<RobotPos> Pos_set)
{
    Config cali("etc/calibration.info");
    RobotPos hq = cali.get<RobotPos>("TINF");
    vector<Eigen::Matrix4d> R;
    Eigen::Matrix4d rz1, ry, rz2, r;
    Eigen::Matrix4d Tz, Txyz, Ro, Ra, Rt;
    RobotAxle agl;
    bool Limit = false;
    double dz6 = 115;
    double x = hq.x;
    double y = hq.y;
    double z = hq.z;
    double o = hq.a*PI/180;
    double a = hq.b*PI/180;
    double t = hq.c*PI/180;

    Tz << 1, 0, 0,   0,
            0, 1, 0,   0,
            0, 0, 1, dz6,
            0, 0, 0,   1;
    Txyz << 1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1;
    Ro << cos(o), -sin(o), 0, 0,
            sin(o),  cos(o), 0, 0,
            0,       0, 1, 0,
            0,       0, 0, 1;

    Ra << cos(a), 0, sin(a), 0,
            0, 1,      0, 0,
            -sin(a), 0, cos(a), 0,
            0, 0,      0, 1;

    Rt << cos(t), -sin(t), 0, 0,
            sin(t),  cos(t), 0, 0,
            0,       0, 1, 0,
            0,       0, 0, 1;

    for(auto p : Pos_set){
        rz1 << cos(p.a/(180/M_PI)), -sin(p.a/(180/M_PI)), 0, 0,
                sin(p.a/(180/M_PI)),  cos(p.a/(180/M_PI)), 0, 0,
                0,                    0, 1, 0,
                0,                    0, 0, 1;

        ry << cos(p.b/(180/M_PI)), 0, sin(p.b/(180/M_PI)), 0,
                0, 1,                   0, 0,
                -sin(p.b/(180/M_PI)), 0, cos(p.b/(180/M_PI)), 0,
                0, 0,                   0, 1;

        rz2 << cos(p.c/(180/M_PI)), -sin(p.c/(180/M_PI)), 0, 0,
                sin(p.c/(180/M_PI)),  cos(p.c/(180/M_PI)), 0, 0,
                0,                    0, 1, 0,
                0,                    0, 0, 1;
        r = rz1*ry*rz2;
        r(0,3) = p.x;
        r(1,3) = p.y;
        r(2,3) = p.z;
        R.push_back(r);
    }
    for(auto &p : R)
        p = p*Rt.inverse()*Ra.inverse()*Ro.inverse()*Txyz.inverse()*Tz.inverse();

    double l, l2, l3, d, h;
    double a_, a2, a3, ad, ah;
    Eigen::Vector3d y4, z4, y4af, z4af, x6, z6, z6af;
    int i = 0;
    bool lt_speed_, lt_agl_;
    for(auto p : R){
        i++;
        agl.a1 = atan2(p(0,3), p(1,3));
        l2 = fabs(550*sin(agl.a1));
        l3 = fabs(sqrt(210*210+700*700)*sin(agl.a1));
        d = p(0,3) - 165*sin(agl.a1);
        h = p(2,3)*sin(agl.a1) - 430*sin(agl.a1);
        l = sqrt(d*d+h*h);
        a_ = atan2(210, 700);
        ah = atan(h/d);
        ad = M_PI/2 - ah;
        double t1 = (l*l+l3*l3-l2*l2)/(2*l*l3);
        double t2 = (l*l+l2*l2-l3*l3)/(2*l*l2);
        if(t1>1 || t1<-1 || t2>1 ||t2<-1){
            Limit = true;
            continue;
        }
        a2 = acos(t1);
        a3 = acos(t2);
        agl.a2 = (M_PI/2 - a3 - ah)*180/M_PI;
        agl.a3 = (agl.a2 - a2 -ad - a_)*180/M_PI;
        if(agl.a2>150 || agl.a2<-90)
            Limit = true;
    }
    return Limit;
}


