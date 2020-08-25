#include "fitCorrelation.h"
#include <QStringList>
#include <nlopt.h>

#define PPSTEP 2   /// 过滤点列的间隔
#define SPAN 1     /// 10MM间隔一个点的吐点

#define PCOUNTS 50.0
vector<WELDPARAMS> wparams;

void getParams()
{
    ifstream ifs;
    ifs.open("etc/weldParams.txt");
    string line;
    while(getline(ifs,line))
    {
        QString ql = QString::fromStdString(line);
        QStringList qls = ql.split("\t");
        if(qls.size() < 7)
            continue;
        WELDPARAMS wp = {qls[0].toDouble(),qls[1].toDouble(),qls[2].toDouble()
                         ,qls[3].toDouble(),qls[4].toDouble(),qls[5].toInt(),qls[6].toDouble()};
        wparams.push_back(wp);
    }
    ifs.close();
}

/// r   base下的姿态
/// dir Base下的方向
/// angle 角度
bool RotateOatByVec(RobotPos & r,point3d<double> dir,double angle)
{
    Matrix4d mr;
    pos2Matrix(r,mr,zyz);
    double dirm = dir.norm();
    point3d<double> dir0 = {dir._x/dirm,dir._y/dirm,dir._z/dirm};
    point3d<double> z0 = {0,0,1};
    point3d<double> y_dir = dir0.cross(z0);
    double ym = y_dir.norm();
    point3d<double> y0 = {y_dir._x/ym,y_dir._y/ym,y_dir._z/ym};
    point3d<double> x0 = y0.cross(dir0);
    Matrix4d Mdir;
    Mdir<<x0._x,y0._x,dir0._x,0,
          x0._y,y0._y,dir0._y,0,
          x0._z,y0._z,dir0._z,0,
              0,    0,      0,1;
    RobotPos rz = {0,0,0,0,0,angle,UPLOAD};
    Matrix4d mrz;
    pos2Matrix(rz,mrz,zyz);
    mr = Mdir*mrz*Mdir.inverse()*mr;
    Matrix2pos(mr,r,zyz);
}

/// 曲线拟合后的取点函数 及其关键！！！
point3dd MultiSepModel::at(double t)
{
    double ss = sepModels.size();
    if(ss < 1)
    {
        return {0,0,0};
    }
    double len0 = sepModels[0].length;
    point3d<double> p0=sepModels[0].model(0),p1=sepModels[0].model(len0);
    if(t <= len0) /// t处于第一段
    {
        return sepModels[0].model(t);
    }
    else
    {
        for(int i=1;i<ss-1;i++)
        {
            double ti = 0,leni = sepModels[i].length;
            while(ti<leni)
            {
                point3d<double> pi = sepModels[i].model(ti);
                if((p0-p1).inner(p1-pi) > 0) /// 避开段落间的重叠部分
                {
                    double disi = pi.dist(p1);
                    if(t <= len0+disi+leni-ti)
                    {
                        return sepModels[i].model(ti+t-len0-disi);
                    }
                    else
                    {
                        len0 += disi+leni-ti;  /// 调整len
                        p0=pi;
                        p1=sepModels[i].model(leni);
                    }
                    break;
                }
                ti+=1;
            }
        }
    }
    if(ss >= 2)
    {
        double te = 0;
        while(te<sepModels[ss-1].length)
        {
            point3d<double> pi = sepModels[ss-1].model(te);
            if((p0-p1).inner(p1-pi) > 0) /// 避开段落间的重叠部分
            {
                return sepModels[ss-1].model(te+t-len0-pi.dist(p1));
            }
            te+=1;
        }
    }
    else
    {
        return sepModels[0].model(t);
    }
}

bool MultiSepModel::build(vector<RobotPos> data,double rank)
{
    if(data.size()< 5)
    {
        return false;
    }
    SepModel3D smd;
    double rank0 = rank<=2?rank:2;
    smd.setParams({rank0});
    vector<double> _x[3];
    for(uint t=0;t<data.size();t++)
    {
        _x[0].push_back(data[t].x);
        _x[1].push_back(data[t].y);
        _x[2].push_back(data[t].z);
    }
    smd.setRealSE(0,data.size()-1,false);
    smd.setData(_x[0],_x[1],_x[2]);
    try
    {
        smd.build();
    }catch(...)
    {
        return false;
    }
    uint rankger = round(rank);
    Config _sys_config(SYS);
    double seplen = _sys_config.get<double>("fit.partlen",50);// 拟合长度50mm
    double total_len = smd.length;
    if(total_len < 200 && rankger == 1)
    {
        SepModel3D smd_tick;
        smd_tick.setParams({rank0});
        smd_tick.setRealSE(0,data.size()-1);
        smd_tick.setData(_x[0],_x[1],_x[2]);
        try
        {
            smd_tick.build();
        }catch(...)
        {
            return false;
        }
        /// 打印踢掉的点
        vector<int> last_t;
        smd_tick.printTvec(last_t);
        lastTvec.push_back(last_t);
        cutVecs.push_back(smd_tick.cutVec);
        rms = sqrt(smd_tick.getRMS());
        distense = smd_tick.length;
        sepModels.push_back(smd_tick);
        if(rms > 2)
            return false;
    }
    else
    {
        rms = 0;
        distense = 0;
        int dsize = data.size();
        int parts = floor(smd.length/seplen);
        parts = parts>=1?parts:1;
        int points = floor((data.size()+0.0)/parts); /// 每段点的个数
        int sets = parts-2;
        sets = sets>=1?sets:1;
        int couts[sets];
        if(sets == 1)
        {
            couts[0] = dsize;
        }
        else ///parts >=4
        {
            for(int s=0;s<sets;s++)
            {
                couts[s] = points;
            }
            couts[0] += points;
            couts[sets-1] += points;
            dsize -= parts*points;
            for(int s=0;s<sets && dsize>0;s++)
            {
                couts[s]++;
                dsize--;
            }
            if(dsize>0)
            {
                couts[0]++;
            }
        }
        uint sindex[sets],eindex[sets];
        sindex[0] = 0;
        eindex[0] = couts[0]-1;
        for(int s=1;s<sets;s++)
        {
            sindex[s] = eindex[s-1]+1;
            eindex[s] = eindex[s-1]+couts[s];
        }
        for(int i=0;i<sets;i++)
        {
            SepModel3D smd;
            uint ranker = round(rank);
            switch(ranker)
            {
            case 1:
                smd.setParams({1});
                break;
            case 2 : /// 所有段落均为2次曲线
            case 6:
                smd.setParams({2});
                break;
            case 3 : /// 头部直线 其他段落2次曲线
                if(i==0)
                    smd.setParams({1});
                else
                    smd.setParams({2});
                break;
            case 4 :
                if(i==parts-1) /// 尾部直线 其他段落2次曲线
                    smd.setParams({1});
                else
                    smd.setParams({2});
                break;
            case 5:  /// 收尾直线 中间段落曲线
                if(i==0 || i==parts-1)
                    smd.setParams({1});
                else
                    smd.setParams({2});
                break;
            }
            vector<double> _x[3];
            uint ts = i>0?sindex[i-1]:sindex[i];
            uint ms = sindex[i];
            uint me = eindex[i];
            uint te = i<sets-1?eindex[i+1]:eindex[i];
            for(uint t=ts;t<te;t++)
            {
                if(t >=data.size())
                    break;
                _x[0].push_back(data[t].x);
                _x[1].push_back(data[t].y);
                _x[2].push_back(data[t].z);
            }
            smd.setRealSE(ms-ts,me-ts);
            smd.setData(_x[0],_x[1],_x[2]);
            try
            {
                //smd.needPick = true;
                smd.build();
            }catch(...)
            {
                return false;
            }
            /// 打印踢掉的点
            vector<int> last_t;
            smd.printTvec(last_t);
            vector<int> last_t_;
            for(auto t : last_t)
            {
                last_t_.push_back(t+ts);
            }
            lastTvec.push_back(last_t_);
            vector<vector<int>> cVec;
            for(auto c1:smd.cutVec)
            {
                vector<int> ivec;
                for(auto c2:c1)
                {
                    ivec.push_back(c2+ts);
                }
                cVec.push_back(ivec);
            }
            cutVecs.push_back(cVec);
            rms += smd.getRMS();
            distense += smd.length;
            if(smd.length < 0)
            {
                cout<<"bad sep!"<<endl;
                return false;
            }
            sepModels.push_back(smd);
        }
        rms = sqrt(rms/parts);
        if(rms > 2)
            return false;
    }
    return true;
}

double MultiSepModel::getRMS() const
{
    return rms;
}


string CircleModel::printCPos()
{
    return cPos.toStr();
}

double CircleModel::getLen()
{
    return distense;
}

double CircleModel::getR()
{
    return r;
}

point3d<double> CircleModel::at(double t)
{
    double xita = 180*t/(r*PI);
    RobotPos cpost = cPos;
    cpost.c += xita;
    Matrix4d m;
    pos2Matrix(cpost,m,zyz);
    Vector4d tvec = {r,0,0,1};
    Vector4d tPos = m*tvec;
    return {tPos(0,0),tPos(1,0),tPos(2,0)};
}

point3d<double> CircleModel::abc(double angle, double t)
{
    Config cali("etc/calibration.info");
    RobotPos t2c = cali.get<RobotPos>("TINC");
    point3d<double> vec_tc = {t2c.x,t2c.y,t2c.z}; /// 中心点到焊枪头的方向
    double dtc = vec_tc.norm();
    double sita = -2*asin(dtc*0.5/r);
    point3d<double> cic = {r*cos(sita),r*sin(sita),0}; ///相机中心点在曲线上的位置
    point3d<double> oic = {r,0,0}; ///焊枪头在曲线上的位置
    point3d<double> o2c = oic-cic;  /// 相机中心点到焊枪头的方向(曲线上)
    point3d<double> vec_tc0 = {vec_tc._x/dtc,vec_tc._y/dtc,vec_tc._z/dtc};
    double o2cm = o2c.norm();
    point3d<double> o2c0 = {o2c._x/o2cm,o2c._y/o2cm,o2c._z/o2cm};
    point3d<double> ab = vec_tc0.cross(o2c0);
    double xita_ab = acos(vec_tc0.inner(o2c0))*180/PI;
    RobotPos r0 = RobotPos::instance();
    RotateOatByVec(r0,ab,xita_ab);
    Vector4d vo2c0 = {o2c0._x,o2c0._y,o2c0._z,1};
    point3d<double> o2c1 = {vo2c0(0,0),vo2c0(1,0),vo2c0(2,0)};
    double b0=0;
    double xita_c = 0;
    while(abs(b0-180+angle)>0.1)
    {
        RobotPos r1 = r0;
        RotateOatByVec(r1,o2c1,xita_c);
        b0 = r1.b;
        xita_c-=0.1;
    }
    cout<<"r0:"<<r0.a<<","<<r0.b<<","<<r0.c<<","
        <<"o2c1:"<<o2c1._x<<","<<o2c1._y<<","<<o2c1._z<<","<<xita_c<<endl;
    RotateOatByVec(r0,o2c1,xita_c);
    Matrix4d mr0;
    pos2Matrix(r0,mr0,zyz);
    RobotPos cpost = cPos;
    cpost.c += t;
    Matrix4d m;
    pos2Matrix(cpost,m,zyz);
    m = m*mr0;
    RobotPos ret;
    Matrix2pos(m,ret,zyz);
    return {ret.a,ret.b,ret.c};
}


bool CircleModel::build(vector<RobotPos> data)
{
    vector<RobotPos>().swap(points);
    points = data;
    Vector3d vabc;
    int fConts = points.size();
    if(fConts <= 3)
        return false;
    MatrixXd f(fConts, 3);
    int row = 0;
    for ( auto & fpoint : points ) {
        f(row,0) = fpoint.x;
        f(row,1) = fpoint.y;
        f(row,2) = fpoint.z;
        row ++;
    }
    MatrixXd ft(3,fConts);
    ft = f.transpose();
    Matrix3d ftf = ft*f;
    FullPivLU<Matrix3d> fpu(ftf);
    if(fpu.rank() < 3)
        return false;
    Matrix3d ftfi = ftf.inverse();
    MatrixXd fp(fConts,1);
    fp.setOnes(fConts,1);
    vabc = ftfi*ft*fp;
    pabc = {vabc(0,0),vabc(1,0),vabc(2,0)};
    int size50 = fConts/4;
    RobotPos r50 = points[size50]-points[0];
    RobotPos r99 = points[2*size50]-points[size50];
    point3d<double> p50 = {r50.x,r50.y,r50.z};
    point3d<double> p99 = {r99.x,r99.y,r99.z};
    point3d<double> p00 = p50.cross(p99);
    double dp = p00.inner(pabc);
    double abcm = sqrt(pabc.inner(pabc));
    point3d<double> pdrt = {pabc._x/abcm,pabc._y/abcm,pabc._z/abcm};
    if(dp < 0)
    {
        pdrt._x = -pdrt._x;
        pdrt._y = -pdrt._y;
        pdrt._z = -pdrt._z;
    }
    vector<RobotPos>().swap(ty_points);
    ty_points = points;
    shadow(pabc, ty_points);
    MatrixXd f2(fConts-size50+1, 3);
    MatrixXd fp2(fConts-size50+1,1);
    int i=0;
    for(;i<fConts-size50;i++)
    {
        f2(i,0) = ty_points[size50+i].x-ty_points[i].x;
        f2(i,1) = ty_points[size50+i].y-ty_points[i].y;
        f2(i,2) = ty_points[size50+i].z-ty_points[i].z;
        fp2(i,0) = (pow(ty_points[size50+i].x,2) +pow(ty_points[size50+i].y,2)+pow(ty_points[size50+i].z,2)
                -pow(ty_points[i].x,2)-pow(ty_points[i].y,2)-pow(ty_points[i].z,2))/2;
    }
    f2(i,0) = pabc._x;
    f2(i,1) = pabc._y;
    f2(i,2) = pabc._z;
    fp2(i,0) = 1;
    auto ft2 = f2.transpose();
    Matrix3d ftf2 = ft2*f2;
    FullPivLU<Matrix3d> fpu2(ftf2);
    if(fpu2.rank() < 3)
        return false;
    Matrix3d ftfi2 = ftf2.inverse();
    auto cxyz = ftfi2*ft2*fp2;
    cPos.x = cxyz(0,0);
    cPos.y = cxyz(1,0);
    cPos.z = cxyz(2,0);
    double check = cPos.x*pabc._x+cPos.y*pabc._y+cPos.z*pabc._z;
    double sumdis = 0;
    for(auto p : ty_points)
    {
        sumdis += p.dis(cPos);
    }
    r = sumdis/ty_points.size();
    point3d<double> pdx0 = {ty_points[0].x - cPos.x,ty_points[0].y - cPos.y,ty_points[0].z - cPos.z};
    double dx0m = sqrt(pdx0.inner(pdx0));
    pdx0 = {pdx0._x/dx0m,pdx0._y/dx0m,pdx0._z/dx0m};
    auto pdy = pdrt.cross(pdx0);
    double pdym = sqrt(pdy.inner(pdy));
    pdy = {pdy._x/pdym,pdy._y/pdym,pdy._z/pdym};
    auto pdx = pdy.cross(pdrt);
    double cb = acos(pdrt._z);
    double ca = atan2(pdrt._y,pdrt._x);
    double cc = atan2((-1*pdx._x*sin(ca)+pdx._y*cos(ca)),(pdy._y*cos(ca)-pdy._x*sin(ca)));//Ax-Ay/-Ox+Oy
    cPos.a = ca * 180 / M_PI;
    cPos.b = cb * 180 / M_PI;
    cPos.c = cc * 180 / M_PI;
    Matrix4d mcpos;
    pos2Matrix(cPos,mcpos,zyz);
    Matrix4d mend;
    pos2Matrix(ty_points[fConts-1],mend,zyz);
    Matrix4d mend_in_c = mcpos.inverse()*mend;
    point3d<double> pinc = {mend_in_c(0,3),mend_in_c(1,3),mend_in_c(2,3)};
    distense = acos(pinc._x/sqrt(pinc.inner(pinc)))*r;
    double rms = getRMS();
    if(rms < 0.2)
    {
        buildEnds();
        return true;
    }
    vector<RobotPos> data2;
    for(auto p : points)
    {
        if(minDis(p)<2*rms)
        {
            data2.push_back(p);
        }
    }
    if(data2.size() < data.size()/2)
    {
        return false;
    }
    else
    {
        if(data2.size() == data.size())
        {
            buildEnds();
            return true;
        }
        else
        {
            build(data2);
        }
    }
    return true;
}

/////////////////////////////////////////
UV2path::UV2path()
{
}

void UV2path::setMUV(map<int,Point2d>  v)
{
    pts = v;
}
void UV2path::setLUV(map<int,Point2d>  v)
{
    if(v.size() > 0)
        lts = v;
    else
        lts = pts;
}
void UV2path::setRUV(map<int,Point2d>  v)
{
    if(v.size() > 0)
        rts = v;
    else
        rts = pts;
}
void UV2path::setPos(map<int,RobotPos> p)
{
    rbts = p;
}

void UV2path::setSeamName(string seamName)
{
    curSeam = seamName;
}

void UV2path::setCurCad(string cadName)
{
    curCad = cadName;
}

void UV2path::setDbInfos(map<string,string> db)
{
    dbInfos = db;
}

string UV2path::getInfos(string key)
{
    return dbInfos[key];
}

void UV2path::setCurTime(string time)
{
    currTime = time;
}

int getAxisY(RobotPos r)
{
    Matrix4d mr;
    pos2Matrix(r,mr,zyz);
    Vector4d dy = {0,1,0,0};
    Vector4d ry = mr*dy;
    double y = ry(2,0);
    if(y > 0)
        return 1;
    else
        return -1;
}

void fitHeight(vector<RobotPos> & pos_vec)
{
    try
    {
        PolyModel1D pm1;
        std::deque<double> gap,dt;
        for(uint t=0;t<pos_vec.size();t++)
        {
            if(abs(pos_vec[t].dh) < 0.001)
                continue;
            gap.push_back(pos_vec[t].dh);
            dt.push_back(t);
        }
        pm1.setParams({2});
        pm1.setData(gap,dt);
        pm1.build();
        for(uint t=0;t<pos_vec.size();t++)
        {
            pos_vec[t].dh = pm1.model(t);
        }
    }
    catch(...){

    }
}

bool UV2path::get3dPath(vector<RobotPos> & pos_vec)
{
    bool ret = true;
    Config _sys_config(SYS);
    Config conf("etc/calibration.info");
    UV ct = conf.get<UV>("cameraCenterUV");
    double max_umis = _sys_config.get<double>("max_umis",300);
    double max_vmis = _sys_config.get<double>("max_vmis",100);
    ofstream downofs;/// 保存原始焊缝pos及序号
    string path = "testLog/"+curCad+"_"+curSeam+"_"+currTime;
    if ( !boost::filesystem::exists(path)) {
        boost::filesystem::create_directories(path) ;
    }
    downofs.open(path+"/origin_pos.log");
    vector<RobotPos>().swap(pos_vec);
    auto it = pts.begin();
    int pre_idx = -1;
    bool fith = false;
    while(it != pts.end()) ///遍历图像算法吐出的UV序列
    {
        auto index = it->first;
        RobotPos outPos;
        RobotPos inPos = rbts[index];
        UV uv;
        uv.u = it->second.x;
        uv.v = it->second.y;
        if(abs(uv.u-ct.u) > max_umis || abs(uv.v-ct.v) > max_vmis)
        {
            it++;
            continue;
        }
        UV2Point3D(inPos,uv,outPos);
        RobotPos hpos = outPos;
        UV uvh = {lts[index].x,lts[index].y};
        if(abs(uvh.u-ct.u) < max_umis && abs(uvh.v-ct.v) < max_vmis)
        {
            UV2Point3D(inPos,uvh,hpos);
            RobotPos mis = (!outPos)<<hpos;
            outPos.dh = mis.z;
            fith = true;
        }
        RobotPos target;
        target.x = outPos.x;
        target.y = outPos.y;
        target.z = outPos.z;
        target.a = inPos.a;
        target.b = inPos.b;
        target.c = inPos.c;
        target.dh = outPos.dh;
        target.dw = 0;
        target.tcp = UPLOAD;
        if(pre_idx>=0) /// 插入空缺的POS，值：0/线性插值
        {
            for(int i=pre_idx+1;i<index;i++)
            {
                RobotPos tari_ = RobotPos::instance();
                tari_.a = target.a;
                tari_.b = target.b;
                tari_.c = target.c;
                downofs<<i<<"\t"<<tari_.toStr()<<endl;
                pos_vec.push_back(tari_);
            }
        }
        pos_vec.push_back(target);
        pre_idx = index;
        downofs<<index<<" "<<uv.u<<" "<<uv.v<<" "<<target.toStr()<<" "<<hpos.toStr()<<" "<<target.dh<<endl;
        it++;
    }
    pts.erase(pts.begin(),pts.end());
    if(fith)
    {
        fitHeight(pos_vec);
        for(auto p : pos_vec)
        {
            downofs<<p.toStr()<<" "<<p.dh<<endl;
        }
    }
    return ret;
}

RobotPos MatrixToPos(Eigen::Matrix4d m)
{
    using namespace Eigen;
    double a, b, c;
    b = acos(m(2, 2));
    a = atan2(m(1,2),m(0,2));
    c = atan2((-1*m(0,0)*sin(a)+m(1,0)*cos(a)),(m(1,1)*cos(a)-m(0,1)*sin(a)));//Ax-Ay/-Ox+Oy
    RobotPos pos;
    pos.x = m(0, 3);
    pos.y = m(1, 3);
    pos.z = m(2, 3);
    pos.a = a * 180 / M_PI;
    pos.b = b * 180 / M_PI;
    pos.c = c * 180 / M_PI;
    return pos;
}

/////////////////////////////下方:多项式拟合应用及关联///////////////////////////////////////
double min2lineDis(MultiSepModel model0,MultiSepModel model1,double & t1,double & t2)
{
    double min = model0.at(t1).dist(model1.at(t2)); /// dis : t1 2 t2
    double step = min/10;  /// t1+-step t2+-step
    step = step<0.5?0.5:step; /// min step = 0.5mm
    point3d<double> p0[3],p1[3];
    for(int i=-1;i<=1;i++)
    {
        p0[i+1] = model0.at(t1+step*i);
        for(int j=-1;j<=1;j++)
        {
            p1[j+1] = model1.at(t2+step*j);
            if(p0[i+1].dist(p1[j+1])<min-0.001)
            {
                t1+=step*i;
                t2+=step*j;
                return min2lineDis(model0,model1,t1,t2);
            }
        }
    }
    return min;
}

bool getModelsT(MultiSepModel model0,MultiSepModel model1,double & t1,double & t2, double & mis)
{
    point3d<double> p00 = model0.at(0);
    point3d<double> p01 = model0.at(model0.distense);
    point3d<double> p10 = model1.at(0);
    point3d<double> p11 = model1.at(model1.distense);
    double d00 = p00.dist(p10);
    double d01 = p00.dist(p11);
    double d10 = p01.dist(p10);
    double d11 = p01.dist(p11);
    double min=d00;
    min = min>d01?d01:min;
    min = min>d10?d10:min;
    min = min>d11?d11:min;
    t1 = 0;
    t2 = 0;
    if(abs(min-d01) < 0.01)
    {
        t2 = model1.distense;
    }
    else if(abs(min-d10) < 0.01)
    {
        t1 = model0.distense;
    }
    else if(abs(min-d11) < 0.01)
    {
        t1 = model0.distense;
        t2 = model1.distense;
    }
    mis = min2lineDis(model0,model1,t1,t2);
    return true;
}

double min2lineDis(SepModel3D model0,SepModel3D model1,double & t1,double & t2)
{
    double min = model0.model(t1).dist(model1.model(t2)); /// dis : t1 2 t2
    double step = min/10;  /// t1+-step t2+-step
    step = step<0.5?0.5:step; /// min step = 0.5mm
    point3d<double> p0[3],p1[3];
    for(int i=-1;i<=1;i++)
    {
        p0[i] = model0.model(t1+step*i);
        for(int j=-1;j<=1;j++)
        {
            p1[j] = model1.model(t2+step*j);
            if(p0[i].dist(p1[j])<min-0.001)
            {
                t1+=step*i;
                t2+=step*j;
                return min2lineDis(model0,model1,t1,t2);
            }
        }
    }
    return min;
}

bool getModelsT(SepModel3D model0,SepModel3D model1,double & t1,double & t2, double & mis)
{
    point3d<double> p00 = model0.model(0);
    point3d<double> p01 = model0.model(model0.length);
    point3d<double> p10 = model1.model(0);
    point3d<double> p11 = model1.model(model1.length);
    double d00 = p00.dist(p10);
    double d01 = p00.dist(p11);
    double d10 = p01.dist(p10);
    double d11 = p01.dist(p11);
    double min=d00;
    min = min>d01?d01:min;
    min = min>d10?d10:min;
    min = min>d11?d11:min;
    if(abs(min-d01) < 0.01)
    {
        t2 = model1.length;
    }
    else if(abs(min-d10) < 0.01)
    {
        t1 = model0.length;
    }
    else if(abs(min-d11) < 0.01)
    {
        t1 = model0.length;
        t2 = model1.length;
    }
    mis = min2lineDis(model0,model1,t1,t2);
    return true;
}

bool fitSeamPath(string path, vector<RobotPos> & seam0,int rank0)
{
    ofstream ofs;
    time_t tt = time(NULL);
    tm * lt = localtime(&tt);
    char timechar[128];
    ::sprintf(timechar,"%s/fit_%02d%02d_%02d%02d%02d.log",path.c_str(),
              lt->tm_mon+1,lt->tm_mday,lt->tm_hour,lt->tm_min,lt->tm_sec);
    ofs.open(timechar);
    ofs<<"seam0原始点:"<<endl;
    for(auto p : seam0){ofs<<p.toStr()<<"\t"<<p.dh<<"\t"<<p.dw<<endl;}
    ofs<<"rank:"<<rank0<<endl;
    ///原始点数太少
    if(seam0.size() < 5)
    {
        ofs << "seam0.size="<<seam0.size()<<endl;
        ofs.close();
        return false;
    }
    if(rank0 <= 5)
    {
       Vector3d abc;
       if(points2face(seam0,abc))
       {
           point3dd pabc = {abc(0,0),abc(1,0),abc(2,0)};
           shadow(pabc,seam0);
       }
    }
    /// seam0的xyz曲线模型fitSeamPath
    MultiSepModel model0;
    if(!model0.build(seam0,rank0))
    {
        double rms = model0.getRMS();
        ofs<<"model0 RMS:"<<rms<<endl;
        ofs << "seam0多项式曲线拟合失败!"<<endl;
        ofs.close();
        return false;
    }
    double rms = model0.getRMS();
    ofs<<"model0 RMS:"<<rms<<endl;
    ofs<<"seam0有效原始点:"<<endl;
    for(uint t=0;t<model0.lastTvec.size();t++){
        ofs<<"第"<<t<<"段"<<endl;
        for(uint m=0;m<model0.lastTvec[t].size();m++)
        {
            auto p = seam0[model0.lastTvec[t][m]];
            ofs<<model0.lastTvec[t][m]<<"\t"<<p.x<<"\t"<<p.y<<"\t"<<p.z<<endl;
        }
    }
    ofs<<"seam0踢点次序:"<<endl;
    for(uint p=0;p<model0.cutVecs.size();p++)
    {
        vector<vector<int>> cutVec = model0.cutVecs[p];
        ofs<<"seam0第"<<p<<"段"<<endl;
        for(uint t=0;t<cutVec.size();t++)
        {
            ofs<<"第"<<t<<"遍"<<endl;
            vector<int> tVec = cutVec[t];
            for(auto i : tVec)
            {
                ofs<<i<<"\t";
            }
            ofs<<endl;
        }
    }
    /// 评价原始点
    if(seam0.size()<0.05*model0.distense)
    {
        /// 原始点太稀疏
        ofs << "原始点太稀疏!"<<endl;
        ofs.close();
        return false;
    }
    /// seam0平滑后的point3d点列
    vector<point3d<double>> seam_p3d;
    double rlen = model0.distense;
    int count = round(rlen/SPAN);
    double span =rlen/count;
    cout<<"吐点:"<<endl;
    for(int i=0;i<=count;i++)
    {
        cout<<"第"<<i<<"行:";
        seam_p3d.push_back(model0.at(span*i));
    }
    /// point3d<double> 转 RobotPos
    if(seam_p3d.size() < 3)
    {
        ofs<<"seam_p3d.size() < 3"<<endl;
        ofs.close();
        return false;
    }
    vector<RobotPos> seam0_;
    /// 原焊缝第0个点
    int t0 = 0;
    point3d<double> pre{seam0[t0].x,seam0[t0].y,seam0[t0].z};
    for(auto p : seam_p3d)
    {
        double mindis = 9999;
        int tmin=t0;
        for(uint t=t0;t<=seam0.size()-1;t+=1)
        {
            point3d<double> ds{seam0[t].x,seam0[t].y,seam0[t].z};
            if(ds.dist(p) < mindis)
            {
                tmin = t;
                mindis = ds.dist(p);
            }
        }
        t0 = tmin;
        pre = {seam0[t0].x,seam0[t0].y,seam0[t0].z};
        RobotPos r{p._x,p._y,p._z,seam0[t0].a,seam0[t0].b,seam0[t0].c,UPLOAD,seam0[t0].dh,seam0[t0].dw,seam0[t0].v,seam0[t0].weld};
        seam0_.push_back(r);
    }
    vector<point3d<double>>().swap(seam_p3d);
    vector<RobotPos>().swap(seam0);
    seam0.insert(seam0.end(),seam0_.begin(),seam0_.end());
    vector<RobotPos>().swap(seam0_);
    ofs<<"吐出的seam0:"<<endl;
    for(auto p : seam0)
    {
        ofs<<p.x<<"\t"<<p.y<<"\t"<<p.z<<"\t"<<p.a<<"\t"<<p.b<<"\t"<<p.c<<endl;
    }
    ofs.close();
    return true;
}

void getPathOffsetByKeel(MultiSepModel km,MultiSepModel rm,vector<RobotPos> seam0, RobotPos & offset)
{
//    nlopt_opt opt = nlopt_create(NLOPT_LN_SBPLX, 1);
//    nlopt_set_min_objective(opt, minTdis, this);
//    nlopt_set_stopval(opt, 1e-1);
//    nlopt_set_ftol_rel(opt, 1e-1);
//    nlopt_set_xtol_rel(opt, 1e-3);
//    curT = t0;
//    double x[1],mis;
//    x[0] = _t[t0];
//    nlopt_result result= nlopt_optimize(opt, x, &mis);
//    if(!result)
//    {
//        return mis;
//    }
//    ti = x[0];
//    return mis;
}

bool correlationByKeel(string logPath,vector<RobotPos> keel, double rank,
                       vector<RobotPos> & seam0,vector<RobotPos> relate1,double rank1)
{
    ofstream ofs;
    time_t tt = time(NULL);
    tm * lt = localtime(&tt);
    char timechar[256];
    ::sprintf(timechar,"%s/correlation_%02d%02d_%02d%02d%02d.log",logPath.c_str(),
              lt->tm_mon+1,lt->tm_mday,lt->tm_hour,lt->tm_min,lt->tm_sec);
    ofs.open(timechar);
    ofs<<"keel:"<<endl;
    for(auto p : keel){ofs<<p.toStr()<<"\t"<<p.dh<<"\t"<<p.dw<<endl;}
    ofs<<"seam0原始点:"<<endl;
    for(auto p : seam0){ofs<<p.toStr()<<"\t"<<p.dh<<"\t"<<p.dw<<endl;}
    ofs<<"relate1原始点:"<<endl;
    for(auto p : relate1){ofs<<p.toStr()<<"\t"<<p.dh<<"\t"<<p.dw<<endl;}
    ofs<<"rank0:"<<rank<<endl;
    ///龙骨点数太少
    if(keel.size() < 5)
    {
        ofs << "seam0.size="<<keel.size()<<endl;
        ofs.close();
        return false;
    }
    /// seam0的xyz曲线模型
    MultiSepModel model0;
    if(!model0.build(keel,rank))
    {
        double rms = model0.getRMS();
        ofs<<"model0 RMS:"<<rms<<endl;
        ofs << "seam0多项式曲线拟合失败!"<<endl;
        ofs.close();
        return false;
    }
    /// relate1的xyz曲线模型
    MultiSepModel model1;
    if(!model1.build(relate1,rank1))
    {
        double rms = model1.getRMS();
        ofs<<"model1 RMS:"<<rms<<endl;
        ofs << "seam1多项式曲线拟合失败!"<<endl;
        ofs.close();
        return false;
    }
    RobotPos offset;
    getPathOffsetByKeel(model0,model1,seam0,offset);
}

bool correlationBySep(string logPath, vector<RobotPos> & seam0, int rank0, RelateInfo rif)
{
    ofstream ofs;
    time_t tt = time(NULL);
    tm * lt = localtime(&tt);
    char timechar[256];
    ::sprintf(timechar,"%s/correlation_%02d%02d_%02d%02d%02d.log",logPath.c_str(),
              lt->tm_mon+1,lt->tm_mday,lt->tm_hour,lt->tm_min,lt->tm_sec);
    ofs.open(timechar);
    ofs<<"seam0原始点:"<<endl;
    for(auto p : seam0){ofs<<p.toStr()<<"\t"<<p.dh<<"\t"<<p.dw<<endl;}
    ofs<<"seam1原始点:"<<endl;
    for(auto p : rif.beginPath){ofs<<p.toStr()<<"\t"<<p.dh<<"\t"<<p.dw<<endl;}
    ofs<<"seam2原始点:"<<endl;
    for(auto p : rif.endPath){ofs<<p.toStr()<<"\t"<<p.dh<<"\t"<<p.dw<<endl;}
    ofs<<"rank0:"<<rank0<<"rank1:"<<rif.beginRank<<"rank2:"<<rif.endRank<<endl;
    ///原始点数太少
    if(seam0.size() < 5)
    {
        ofs << "seam0.size="<<seam0.size()<<endl;
        ofs.close();
        return false;
    }
    /// seam0的xyz曲线模型
    MultiSepModel model0;
    if(!model0.build(seam0,rank0))
    {
        double rms = model0.getRMS();
        ofs<<"model0 RMS:"<<rms<<endl;
        ofs << "seam0多项式曲线拟合失败!"<<endl;
        ofs.close();
        return false;
    }
    double rms = model0.getRMS();
    ofs<<"model0 RMS:"<<rms<<endl;
    ofs<<"seam0有效原始点:"<<endl;
    for(int t=0;t<model0.lastTvec.size();t++){
        ofs<<"第"<<t<<"段"<<endl;
        for(int m=0;m<model0.lastTvec[t].size();m++)
        {
            auto p = seam0[model0.lastTvec[t][m]];
            ofs<<model0.lastTvec[t][m]<<"\t"<<p.x<<"\t"<<p.y<<"\t"<<p.z<<endl;
        }
    }
    ofs<<"seam0踢点次序:"<<endl;
    for(int p=0;p<model0.cutVecs.size();p++)
    {
        vector<vector<int>> cutVec = model0.cutVecs[p];
        ofs<<"seam0第"<<p<<"段"<<endl;
        for(int t=0;t<cutVec.size();t++)
        {
            ofs<<"第"<<t<<"遍"<<endl;
            vector<int> tVec = cutVec[t];
            for(auto i : tVec)
            {
                ofs<<i<<"\t";
            }
            ofs<<endl;
        }
    }
    /// 评价原始点
    if(seam0.size()<0.05*model0.distense)
    {
        /// 原始点太稀疏
        ofs << "原始点太稀疏!"<<endl;
        ofs.close();
        return false;
    }
    /// seam1的xyz曲线模型
    MultiSepModel model1;
    if(rif.beginPath.size()  < 3)
    {
        ofs << "seam1.size="<<rif.beginPath.size()<<endl;
        vector<RobotPos>().swap(rif.beginPath);
    }
    if(rif.endPath.size()  < 3)
    {
        ofs << "seam2.size="<<rif.endPath.size()<<endl;
        vector<RobotPos>().swap(rif.endPath);
    }
    if(rif.beginRank > 0 && rif.beginPath.size() == 0)
    {
        ofs<<"关联失败,seam1缺失!"<<endl;
        ofs.close();
        return false;
    }
    if(rif.endRank > 0 && rif.endPath.size() == 0)
    {
        ofs<<"关联失败,seam2缺失!"<<endl;
        ofs.close();
        return false;
    }
    if(rif.beginPath.size() > 0)
    {
        if(!model1.build(rif.beginPath,rif.beginRank))
        {
            double rms = model1.getRMS();
            ofs<<"model1 RMS:"<<rms<<endl;
            ofs << "seam1多项式曲线拟合失败!"<<endl;
            vector<RobotPos>().swap(rif.beginPath);
            return false;
        }
        double rms1 = model1.getRMS();
        ofs<<"model1 RMS:"<<rms1<<endl;
        ofs<<"seam1有效原始点:"<<endl;
        for(int t=0;t<model1.lastTvec.size();t++){
            ofs<<"第"<<t<<"段"<<endl;
            for(int m=0;m<model1.lastTvec[t].size();m++)
            {
                auto p = rif.beginPath[model1.lastTvec[t][m]];
                ofs<<model1.lastTvec[t][m]<<"\t"<<p.x<<"\t"<<p.y<<"\t"<<p.z<<endl;
            }
        }
        ofs<<"seam0踢点次序:"<<endl;
        for(int p=0;p<model1.cutVecs.size();p++)
        {
            vector<vector<int>> cutVec = model1.cutVecs[p];
            ofs<<"seam0第"<<p<<"段"<<endl;
            for(int t=0;t<cutVec.size();t++)
            {
                ofs<<"第"<<t<<"遍"<<endl;
                vector<int> tVec = cutVec[t];
                for(auto i : tVec)
                {
                    ofs<<i<<"\t";
                }
                ofs<<endl;
            }
        }
    }
    /// seam2的xyz曲线模型
    MultiSepModel model2;
    if(rif.endPath.size() > 0)
    {
        if(!model2.build(rif.endPath,rif.endRank))
        {
            double rms = model2.getRMS();
            ofs<<"model2 RMS:"<<rms<<endl;
            ofs << "seam2多项式曲线拟合失败!"<<endl;
            vector<RobotPos>().swap(rif.endPath);
            return false;
        }
        double rms2 = model2.getRMS();
        ofs<<"model2 RMS:"<<rms2<<endl;
        ofs<<"seam2有效原始点:"<<endl;
        for(uint t=0;t<model2.lastTvec.size();t++){
            ofs<<"第"<<t<<"段"<<endl;
            for(uint m=0;m<model2.lastTvec[t].size();m++)
            {
                auto p = rif.endPath[model2.lastTvec[t][m]];
                ofs<<model2.lastTvec[t][m]<<"\t"<<p.x<<"\t"<<p.y<<"\t"<<p.z<<endl;
            }
        }
        ofs<<"seam0踢点次序:"<<endl;
        for(uint p=0;p<model2.cutVecs.size();p++)
        {
            vector<vector<int>> cutVec = model2.cutVecs[p];
            ofs<<"seam0第"<<p<<"段"<<endl;
            for(uint t=0;t<cutVec.size();t++)
            {
                ofs<<"第"<<t<<"遍"<<endl;
                vector<int> tVec = cutVec[t];
                for(auto i : tVec)
                {
                    ofs<<i<<"\t";
                }
                ofs<<endl;
            }
        }
    }
    double seam0_t0,seam0_t1,seam1_t0,seam2_t0;
    if(rif.beginPath.size() > 0)  /// seam0关联seam1
    {
        double mis;
        if(!getModelsT(model0,model1,seam0_t0,seam1_t0,mis) || (rif.beginDistance >=0 && mis > 5+rif.beginDistance))
        {
            point3d<double> mt0 = model0.at(seam0_t0);
            point3d<double> mt1 = model1.at(seam1_t0);
            ofs<<"关联失败!"<<endl;
            ofs<<"seam0<->seam1:"<<mis<<endl;
            ofs<<"model0(t0):"<<mt0._x<<","<<mt0._y<<","<<mt0._z<<endl;
            ofs<<"model1(t1):"<<mt1._x<<","<<mt1._y<<","<<mt1._z<<endl;
            vector<RobotPos>().swap(rif.beginPath);
            return false;
        }
        point3d<double> mt0 = model0.at(seam0_t0);
        point3d<double> mt1 = model1.at(seam1_t0);
        ofs<<"model0(t0):"<<mt0._x<<","<<mt0._y<<","<<mt0._z<<endl;
        ofs<<"model1(t1):"<<mt1._x<<","<<mt1._y<<","<<mt1._z<<endl;
        ofs<<"seam0<->seam1:"<<mis<<endl;
    }
    if(rif.endPath.size() > 0)
    {
        double mis;
        if(!getModelsT(model0,model2,seam0_t1,seam2_t0,mis) || (rif.endDistance >=0 && mis > 5+rif.endDistance))
        {
            point3d<double> mt0 = model0.at(seam0_t1);
            point3d<double> mt2 = model2.at(seam2_t0);
            ofs<<"关联失败!"<<endl;
            ofs<<"seam0<->seam2:"<<mis<<endl;
            ofs<<"model0(t0):"<<mt0._x<<","<<mt0._y<<","<<mt0._z<<endl;
            ofs<<"model2(t2):"<<mt2._x<<","<<mt2._y<<","<<mt2._z<<endl;
            vector<RobotPos>().swap(rif.endPath);
            return false;
        }
        point3d<double> mt0 = model0.at(seam0_t1);
        point3d<double> mt2 = model2.at(seam2_t0);
        ofs<<"model0(t0):"<<mt0._x<<","<<mt0._y<<","<<mt0._z<<endl;
        ofs<<"model2(t2):"<<mt2._x<<","<<mt2._y<<","<<mt2._z<<endl;
        ofs<<"seam0<->seam2:"<<mis<<endl;
    }
    //    double srate = _cad_config.get<double>(name+".srate",0);
    //    double erate = _cad_config.get<double>(name+".erate",0);
    double srate = 0;
    double erate = 0;
    double L1 = model0.distense*srate;
    double L2 = model0.distense*erate;
    /// seam0平滑后的point3d点列
    vector<point3d<double>> seam_p3d;
    if(rif.beginPath.size() == 0 && rif.endPath.size() == 0)
    {
        ofs<<"两端均无任何限制"<<endl;
        double rlen = model0.distense+L1+L2+rif.beginExtern+rif.endExtern;
        int count = round(rlen/SPAN);
        double span =rlen/count;
        cout<<"吐点:"<<endl;
        for(int i=0;i<=count;i++)
        {
            cout<<"第"<<i<<"行:";
            seam_p3d.push_back(model0.at(span*i-rif.beginExtern-L1));
        }
    }
    else if(rif.beginPath.size() != 0 && rif.endPath.size() == 0)
    {
        ofs<<"只关联seam1"<<endl;
        point3d<double> p1 = model1.at(seam1_t0);
        point3d<double> ps = model0.at(0);
        point3d<double> pe = model0.at(model0.distense);
        double diss = ps.dist(p1);
        double dise = pe.dist(p1);
        if(abs(rif.L) < 1)
        {
            ofs<<"一端焊缝关联,另一端无限制"<<endl;
            if(diss<dise)
            {
                ofs<<"靠近0"<<endl;
                double len = model0.distense-(seam0_t0-rif.beginExtern)+L2;
                int count = round(len/SPAN);
                double span_ = len/count;
                for(int i=0;i<=count;i++)
                {
                    seam_p3d.push_back(model0.at(seam0_t0-rif.beginExtern+i*span_));
                }
            }
            else
            {
                ofs<<"靠近model0.next"<<endl;
                double len = seam0_t0+rif.beginExtern+L1;
                int count = round(len/SPAN);
                double span_ = len/count;
                for(int i=0;i<=count;i++)
                {
                    if(rif.endPath.size()  < 3)
                    {
                        ofs << "seam2.size="<<rif.endPath.size()<<endl;
                        vector<RobotPos>().swap(rif.endPath);
                    }
                    seam_p3d.push_back(model0.at(-rif.beginExtern-L1+i*span_));
                }
            }
        }
        else
        {
            ofs<<"一端焊缝关联,另一端长度限制"<<endl;
            int counts = round(rif.L/SPAN);
            double span_ = rif.L/counts;
            if(diss<dise)
            {
                ofs<<"靠近0"<<endl;
                for(int i=0;i<=counts;i++)
                {
                    seam_p3d.push_back(model0.at(seam0_t0-rif.beginExtern+i*span_));
                }
            }
            else
            {
                ofs<<"靠近next"<<endl;
                for(int i=counts;i>=0;i--)
                {
                    seam_p3d.push_back(model0.at(seam0_t0+rif.beginExtern-i*span_));
                }
            }
        }
    }
    if(rif.endPath.size()  < 3)
    {
        ofs << "seam2.size="<<rif.endPath.size()<<endl;
        vector<RobotPos>().swap(rif.endPath);
    }
    else if(rif.endPath.size() != 0 && rif.beginPath.size() == 0)
    {
        ofs<<"只关联seam2"<<endl;
        point3d<double> p2 = model2.at(seam2_t0);
        point3d<double> ps = model0.at(0);
        point3d<double> pe = model0.at(model0.distense);
        double diss = ps.dist(p2);
        double dise = pe.dist(p2);
        if(abs(rif.L) < 1)
        {
            ofs<<"一端焊缝关联,另一端无限制"<<endl;
            if(diss<dise)
            {
                ofs<<"靠近0"<<endl;
                double len = model0.distense-(seam0_t1-rif.beginExtern);
                int count = round(len/SPAN);
                double span_ = len/count;
                for(int i=0;i<=count;i++)
                {
                    seam_p3d.push_back(model0.at(seam0_t1-rif.beginExtern+i*span_));
                }
            }
            else
            {
                ofs<<"靠近model0.next"<<endl;
                double len = seam0_t1+rif.beginExtern;
                int count = round(len/SPAN);
                double span_ = len/count;
                for(int i=0;i<=count;i++)
                {
                    seam_p3d.push_back(model0.at(i*span_));
                }
            }
        }
        else
        {
            ofs<<"一端焊缝关联,另一端长度限制"<<endl;
            int counts = round(rif.L/SPAN);
            double span_ = rif.L/counts;
            if(diss<dise)
            {
                ofs<<"靠近0"<<endl;
                for(int i=0;i<=counts;i++)
                {
                    seam_p3d.push_back(model0.at(seam0_t1-rif.beginExtern+i*span_));
                }
            }
            else
            {
                ofs<<"靠近next"<<endl;
                for(int i=counts;i>=0;i--)
                {
                    seam_p3d.push_back(model0.at(seam0_t1+rif.beginExtern-i*span_));
                }
            }
        }
    }else{
        ofs<<"两端均焊缝关联\r\n"<<endl;
        double len = abs(seam0_t0-seam0_t1)+rif.beginExtern+rif.endExtern;
        int counts = round(len/SPAN);
        double span_ = len/counts;
        double t_ = seam0_t0<seam0_t1?seam0_t0-rif.beginExtern:seam0_t1-rif.endExtern;
        for(int i=0;i<=counts;i++)
        {
            seam_p3d.push_back(model0.at(t_+i*span_));
        }
    }
    /// point3d<double> 转 RobotPos
    if(seam_p3d.size() < 3)
    {
        ofs<<"seam_p3d.size() < 3"<<endl;
        ofs.close();
        return false;
    }
    vector<RobotPos> seam0_;
    /// 原焊缝第0个点
    int t0 = 0;
    point3d<double> pre{seam0[t0].x,seam0[t0].y,seam0[t0].z};
    for(auto p : seam_p3d)
    {
        double mindis = 9999;
        int tmin=t0;
        for(uint t=t0;t<=seam0.size()-1;t+=1)
        {
            point3d<double> ds{seam0[t].x,seam0[t].y,seam0[t].z};
            if(ds.dist(p) < mindis)
            {
                tmin = t;
                mindis = ds.dist(p);
            }
        }
        t0 = tmin;
        pre = {seam0[t0].x,seam0[t0].y,seam0[t0].z};
        RobotPos r{p._x,p._y,p._z,seam0[t0].a,seam0[t0].b,seam0[t0].c,UPLOAD,seam0[t0].dh,seam0[t0].dw,seam0[t0].v,seam0[t0].weld};
        seam0_.push_back(r);
    }
    vector<point3d<double>>().swap(seam_p3d);
    vector<RobotPos>().swap(seam0);
    seam0.insert(seam0.end(),seam0_.begin(),seam0_.end());
    vector<RobotPos>().swap(seam0_);
    ofs<<"吐出的seam0:"<<endl;
    for(auto p : seam0)
    {
        ofs<<p.toStr()<<endl;
    }
    ofs.close();
    return true;
}

double CircleModel::getRMS() const
{
    double rms = 0;
    for(auto p :points)
    {
        rms += pow(minDis(p),2);
    }
    rms = rms/points.size();
    rms = sqrt(rms);
    return rms;
}

double CircleModel::getMis()
{
    double mis = 0;
    for(auto p :points)
    {
        mis += pow(minDis(p),2);
    }
    mis = sqrt(mis);
    return mis;
}

void CircleModel::buildEnds()
{
    vector<int> delVec;
    vector<int> godVec;
    for(uint j=0;j<points.size();j++)
    {
        int gs = godVec.size();
        int si = gs>0?godVec[gs-1]:-1;
        for(uint i=si+1;i<init_points.size();i++)
        {
            point3d<double> pj={points[j].x,points[j].y,points[j].z};
            point3d<double> pi={init_points[i].x,init_points[i].y,init_points[i].z};
            if(pi.dist(pj) < 0.01)
            {
                godVec.push_back(i);
                break;
            }
            else
            {
                delVec.push_back(i);
            }
        }
    }
    lastTvec.push_back(godVec);
    vector<vector<int>> del;
    del.push_back(delVec);
    cutVecs.push_back(del);
}
void CircleModel::paramInit(const double* x)
{
    cPos.x = x[0];
    cPos.y = x[1];
    cPos.z = (1-x[0]*pabc._x-x[1]*pabc._y)/pabc._z;
    r = x[2];
}
double CircleModel::minDis(RobotPos p) const
{
    point3d<double> cp = {p.x,p.y,p.z};
    point3d<double> ct = {cPos.x,cPos.y,cPos.z};
    return cp.dist(ct)-r;
}

/////////////////////////////下方:关联评价函数///////////////////////////////////////
point3d<double> getDir(RobotPos target, MultiSepModel msm, double & t)
{
    point3d<double> tar =  {target.x,target.y,target.z};
    point3d<double> pc =  msm.at(t);
    double mis = pc.dist(tar);
    double step = mis/5;
    step = step>1?step:1;
    point3d<double> pc1 =  msm.at(t+step);
    if(pc1.dist(tar) < mis-0.01)
    {
        t += step;
        return getDir(target,msm,t);
    }
    point3d<double> pc2 =  msm.at(t-step);
    if(pc2.dist(tar) < mis-0.01)
    {
        t -= step;
        return getDir(target,msm,t);
    }
    return msm.at(t+0.1)-msm.at(t-0.1);
}

RobotPos getModelP(RobotPos target, MultiSepModel msm, double t)
{
    point3d<double> tar =  {target.x,target.y,target.z};
    point3d<double> pc =  msm.at(t);
    double mis = pc.dist(tar);
    double step = mis/5;
    step = step>0.5?step:0.5;
    point3d<double> pc1 =  msm.at(t+step);
    if(pc1.dist(tar) < mis-0.01)
    {
        t += step;
        return getModelP(target,msm,t);
    }
    point3d<double> pc2 =  msm.at(t-step);
    if(pc2.dist(tar) < mis-0.01)
    {
        t -= step;
        return getModelP(target,msm,t);
    }
    auto mct = msm.at(t);
    RobotPos rmct = target;
    rmct.x = mct._x;
    rmct.y = mct._y;
    rmct.z = mct._z;
    return rmct;
}

void Euler_Adjustment2(RobotPos &p, RobotPos p2, point3d<double> dir, double rx, double ry, double dy, double dz)
{
    double pi = M_PI;
    Eigen::Matrix4d mrx;
    double a1 = rx*M_PI/180;
    mrx << 1,   0,       0,        0,
            0,   cos(a1), -sin(a1), 0,
            0,   sin(a1), cos(a1),  0,
            0,   0,       0,        1;
    Eigen::Matrix4d mry;
    double a2 = ry*M_PI/180-M_PI/2;
    mry << cos(a2),  0,   sin(a2),  0,
            0,        1,   0,        0,
            -sin(a2), 0,   cos(a2),  0,
            0,        0,   0,        1;
    Eigen::Matrix4d mp;
    pos2Matrix(p,mp,zyz);
    mp(0,3) = 0;
    mp(1,3) = 0;
    mp(2,3) = 0;
    Eigen::Vector3d x_ = mp.block(0,0,3,1);   /// p x_axis in base
    Eigen::Vector3d z_ = mp.block(0,2,3,1);   /// p z_axis in base
    Eigen::Vector3d vx = {dir._x,dir._y,dir._z};   /// seam x_axis
    bool fx = true;
    if(x_.dot(vx) < 0)
    {
        fx = false;
        vx = -1*vx;
    }
    vx = vx/sqrt(vx.dot(vx));
    Eigen::Vector3d vz0 = {0,0,-1.0};
    Eigen::Vector3d vy = vz0.cross(vx);
    Eigen::Vector3d vz = vx.cross(vy);
    Eigen::Matrix4d mt4;
    mt4<<vx(0,0),vy(0,0),vz(0,0),0,
            vx(1,0),vy(1,0),vz(1,0),0,
            vx(2,0),vy(2,0),vz(2,0),0,
            0,      0,      0,1;     /// tool in base
    Eigen::Vector3d z_t = mt4.inverse().block(0,0,3,3)*z_;   /// p z_axis in t
    Matrix4d mt4rxry;
    if(z_t(1,0) > 0)
        mt4rxry = mt4*mrx.inverse();
    else
        mt4rxry = mt4*mrx;
    if(fx)
        mt4rxry = mt4rxry*mry;
    else
        mt4rxry = mt4rxry*mry.inverse();
    mt4rxry(0,3) = p.x;
    mt4rxry(1,3) = p.y;
    mt4rxry(2,3) = p.z;
    RobotPos dyz = {0,dy,dz,0,0,0};
    Matrix4d mdyz;
    pos2Matrix(dyz,mdyz,zyz);
    mt4rxry = mt4rxry*mdyz;  /// change oat

    Eigen::Matrix4d Rz1, Ry, Rz2, r;
    Rz1 << cos(p2.a/(180/pi)), -sin(p2.a/(180/pi)), 0, 0,
            sin(p2.a/(180/pi)),  cos(p2.a/(180/pi)), 0, 0,
            0,                   0, 1, 0,
            0,                   0, 0, 1;

    Ry <<  cos(p2.b/(180/pi)),  0, sin(p2.b/(180/pi)), 0,
            0,  1,                  0, 0,
            -sin(p2.b/(180/pi)),  0, cos(p2.b/(180/pi)), 0,
            0,  0,                  0, 1;

    Rz2 << cos(p2.c/(180/pi)), -sin(p2.c/(180/pi)), 0, 0,
            sin(p2.c/(180/pi)),  cos(p2.c/(180/pi)), 0, 0,
            0,                   0, 1, 0,
            0,                   0, 0, 1;
    r = Rz1*Ry*Rz2;

    Eigen::Matrix4d ri, rj, rx_;
    Eigen::Vector3d z1, z2, z, zi, xi, yi;
    double n1, n2;
    z1 << r(0,2), r(1,2), r(2,2);
    z2 << mt4rxry(0,2), mt4rxry(1,2), mt4rxry(2,2);
    z = z1.cross(z2);
    z << z(0)/sqrt(z.dot(z)), z(1)/sqrt(z.dot(z)), z(2)/sqrt(z.dot(z));
    double t = z1.dot(z2)/(sqrt(z1.dot(z1))*sqrt(z2.dot(z2)));
    if(t >= 1){
        t = 1;
        z = {0, 0, 1};   //zèz1涓è界哥­
        z = z.cross(z1);
    }
    double a = acos(t);
    zi << 0, 0, 1;
    xi << z(0), z(1), 0;
    xi << xi(0)/sqrt(xi.dot(xi)), xi(1)/sqrt(xi.dot(xi)), 0;
    yi = zi.cross(xi);
    xi = z;
    zi = xi.cross(yi);
    ri << xi(0), yi(0), zi(0), 0,
            xi(1), yi(1), zi(1), 0,
            xi(2), yi(2), zi(2), 0,
            0,     0,     0, 1;

    rx_ << 1,      0,       0,  0,
            0, cos(a), -sin(a),  0,
            0, sin(a),  cos(a),  0,
            0,      0,       0,  1;

    r = ri*rx_*ri.inverse()*r;
    r(0,3) = mt4rxry(0,3);
    r(1,3) = mt4rxry(1,3);
    r(2,3) = mt4rxry(2,3);
    p = MatrixToPos(r);
}

void Euler_Adjustment3(RobotPos p, point3d<double> dir)
{
    double pi = M_PI;
    Eigen::Matrix4d mp;
    pos2Matrix(p,mp,zyz);
    mp(0,3) = 0;
    mp(1,3) = 0;
    mp(2,3) = 0;
    Eigen::Vector3d x_ = mp.block(0,0,3,1);   /// p x_axis in base
    Eigen::Vector3d z_ = mp.block(0,2,3,1);   /// p z_axis in base
    Eigen::Vector3d vx = {dir._x,dir._y,dir._z};   /// seam x_axis
    bool fx = true;
    if(x_.dot(vx) < 0)
    {
        fx = false;
        vx = -1*vx;
    }
    vx = vx/sqrt(vx.dot(vx));
    Eigen::Vector3d vz0 = {0,0,-1.0};
    Eigen::Vector3d vy = vz0.cross(vx);
    Eigen::Vector3d vz = vx.cross(vy);
    Eigen::Matrix4d mt4;
    mt4<<vx(0,0),vy(0,0),vz(0,0),0,
            vx(1,0),vy(1,0),vz(1,0),0,
            vx(2,0),vy(2,0),vz(2,0),0,
            0,      0,      0,1;     /// tool in base
    Eigen::Vector3d z_t = mt4.inverse().block(0,0,3,3)*z_;   /// p z_axis in t

    Eigen::Vector3d vx1 = {1.0,0,0};
    Eigen::Vector3d vz1 = {0,0,1.0};
    Eigen::Vector3d z_t_yz = {0, z_t(1), z_t(2)};
    double ax = acos(z_t_yz.dot(vz1)/(sqrt(z_t_yz.dot(z_t_yz))*sqrt(vz1.dot(vz1))));
    double ay = acos(z_t.dot(vx1)/(sqrt(z_t.dot(z_t))*sqrt(vx1.dot(vx1))));
    if(fx)
        ay = pi - ay;

    ax = ax*180/pi;
    ay = ay*180/pi;
    cout <<"Rx_angle :"<< ax << "degree;Ry_angle :" << ay << "degree"<<endl;
}

void interpolation2(vector<RobotPos> &seam, vector<RobotPos> pos_set)
{
    Eigen::Matrix4d r, Rz, Rz1, Ry, Rz2;
    vector<Eigen::Matrix4d> R, R1;
    double pi = M_PI;

    for(int j=0; j<(int)pos_set.size()-1; j++){
        for(int i=j; i<j+2; i++){
            Rz1 << cos(pos_set[i].a/(180/pi)), -sin(pos_set[i].a/(180/pi)), 0, 0,
                    sin(pos_set[i].a/(180/pi)),  cos(pos_set[i].a/(180/pi)), 0, 0,
                    0,                           0, 1, 0,
                    0,                           0, 0, 1;

            Ry <<  cos(pos_set[i].b/(180/pi)),  0, sin(pos_set[i].b/(180/pi)), 0,
                    0,  1,                          0, 0,
                    -sin(pos_set[i].b/(180/pi)),  0, cos(pos_set[i].b/(180/pi)), 0,
                    0,  0,                          0, 1;

            Rz2 << cos(pos_set[i].c/(180/pi)), -sin(pos_set[i].c/(180/pi)), 0, 0,
                    sin(pos_set[i].c/(180/pi)),  cos(pos_set[i].c/(180/pi)), 0, 0,
                    0,                           0, 1, 0,
                    0,                           0, 0, 1;
            r = Rz1*Ry*Rz2;
            r(0, 3) = pos_set[i].x ;
            r(1, 3) = pos_set[i].y ;
            r(2, 3) = pos_set[i].z ;
            R.push_back(r);
        }

        Eigen::Matrix4d ri, rj, rx, r1, r2;
        Eigen::Vector3d z1, z2, z, zi, xi, yi, x1, x2;
        double n1, n2;
        z1 << R[0](0,2), R[0](1,2), R[0](2,2);
        z2 << R[1](0,2), R[1](1,2), R[1](2,2);
        z = z1.cross(z2);
        z << z(0)/sqrt(z.dot(z)), z(1)/sqrt(z.dot(z)), z(2)/sqrt(z.dot(z));
        double t1 = z1.dot(z2)/(sqrt(z1.dot(z1))*sqrt(z2.dot(z2)));
        if(t1 >= 1){
            t1 = 1;
            z = {0, 0, 1};   //zèz1涓è界哥­
            z = z.cross(z1);
        }
        double a = acos(t1);
        zi << 0, 0, 1;
        xi << z(0), z(1), 0;
        xi << xi(0)/sqrt(xi.dot(xi)), xi(1)/sqrt(xi.dot(xi)), 0;
        yi = zi.cross(xi);
        xi = z;
        zi = xi.cross(yi);
        ri << xi(0), yi(0), zi(0), 0,
                xi(1), yi(1), zi(1), 0,
                xi(2), yi(2), zi(2), 0,
                0,     0,     0, 1;

        rx << 1,      0,       0,  0,
                0, cos(a), -sin(a),  0,
                0, sin(a),  cos(a),  0,
                0,      0,       0,  1;

        r1 = R[0];
        r2 = R[1];
        r1 = ri*rx*ri.inverse()*r1;
        x1 << r1(0,0), r1(1,0), r1(2,0);
        x2 << r2(0,0), r2(1,0), r2(2,0);
        z2 << r2(0,2), r2(1,2), r2(2,2);

        double t2 = x1.dot(x2)/(sqrt(x1.dot(x1))*sqrt(x2.dot(x2)));
        if(t2 > 1)
            t2 = 1;
        double a2 = acos(t2);
        z = x1.cross(x2);
        z << z(0)/sqrt(z.dot(z)), z(1)/sqrt(z.dot(z)), z(2)/sqrt(z.dot(z));
        if(z.dot(z2) < 0)
            a2 = - a2;
        vector<Eigen::Matrix4d>().swap(R);

        for(int i=0; i<(int)seam.size(); i++){
            if(pos_set[j].x == seam[i].x && pos_set[j].y == seam[i].y && pos_set[j].z == seam[i].z)
                n1 = i;
            if(pos_set[j+1].x == seam[i].x && pos_set[j+1].y == seam[i].y && pos_set[j+1].z == seam[i].z){
                n2 = i;
                break;
            }
        }

        int N = n2 - n1;
        for(int i=n1; i<n2; i++){
            rx << 1,               0,                0,  0,
                    0, cos(a*(i-n1)/N), -sin(a*(i-n1)/N),  0,
                    0, sin(a*(i-n1)/N),  cos(a*(i-n1)/N),  0,
                    0,               0,                0,  1;

            Rz << cos(a2*(i-n1)/N), -sin(a2*(i-n1)/N), 0, 0,
                    sin(a2*(i-n1)/N),  cos(a2*(i-n1)/N), 0, 0,
                    0,                 0, 1, 0,
                    0,                 0, 0, 1;

            Rz1 << cos(seam[n1].a/(180/pi)), -sin(seam[n1].a/(180/pi)), 0, 0,
                    sin(seam[n1].a/(180/pi)),  cos(seam[n1].a/(180/pi)), 0, 0,
                    0,                         0, 1, 0,
                    0,                         0, 0, 1;

            Ry <<  cos(seam[n1].b/(180/pi)),  0, sin(seam[n1].b/(180/pi)), 0,
                    0,  1,                        0, 0,
                    -sin(seam[n1].b/(180/pi)),  0, cos(seam[n1].b/(180/pi)), 0,
                    0,  0,                        0, 1;

            Rz2 << cos(seam[n1].c/(180/pi)), -sin(seam[n1].c/(180/pi)), 0, 0,
                    sin(seam[n1].c/(180/pi)),  cos(seam[n1].c/(180/pi)), 0, 0,
                    0,                         0, 1, 0,
                    0,                         0, 0, 1;
            r = Rz1*Ry*Rz2;
            r = ri*rx*ri.inverse()*r*Rz;
            r(0, 3) = seam[i].x ;
            r(1, 3) = seam[i].y ;
            r(2, 3) = seam[i].z ;
            R1.push_back(r);
        }
    }

    double a, b, c;
    RobotPos pos, pos_end;
    pos_end = seam[seam.size()-1];
    pos_end.tcp = UPLOAD;
    vector<RobotPos>().swap(seam);
    for(int i=0; i<(int)R1.size(); i++){
        b = acos(R1[i](2, 2));
                a = atan2(R1[i](1,2),R1[i](0,2));
                c = atan2((-1*R1[i](0,0)*sin(a)+R1[i](1,0)*cos(a)),(R1[i](1,1)*cos(a)-R1[i](0,1)*sin(a)));
                           pos.x = R1[i](0, 3);
                          pos.y = R1[i](1, 3);
                pos.z = R1[i](2, 3);
        pos.a = a * 180 / M_PI;
        pos.b = b * 180 / M_PI;
        pos.c = c * 180 / M_PI;
        pos.tcp = UPLOAD;
        seam.push_back(pos);
    }
    seam.push_back(pos_end);
    vector<Eigen::Matrix4d>().swap(R1);
}

void interpolation4(vector<RobotPos> seam, vector<RobotPos> &pos_set, vector<vector<double>> Add)
{
    double dis = 1000000;
    int idx = 0;
    vector<double> index;
    point3d<double> dir;
    vector<point3d<double>> Dir;
    vector<RobotPos> pos_set2, pos_set3;
    for(int i=0; i<(int)pos_set.size(); i++){
        for(int j=0; j<(int)seam.size(); j++){
            if(pow(seam[j].x-pos_set[i].x, 2) + pow(seam[j].y-pos_set[i].y, 2) + pow(seam[j].z-pos_set[i].z, 2) < dis){
                dis = pow(seam[j].x-pos_set[i].x, 2) + pow(seam[j].y-pos_set[i].y, 2) + pow(seam[j].z-pos_set[i].z, 2);
                idx = j;
            }
        }
        index.push_back(idx);
        dis = 1000000;
        idx = 0;
    }

    for(int i=0; i<(int)seam.size()-1; i++){
        dir._x = seam[i+1].x-seam[i].x;
        dir._y = seam[i+1].y-seam[i].y;
        dir._z = seam[i+1].z-seam[i].z;
        Dir.push_back(dir);
    }

    for(int i=0; i<(int)index.size(); i++){
        if(index[i] == int(seam.size())-1)
            Euler_Adjustment3(pos_set[i], Dir[index[i]-1]);
        else
            Euler_Adjustment3(pos_set[i], Dir[index[i]]);
    }


    for(int i=0; i<(int)index.size(); i++){
        if(index[i] == int(seam.size())-1)
            Euler_Adjustment2(seam[index[i]], pos_set[i], Dir[index[i]-1], Add[i][0], Add[i][1], 0, 0);
        else
            Euler_Adjustment2(seam[index[i]], pos_set[i], Dir[index[i]], Add[i][0], Add[i][1], 0, 0);
        pos_set3.push_back(seam[index[i]]);
    }

    if(Add[0][0] == 0 && Add[0][1] == 0){
        pos_set3[0].a = pos_set[0].a;
        pos_set3[0].b = pos_set[0].b;
        pos_set3[0].c = pos_set[0].c;
    }

    if(Add[pos_set3.size()-1][0] == 0 && Add[pos_set3.size()-1][1] == 0){
        pos_set3[pos_set3.size()-1].a = pos_set[pos_set3.size()-1].a;
        pos_set3[pos_set3.size()-1].b = pos_set[pos_set3.size()-1].b;
        pos_set3[pos_set3.size()-1].c = pos_set[pos_set3.size()-1].c;
    }

    for(int i=0; i<(int)pos_set3.size(); i++)
        pos_set[i] = pos_set3[i];

    pos_set2.push_back(pos_set[0]);
    for(int i=1; i<(int)index.size()-1; i++){
        if(Add[i][0] == 0 && Add[i][1] == 0)
            continue;
        pos_set2.push_back(pos_set[i]);
    }
    pos_set2.push_back(pos_set[pos_set3.size()-1]);

    interpolation2(pos_set, pos_set2);
    vector<double>().swap(index);
    vector<point3d<double>>().swap(Dir);
    vector<RobotPos>().swap(pos_set2);
    vector<RobotPos>().swap(pos_set3);
}

bool tracing_end_correlation(string name, std::vector<RobotPos> & oriPath,
                  std::vector<RobotPos> seam2,double l2,double gap2,
                  std::vector<RobotPos> & weldVec,const double p2p_len)
{
    ofstream ofs;
    ofs.open(name);
    /// 提取部分點列,用于拟合
    ofs<<"oriPath拟合用原始点:"<<endl;
    auto iter_ori = oriPath.end()-1;
    vector<RobotPos> fitVec0;
    double sumlen = 0;
    while(iter_ori>=oriPath.begin() && sumlen < 100)
    {
        RobotPos p0 = *iter_ori;
        fitVec0.push_back(p0);
        iter_ori--;
        RobotPos p1 = *iter_ori;
        sumlen += p0.dis(p1);
    }
    for(auto p : fitVec0){
        ofs<<p.toStr()<<" "<<p.dh<<","<<p.dw<<endl;
    }
    /// 提取部分关联点裂,用于拟合
    ofs<<"seam2原始点:"<<endl;
    for(auto p : seam2){
        ofs<<p.toStr()<<" "<<p.dh<<","<<p.dw<<endl;
    }
    if(seam2.size() == 0)
    {
        ofs.close();
        return false;
    }
    auto iter_end = seam2.end()-1;
    auto iter_start = seam2.begin();
    vector<RobotPos> fitVec2;
    double sumlen2 = 0;
    if(iter_start->dis(fitVec0[0]) < iter_end->dis(fitVec0[0]))
    {
        while(iter_start<seam2.end() && sumlen2 < 100)
        {
            RobotPos p0 = *iter_start;
            fitVec2.push_back(p0);
            iter_start++;
            RobotPos p1 = *iter_start;
            sumlen2 += p0.dis(p1);
        }
    }
    else
    {
        while(iter_end>=seam2.begin() && sumlen2 < 100)
        {
            RobotPos p0 = *iter_end;
            fitVec2.push_back(p0);
            iter_end--;
            RobotPos p1 = *iter_end;
            sumlen2 += p0.dis(p1);
        }
    }
    ///原始点数太少
    if(fitVec0.size() < TRACK_POINT_LIMIT)
    {
        ofs << "seam0.size="<<fitVec0.size()<<endl;
        ofs.close();
        return false;
    }
    /// seam0的xyz曲线模型
    SepModel3D model0;
    model0.setParams({2});
    vector<double> _x[3];
    for(uint t=0;t<fitVec0.size();t++)
    {
        _x[0].push_back(fitVec0[t].x);
        _x[1].push_back(fitVec0[t].y);
        _x[2].push_back(fitVec0[t].z);
    }
    model0.setRealSE(0,fitVec0.size()-1);
    model0.setData(_x[0],_x[1],_x[2]);
    try
    {
        model0.build();
    }catch(...)
    {
        ofs << "2次曲线拟合失败!"<<endl;
        return false;
    }
    double rms = model0.getRMS();
    ofs<<"model0 RMS:"<<rms<<endl;
    if(fitVec2.size()  < 3)
    {
        ofs << "fitVec2.size="<<fitVec2.size()<<endl;
        return false;
    }
    /// seam2的xyz曲线模型
    SepModel3D model2;
    model2.setParams({2});
    vector<double> _x2[3];
    for(uint t=0;t<fitVec2.size();t++)
    {
        _x2[0].push_back(fitVec2[t].x);
        _x2[1].push_back(fitVec2[t].y);
        _x2[2].push_back(fitVec2[t].z);
    }
    model2.setRealSE(0,fitVec2.size()-1);
    model2.setData(_x2[0],_x2[1],_x2[2]);
    try
    {
        model2.build();
    }catch(...)
    {
        ofs << "seam2多项式曲线拟合失败!"<<endl;
        return false;
    }
    double rms2 = model2.getRMS();
    ofs<<"model2 RMS:"<<rms2<<endl;
    double seam0_t1,seam2_t0;
    if(fitVec2.size() > 0)
    {
        double mis;
        if(!getModelsT(model0,model2,seam0_t1,seam2_t0,mis) || (gap2 >=0 && mis > LINE_2_LINE+gap2))
        {
            point3d<double> mt0 = model0.model(seam0_t1);
            point3d<double> mt2 = model2.model(seam2_t0);
            ofs<<"关联失败!"<<endl;
            ofs<<"seam0<->seam2:"<<mis<<endl;
            ofs<<"model0(t0):"<<mt0._x<<","<<mt0._y<<","<<mt0._z<<endl;
            ofs<<"model2(t2):"<<mt2._x<<","<<mt2._y<<","<<mt2._z<<endl;
            return false;
        }
        point3d<double> mt0 = model0.model(seam0_t1);
        point3d<double> mt2 = model2.model(seam2_t0);
        ofs<<"model0("<<seam0_t1<<"):"<<mt0._x<<","<<mt0._y<<","<<mt0._z<<endl;
        ofs<<"model2("<<seam2_t0<<"):"<<mt2._x<<","<<mt2._y<<","<<mt2._z<<endl;
        ofs<<"seam0<->seam2:"<<mis<<endl;
    }
    double t0 = seam0_t1;
    auto iter_we = weldVec.end()-1;
    RobotPos wE = *iter_we;
    iter_we--;
    RobotPos wEp = *iter_we;
    point3dd we3d = {wE.x,wE.y,wE.z};
    point3dd we3dp = {wEp.x,wEp.y,wEp.z};
    point3dd we_dir = we3d - we3dp;
    point3dd pti;
    while(1)
    {
        pti = model0.model(t0);
        if(we3d.dist(pti) > p2p_len)
        {
            t0 += 0.1;
        }
        else
        {
            break;
        }
    }
    point3dd pti_dir = pti-we3d;
    if(pti_dir.inner(we_dir)<0 || pti.dist(we3d) < p2p_len-0.11)
    {
        ofs<<"关联失败!"<<endl;
        ofs.close();
        return false;
    }
    while(t0 > seam0_t1-l2)
    {
        pti = model0.model(t0);
        RobotPos p = wE;
        p.x = pti._x;
        p.y = pti._y;
        p.z = pti._z;
        ofs<<p.toStr()<<endl;
        weldVec.push_back(p);
        t0 -= p2p_len;
    }
    ofs.close();
    return true;
}
///初始跟踪轨迹拟合
bool initStartPath(string logPath, std::vector<RobotPos> oriPath,
                            std::vector<RobotPos> seamKeel,
                            std::vector<RobotPos>& weldVec,const double p2p_len,
                   RelateInfo rif)
{
    if(oriPath.size()< START_POINT_LIMIT)
        return false;
    if(rif.beginPath.size() > 0)
    {
        Vector3d abc;
        if(points2face(oriPath,abc))
        {
            point3dd pabc = {abc(0,0),abc(1,0),abc(2,0)};
            shadow(pabc,oriPath);
        }
        vector<RobotPos>().swap(rif.endPath);
        rif.endRank = 0;
        rif.endId = 0;
        correlationBySep(logPath,oriPath,TRACK_RANK,rif);
    }
    ofstream ofs;
    ofs.open(logPath+"/path1.log");
    SepModel3D smd;
    smd.setParams({TRACK_RANK});
    vector<double> _x[3];
    for(auto & pt : oriPath)
    {
        _x[0].push_back(pt.x);
        _x[1].push_back(pt.y);
        _x[2].push_back(pt.z);
    }
    smd.setRealSE(0,oriPath.size());
    smd.setData(_x[0],_x[1],_x[2]);
    point3dd smdpos;
    try
    {
        smd.build();
        /// 放入第0个pos
        point3dd pre = smd.model(0);
        RobotPos rpp = seamKeel[0];
        rpp.x = pre._x;
        rpp.y = pre._y;
        rpp.z = pre._z;
        weldVec.push_back(rpp);
        ofs <<rpp.toStr()<<endl;
        double len = smd.length;
        double cur = 0.001;
        int sk = 1;
        while(cur < len)
        {
            smdpos = smd.model(cur);
            if(abs(smdpos.dist(pre)-p2p_len) < 0.01)
            {
                RobotPos rpp = seamKeel[sk];
                rpp.x = smdpos._x;
                rpp.y = smdpos._y;
                rpp.z = smdpos._z;
                weldVec.push_back(rpp);
                pre = smdpos;
                sk++;
                ofs <<rpp.toStr()<<endl;
            }
            cur += 0.001;
        }
        ofs.close();
        return true;
    }catch(...)
    {
        ofs.close();
        return false;
    }
}

bool getModelPos(SepModel3D sm, point3dd & pos,point3dd pte,point3dd ppte,const double p2p_len)
{
    double smlen = sm.length;
    auto p = sm.model(smlen);
    point3dd pdir = pte-ppte;
    point3dd pcur = p-pte;
    if(pdir.inner(pcur) < 0)
    {
        return false;
    }
    else
    {
        while(pte.dist(p) > p2p_len && smlen > 0 && pdir.inner(pcur) > 0)
        {
            double pdp = pte.dist(p) - p2p_len;
            pdp = pdp>0.0005?pdp:0.0005;
            smlen-=pdp*0.618;
            p = sm.model(smlen);
            pcur = p-pte;
        }
    }
    if(smlen > 0 && fabs(pte.dist(p) - p2p_len) < 0.01)
    {
        pos = p;
        return true;
    }
    return false;
}

/// 从龙骨中取出需要的pos汇入oriPath
bool getPosFromKeel(RobotPos & keel,std::vector<RobotPos> oriPath,
                    std::vector<RobotPos> weldVec,
                    std::vector<RobotPos> seamKeel,const double p2p_len)
{
    int ws = weldVec.size();
    if(ws<2)
        return false;
    if(ws >= seamKeel.size())
        return false;
    int os = oriPath.size();
    if(os < 2)
        return false;
    RobotPos rp = weldVec[ws-1];
    RobotPos op = oriPath[os-1];
    if(rp.dis(op) > p2p_len)
        return false;
    point3dd wE = {weldVec[ws-1].x-weldVec[ws-2].x,
                   weldVec[ws-1].y-weldVec[ws-2].y,
                   weldVec[ws-1].z-weldVec[ws-2].z};
    point3dd kE = {seamKeel[ws-1].x-seamKeel[ws-2].x,
                   seamKeel[ws-1].y-seamKeel[ws-2].y,
                   seamKeel[ws-1].z-seamKeel[ws-2].z};
    point3dd kEp = {seamKeel[ws].x-seamKeel[ws-1].x,
                    seamKeel[ws].y-seamKeel[ws-1].y,
                    seamKeel[ws].z-seamKeel[ws-1].z};
    point3dd wEp = getKeelDir(wE,kE,kEp);
    keel = seamKeel[ws];
    keel.x = rp.x + wEp._x;
    keel.y = rp.y + wEp._y;
    keel.z = rp.z + wEp._z;
    return true;
}

bool appendWeldPath(string logPath, RobotPos tar,ofstream & ofs,std::vector<RobotPos>& oriPath
                             ,const std::vector<RobotPos> seamKeel,std::vector<RobotPos>& weldVec,const double p2p_len,
                    RelateInfo rif,double endlen,double c2tx, double fit_ps)
{
    int osize = oriPath.size();
    if(osize < START_POINT_LIMIT)
    {
        oriPath.push_back(tar);
        return true;
    }
    Vector4d p2Ori;  // 当前点在oriPath点坐标系内的表达
    int i=osize-1;
    Vector4d vr = {tar.x,tar.y,tar.z,1};
    for(;i>=osize/2;i--)
    {
        Matrix4d mer;
        pos2Matrix(oriPath[i],mer,zyz);
        p2Ori = mer.inverse()*vr;
        /// 激光面是yz平面，点跟点位置差异主要在x方向
        if(p2Ori(0,0) <= 0)
            break;
    }
    if(p2Ori(0,0) > 0) /// 说明当前pos无适合插入的位置
    {
        ofs<<"int oriPath "<<p2Ori(0,0)<<","<<p2Ori(1,0)<<","<<p2Ori(2,0)<<"丢弃";
        return false;
    }
    uint wvs = weldVec.size();
    int sks = seamKeel.size();
    if(wvs > 0)
    {
        RobotPos we = weldVec[wvs-1];
        RobotPos p2w = (!we)<<tar;
        //if(abs(p2Ori(1,0))>p2p_len || abs(p2Ori(2,0))>p2p_len)
        if(abs(p2w.y)>p2p_len && abs(p2w.y)*2 > abs(p2w.x))
        {
            char str[128];
            ::sprintf(str,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\t%.1f,%.3f,%.3f 丢弃",
                      we.x,we.y,we.z,we.a,we.b,we.c,p2Ori(0,0),p2Ori(1,0),p2Ori(2,0));
            ofs <<str<<endl;
            return false;
        }
        /// wvs>sks-10
        if(wvs>sks-10 && (abs(p2Ori(1,0))>p2p_len || abs(p2Ori(2,0))>p2p_len))
        {
            char str[128];
            ::sprintf(str,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\t%.1f,%.3f,%.3f 丢弃",
                      we.x,we.y,we.z,we.a,we.b,we.c,p2Ori(0,0),p2Ori(1,0),p2Ori(2,0));
            ofs <<str<<endl;
            return false;
        }
    }
    oriPath.insert(oriPath.begin()+i+1,tar);
    RobotPos p0 = oriPath[0];
    RobotPos p1 = oriPath[osize];
    point3dd p03d = {p0.x,p0.y,p0.z};
    point3dd p13d = {p1.x,p1.y,p1.z};
    if(p13d.dist(p03d) < START_LEN_LIMIT*p2p_len && weldVec.size()<3)
    {
        return true;
    }
    static int fitIndex = 0;
    if(weldVec.size() == 0)
    {
        initStartPath(logPath,oriPath,seamKeel,weldVec,p2p_len,rif);
        fitIndex = 0;
        return true;
    }
    RobotPos weldE = weldVec[wvs-1]; /// 当前焊缝尾部
    RobotPos pweldE = weldVec[wvs-2]; /// 当前焊缝尾部
    vector<RobotPos> fitPs;
    point3dd pte = {weldE.x,weldE.y,weldE.z};
    point3dd ppte = {pweldE.x,pweldE.y,pweldE.z};
    double ll = oriPath[osize].dis(weldE);
    ofs<<"\ttoEnd(mm):"<<ll;
    if(ll < c2tx-fit_ps*p2p_len)
        return true; /// no need fit
    int fi= fitIndex;
    fi = fi>20?fi-20:fi;
    fi = fi<0?0:fi;
    bool updateFitIndex = false;
    for(;fi<=osize;fi++)
    {
        auto p = oriPath[fi];
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
        ofs<<" "<<fi<<"-"<<fitPs.size()<<"fit点数过少";
        return true;
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
        ofs<<"fit start\t"<<fi-fitPs.size()+1<<"-"<<fi;
        smd.build();
        if(getModelPos(smd,smdpos,pte,ppte,p2p_len))
        {
            uint abci = wvs<sks?wvs:sks-1;
            RobotPos keelPos = seamKeel[abci];
            keelPos.x = smdpos._x;
            keelPos.y = smdpos._y;
            keelPos.z = smdpos._z;
            if(wvs >= sks-10)
            {
                RobotPos wc0 = weldVec[0];
                RobotPos mis = (!wc0)<<keelPos;
                if(keelPos.dis(wc0)>2*endlen || mis.x > -endlen-0.1)
                {
                    weldVec.push_back(keelPos);
                }
            }
            else
            {
                weldVec.push_back(keelPos);
            }
        }
    }catch(...)
    {
        ofs<<" "<<"fit err";
    }
    return true;
}

bool getCenter(vector<RobotPos> p3d, RobotPos & p0)
{
    uint counts = p3d.size();
    if(counts < 1)
        return false;
    RobotPos avg = p0;
    avg.x = 0;
    avg.y = 0;
    avg.z =0;
    for(auto d : p3d)
    {
        avg.x += d.x;
        avg.y += d.y;
        avg.z += d.z;
    }
    avg.x/=counts;
    avg.y/=counts;
    avg.z/=counts;
    if(counts <= 1)
    {
        p0 = avg;
        return true;
    }
    double max;
    for(auto d : p3d)
    {
        double ddis = d.dis(avg);
        max = ddis>max?ddis:max;
    }
    if(max < 3)
    {
        p0 = avg;
        return true;
    }
    for(int per = 9;per>0;per--)
    {
        vector<RobotPos> p3d_;  /// 晒选
        for(auto d : p3d)
        {
            if(d.dis(avg) < max*per/10.0)
                p3d_.push_back(d);
        }
        if(p3d_.size() < counts*0.8)
        {
            return getCenter(p3d_,p0);
        }
    }
    return false;
}

/// 跟踪因图像异常退出前，将轨迹拟合结束
void fitTracingErrEnd(ofstream & ofs,std::vector<RobotPos>& oriPath,
                      const std::vector<RobotPos> seamKeel,
                      std::vector<RobotPos>& weldVec,const double p2p_len,double endlen)
{
    ofs<<"imgprocess enter in err end!"<<endl;
    int osize = oriPath.size();
    int wvs = weldVec.size();
    int sks = seamKeel.size();
    vector<double> _x[3];
    uint fi = osize>50?osize-50:0;
    for(;fi<=osize;fi++)
    {
        _x[0].push_back(oriPath[fi].x);
        _x[1].push_back(oriPath[fi].y);
        _x[2].push_back(oriPath[fi].z);
    }
    int fs = _x[0].size();
    if(fs < TRACK_POINT_LIMIT)
    {
        ofs<<" "<<fi-fs<<"-"<<fs<<"fit点数过少";
        return;
    }
    SepModel3D smd;
    smd.setParams({TRACK_RANK});
    smd.setRealSE(0,_x[0].size());
    smd.setData(_x[0],_x[1],_x[2]);
    point3dd smdpos;
    try
    {
        ofs<<"fit start\t"<<fi-fs<<"-"<<fi;
        smd.build();
        smdpos = smd.model(smd.length);
        RobotPos wce = weldVec[wvs-1];
        uint abci = wvs<sks?wvs:sks-1;
        RobotPos keelPos = seamKeel[abci];
        keelPos.x = smdpos._x;
        keelPos.y = smdpos._y;
        keelPos.z = smdpos._z;
        RobotPos kinwc = (!wce)<<keelPos;
        if(kinwc.x >= 0)
        {
            ofs<<" "<<"scaned";
            return;
        }
        if(wvs >= sks-10)
        {
            RobotPos wc0 = weldVec[0];
            RobotPos mis = keelPos>>(!wc0);
            if(keelPos.dis(wc0)>endlen || mis.x > -endlen - 0.1)
            {
                weldVec.push_back(keelPos);
            }
        }
        else
        {
            weldVec.push_back(keelPos);
        }
    }catch(...)
    {
        ofs<<" "<<"fit err";
    }
}

UV getJUV(cv::Mat& m,int p,int thresh,string time)
{
    cout<<"getJUV 算法失效！！！！！"<<endl;
    throw -1;
}

PolyModel1D pmds,pmdi,pmdv;
void weldEmpi()
{
    ifstream ifs;
    ifs.open("etc/weld.info");
    string line;
    std::deque<double> ds,di,dv,gap;
    while(getline(ifs,line))
    {
        QString ql = QString::fromStdString(line);
        QStringList qls = ql.split(",");
        if(qls.size() < 5)
            continue;
        gap.push_back(qls[1].toDouble());
        ds.push_back(qls[2].toDouble());
        di.push_back(qls[3].toDouble());
        dv.push_back(qls[4].toDouble());
    }
    pmds.setParams({2});
    pmdi.setParams({2});
    pmdv.setParams({2});
    pmds.setData(ds,gap);
    pmdi.setData(di,gap);
    pmdv.setData(dv,gap);
    pmds.build();
    pmdi.build();
    pmdv.build();
}

void weldAdaption(double gap, RobotPos & p, weldInfo & wf)
{
    double ps0 = pmds.model(0);
    double psp = pmds.model(gap);
    double rate = ps0>0.1?psp/ps0:psp/0.1;
    rate = rate>2?2:rate;
    rate = rate<0.5?0.5:rate;
    p.v *= rate;
    wf.I_W = pmdi.model(gap);
    wf.V_W = pmdv.model(gap);
    cout<<"ps0"<<ps0<<"psp"<<psp<<"p.v"<<p.v<<"I_W"<<wf.I_W<<"V_W"<<wf.V_W;
}

bool checkUV(UV tar,UV center)
{
    if(abs(tar.u-center.u)> 200 || abs(tar.v-center.v)> 200)
    {
        return false;
    }
    return true;
}

void externPath3(vector<RobotPos> path,
                 vector<RobotPos> keel0,
                 vector<RobotPos> & keel1,
                 vector<RobotPos> & keel2,
                 vector<RobotPos> & keel3)
{
//    if(path.size() < 2  || keel0.size() < 2 || keel1.size() < 2 || keel2.size() < 2 || keel3.size() < 2)
//    {
//        cout<<"externPath3轨迹点数太少！，扩展失败！"<<endl;
//        return;
//    }
    ofstream ofs;
    ofs.open("externPath3_out.log");
    ofs<<"in path0"<<endl;
    for(auto p :path)
    {
        ofs<<p.toStr()<<endl;
    }
    ofs<<"in keel0"<<endl;
    for(auto k :keel0)
    {
        ofs<<k.toStr()<<endl;
    }
    ofs<<"in keel1"<<endl;
    for(auto k :keel1)
    {
        ofs<<k.toStr()<<endl;
    }
    ofs<<"in keel2"<<endl;
    for(auto k :keel2)
    {
        ofs<<k.toStr()<<endl;
    }
    ofs<<"in keel3"<<endl;
    for(auto k :keel3)
    {
        ofs<<k.toStr()<<endl;
    }
    double pspn = path[0].dis(path[1]);
    double kspn = keel0[0].dis(keel0[1]);
    int rate = d_round(kspn/pspn);
    double Len0 = (keel0.size()-1)*rate;
    double Len1 = (keel1.size()-1)*rate;
    vector<RobotPos> path1;
    RobotPos zero = RobotPos::instance();
    point3dd dir_90;
    int i=0;
    RobotPos z0 = RobotPos::instance();
    z0.z = 0; /// TEST
    for(;i<path.size()-1;i++)
    {
        RobotPos pi = path[i]<<z0;
        RobotPos pj = path[i+1]<<z0;
        point3dd dir = {pj.x-pi.x,pj.y-pi.y,0};
        dir_90 = {dir._y,-dir._x,0};
        dir_90.normalize();
        RobotPos p1i = keel1[d_round(i/rate)];
        p1i.x = pi.x+dir_90._x*4;
        p1i.y = pi.y+dir_90._y*4;
        p1i.z = pi.z;
        if(i<Len1)
            path1.push_back(p1i);
    }
    keel1.swap(path1);
    ofs<<"out keel1"<<endl;
    for(auto k :keel1)
    {
        ofs<<k.toStr()<<endl;
    }
    if(keel2.size() > 0)
    {
        double Len2 = (keel2.size()-1)*rate;
        vector<RobotPos> path2;
        for(int i=0;i<Len2;i++)
        {
            int i0 = d_round(i*Len0/Len2);
            i0 = i0<path.size()?i0:path.size()-1;
            RobotPos mis = (!keel0[d_round(i0/rate)])<<keel2[d_round(i/rate)];
            RobotPos p2i = path[i0]<<mis;
            path2.push_back(p2i);
        }
        keel2.swap(path2);
        ofs<<"out keel2"<<endl;
        for(auto k :keel2)
        {
            ofs<<k.toStr()<<endl;
        }
    }
    if(keel3.size() > 0)
    {
        double Len3 = (keel3.size()-1)*rate;
        vector<RobotPos> path3;
        for(int i=0;i<Len3;i++)
        {
            int i0 = d_round(i*Len0/Len3);
            i0 = i0<path.size()?i0:path.size()-1;
            RobotPos mis = (!keel0[d_round(i0/rate)])<<keel3[d_round(i/rate)];
            RobotPos p3i = path[i0]<<mis;
            path3.push_back(p3i);
        }
        keel3.swap(path3);
        ofs<<"out keel3"<<endl;
        for(auto k :keel3)
        {
            ofs<<k.toStr()<<endl;
        }
    }
    ofs.close();
}

bool getCirclePos(CircleModel cm, point3dd & pos,point3dd pte,point3dd ppte,const double p2p_len)
{
    double smlen = cm.getLen();
    auto p = cm.at(smlen);
    point3dd pdir = pte-ppte;
    point3dd pcur = p-pte;
    if(pdir.inner(pcur) < 0)
    {
        return false;
    }
    else
    {
        while(pte.dist(p) > p2p_len && smlen > 0 && pdir.inner(pcur) > 0)
        {
            double pdp = pte.dist(p) - p2p_len;
            pdp = pdp>0.0005?pdp:0.0005;
            smlen-=pdp*0.618;
            p = cm.at(smlen);
            pcur = p-pte;
        }
    }
    if(smlen > 0 && fabs(pte.dist(p) - p2p_len) < 0.01)
    {
        pos = p;
        return true;
    }
    return false;
}

MOTORDEGRE getAngleByTime(double curTime, WELDANGLEINFO angleInfo)
{
    MOTORDEGRE degrees = angleInfo.motors;
    if(angleInfo.rotateTime > 1 && curTime > angleInfo.rotateTime)
    {
        double timemis = curTime-angleInfo.rotateTime;
        double curpAngle1 = timemis*angleInfo.angleSpeed1;
        double maxAngleCross1 = (angleInfo.motore.angle1-angleInfo.motors.angle1);
        degrees.angle1 += abs(curpAngle1)>abs(maxAngleCross1)?maxAngleCross1:curpAngle1;
        double curpAngle2 = timemis*angleInfo.angleSpeed2;
        double maxAngleCross2 = (angleInfo.motore.angle2-angleInfo.motors.angle2);
        degrees.angle2 += abs(curpAngle2)>abs(maxAngleCross2)?maxAngleCross2:curpAngle2;
        double curpAngle3 = timemis*angleInfo.angleSpeed3;
        double maxAngleCross3 = (angleInfo.motore.angle3-angleInfo.motors.angle3);
        degrees.angle3 += abs(curpAngle3)>abs(maxAngleCross3)?maxAngleCross3:curpAngle3;
    }
    return degrees;
}
