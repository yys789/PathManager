#include <sstream>
#include <iostream>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/filesystem.hpp>
#include "base.h"
#include "tools.h"
#include "tools/matrix.h"

using namespace chrono;
using namespace boost::property_tree ;
using namespace cv;

/// int转换字符串正数正常返回，负数或0返回空字符串
std::string int2str(int m)
{
    if(m>0)
        return to_string(m);
    else
        return "";
}

bool str2Robot(string str, RobotPos & f)
{
    f = RobotPos::instance();
    char split[3] = {',',' ','\t'};
    vector<double> ds;
    for(int i=0;i<3;i++)
    {
        ds.resize(0);
        ds.clear();
        stringstream ss(str);
        while(ss.good())
        {
            string substr;
            getline( ss, substr,split[i]);
            stringstream sd(substr);
            double di;
            sd>>di;
            ds.push_back(di);
        }
        if(ds.size() >= 6)
            break;
    }
    if(ds.size() < 6)
        return false;
    f.x = ds[0];
    f.y = ds[1];
    f.z = ds[2];
    f.a = ds[3];
    f.b = ds[4];
    f.c = ds[5];
    return true;
}

double getMicTime()
{
    return duration_cast<milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()/1000.0;
}

std::vector<float> fitPath( const std::vector<RobotPos> pos, std::vector<RobotPos> &new_pos)
{
    using namespace cv;
    if ( 0 == pos.size() ) {
       return {} ;
    }
    std::vector<Point3f> position;  //位置拟合
    std::vector<float> diff;
    for (unsigned long  i = 0; i < pos.size(); i++)
    {
        position.push_back(Point3f(pos[i].x, pos[i].y, pos[i].z));
    }

    //储存拟合直线的容器
    Vec6f line1;
    //直线拟合函数
    fitLine(position, line1, CV_DIST_L1, 0, 0.01, 0.01);
    float vx = line1[0];
    float vy = line1[1];
    float vz = line1[2];
    float x0  = line1[3];
    float y0  = line1[4];
    float z0  = line1[5];

    for (unsigned long i = 0; i < pos.size(); i++)
    {
        //求解直线参数
        float t = (vx*(pos[i].x-x0)+vy*(pos[i].y-y0)+vz*(pos[i].z-z0))/(vx*vx+vy*vy+vz*vz);
        float new_x = vx * t + x0;
        float new_y = vy * t + y0;
        float new_z = vz * t + z0;

        new_pos.push_back(RobotPos{new_x, new_y, new_z, pos[i].a, pos[i].b, pos[i].c});
        diff.push_back(sqrt(pow(new_x - pos[i].x, 2) + pow(new_y  - pos[i].y, 2) + pow(new_z - pos[i].z, 2)));
    }
    return diff;
}

std::vector<UV> findUV(std::vector<UV> seamUV)
{

    using namespace std;
    int r_umax = int(sqrt(1280 * 1280 + pow(seamUV.size(),2)));
    int r_vmax = int(sqrt(1024 * 1024 + pow(seamUV.size(),2)));
    int r_u,r_v;

    cout << r_umax << ' ' << r_vmax << endl;
    int  **uhist = new int*[500];
    int  **vhist = new int*[500];
    //int vhist1[180][r_umax] ;
    for( int i=0; i<500; i++ )
    {
        uhist[i] = new int [2*r_umax];
        memset(uhist[i], 0, sizeof(uhist[i][0]) * 2 * r_umax) ;
        vhist[i] = new int [2*r_vmax];
        memset(vhist[i], 0, sizeof(vhist[i][0]) * 2 * r_vmax) ;
    }
    //每1000张照片改变1.7个像素 1000*tan0.1
    for (int theta = 0; theta < 500; theta++)
    {
        for(unsigned long i = 0; i < seamUV.size(); i++)
        {
            if(!(lessequald(seamUV[i].u, 10)||greaterequald(seamUV[i].u,1270)||lessequald(seamUV[i].v, 5)||greaterequald(seamUV[i].v,1020)))
            {
                r_u = int(sin((theta/10-25)/180.0*M_PI) * i + cos((theta/10-25)/180.0*M_PI) * seamUV[i].u);
                r_v = int(sin((theta/10-25)/180.0*M_PI) * i + cos((theta/10-25)/180.0*M_PI) * seamUV[i].v);
                uhist[theta][r_u+r_umax]++;
                vhist[theta][r_v+r_vmax]++;
            }
        }
    }
    //寻找最大值
    int theta_u = 0;
    int theta_v = 0;
    r_u = 0;
    r_v = 0;
    int u_max = 0;
    int v_max = 0;
    for(int i = 0; i < 500; i++)
    {
       for(int j = 0; j < 2 * r_umax; j++)
       {
          if(uhist[i][j] > u_max)
          {
             u_max = uhist[i][j];
             theta_u = i;
             r_u = j;
          }
       }
       for(int k = 0; k < 2 * r_vmax; k++)
       {
          if(vhist[i][k] > v_max)
          {
             v_max = vhist[i][k];
             theta_v = i;
             r_v = k;
          }
       }
    }
    cout << u_max << ' ' << v_max << ' ' << theta_u << ' ' << r_u << ' ' << theta_v << ' ' << r_v << endl;
    delete [] uhist;
    delete [] vhist;

    //计算新直线的点集
    vector<UV> uv;
    for(unsigned long i = 0; i < seamUV.size(); i++)
    {
        uv.push_back({-tan((theta_u/10-25)/180.0*M_PI) * i + (r_u - r_umax) / cos((theta_u/10-25)/180.0*M_PI),
              -tan((theta_v/10-25)/180.0*M_PI) * i + (r_v - r_vmax) / cos((theta_v/10-25)/180.0*M_PI)});
    }
    return uv;
}

boost::signals2::signal<UV (cv::Mat)> manaualDetectUV_sig ;
UV manaualDetectUV( const cv::Mat &image )
{
   return *manaualDetectUV_sig(image) ;
}

string getTime()
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep) );
    return tmp;
}

string getCurrentTime()
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S",localtime(&timep) );
    return tmp;
}

string getDate()
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d",localtime(&timep) );
    return tmp;
}

void saveImg(const Mat &mat, const string &path, const string &name)
{
   namespace bf = boost::filesystem ;
   using namespace std ;
   string strDir = "picture" ;
   boost::posix_time::ptime pt = boost::posix_time::microsec_clock::universal_time() ;
   // 以当前时间为默认图片文件名
   string strName = boost::posix_time::to_iso_string(pt) + ".png";
   if ( "" != path ) {
       strDir = path ;
   }
   if ( "" != name ) {
       strName = name ;
   }
   if ( !mat.data ) return ;

   if ( !bf::exists(strDir) ) {
         bf::create_directory(strDir) ;
   }
   imwrite(strDir + "/" + strName , mat) ;
}

void UV2Point3D(const RobotPos &currPos, struct UV uv, RobotPos &posOut,int mode)
{
    Config config("etc/calibration.info") ;
    UV tempUV = config.get<UV>("cameraCenterUV"+int2str(mode));
    uv.u -= tempUV.u ;
    uv.v -= tempUV.v ;
    float y, z;
        laser_yz(uv.u, uv.v, y, z,mode);
    cout<< y<<" "<<z<<"\n";
    double TCP[3] = {0, y, z};
    laser2Base(TCP,  currPos, posOut, mode);
}

double stringToNum(const string& str)
{
    if(str == "")
        return 0;
    double d = 0;
    try
    {
        string::size_type size; //13
        d = stod(str, &size);  //1234.56789
    }
    catch(...){}
    return d;
//    istringstream iss(str);
//    double num;
//    iss >> num;
//    return num;
}

string DecToBin(int n)
{
    string   s="";
    for(int a=n;a;a=a/2)
        s=s+(a%2?'1':'0');
    std::reverse(s.begin(),s.end());
    return s;
}

int BinToDec(string str)
{
    int d=0;
    int i=0;
    while(str[i++]!='\0')
        d=d*2+str[i-1]-'0';
    return d;
}

UV strToUV(string str)
{
    int npos = str.find(',',0);
    UV uv;
    uv.u = stringToNum(str.substr(0,npos));
    uv.v = stringToNum(str.substr(npos + 1));
}

void sPosTodVec(string sPos,vector<double>& vec)
{
    string temp = sPos;
    while(1)
    {
        int npos = temp.find(',',0);
        if(npos != string::npos)
        {
            string s = temp.substr(0,npos);
            vec.push_back(stringToNum(s));
            temp = temp.substr(npos+1);
        }else{
            vec.push_back(stringToNum(temp));
            break;
        }
    }
}

void sPosTodPos(string sPos,RobotAxle& axle)
{
    cout<<"sPos : "<<sPos<<endl;
    if(sPos.empty())
        throw "string format error";

    string temp = sPos;
    vector<double> vec;
    while(1)
    {
        int npos = temp.find(',',0);
        if(npos != string::npos)
        {
            string s = temp.substr(0,npos);
            vec.push_back(stringToNum(s));
            temp = temp.substr(npos+1);
        }else{
            vec.push_back(stringToNum(temp));
            break;
        }
    }
    cout<<"pos : "<<endl;
    for(auto& v:vec)
        cout<<v<<endl;

    if(vec.size() == 12)
    {
        axle.a1 = vec[6];
        axle.a2 = vec[7];
        axle.a3 = vec[8];
        axle.a4 = vec[9];
        axle.a5 = vec[10];
        axle.a6 = vec[11];
    }
}

void sPosTodPos(string sPos,RobotPos& pos)
{
    cout<<"sPos : "<<sPos<<endl;
    string temp = sPos;
    vector<double> vec;
    while(1)
    {
        int npos = temp.find(',',0);
        if(npos != string::npos)
        {
            string s = temp.substr(0,npos);
            vec.push_back(stringToNum(s));
            temp = temp.substr(npos+1);
        }else{
            vec.push_back(stringToNum(temp));
            break;
        }
    }
    pos.x = vec[0];
    pos.y = vec[1];
    pos.z = vec[2];
    pos.a = vec[3];
    pos.b = vec[4];
    pos.c = vec[5];
    pos.tcp = RobotTCP::UPLOAD;
}

string AscToString(vector<int>& vec)
{
    int count = vec.size();
    char ch[count+1];
    memset(ch,0,sizeof(ch));
    int i = 0;
    cout<<count<<endl;
    for(; i< count; i++)
    {
        ch[i] = vec[i] + 97 - 'a';
    }
    ch[i+1] ='\0';
    string str(ch);
    return str;
}

string AscToChar(int n)
{
    string c;
    c = n + 97 - 'a';
    return c;
}

void stringToAsc(vector<int>& vec,string str)
{
    int count = str.length();
    char ch[count+1];
    strncpy(ch,str.c_str(),count);
    ch[count+1] = '\n';
    char* p =ch;
    for(int i = 0; i< count; i++)
    {
        vec.push_back(toascii(*p));
        p++;
    }
    for(int i = vec.size()+1; i<= 20;i++)
        vec.push_back(0);
}

void charToAsc(int& n,string str)
{
    int count = str.length();
    char ch[count+1];
    strncpy(ch,str.c_str(),count);
    ch[count+1] = '\n';
    char* p =ch;
    n = toascii(*p);
}

RobotAxle mapToRobotAxle(map<string,string>& m)
{
    double a1 = stringToNum(m["a1"]);
    double a2 = stringToNum(m["a2"]);
    double a3 = stringToNum(m["a3"]);
    double a4 = stringToNum(m["a4"]);
    double a5 = stringToNum(m["a5"]);
    double a6 = stringToNum(m["a6"]);
    RobotAxle axle{a1,a2,a3,a4,a5,a6};
    return axle;
}

RobotPos mapToRobotPos(map<string,string>& m)
{
    double x = stringToNum(m["x"]);
    double y = stringToNum(m["y"]);
    double z = stringToNum(m["z"]);
    double a = stringToNum(m["a"]);
    double b = stringToNum(m["b"]);
    double c = stringToNum(m["c"]);
    RobotPos pos{x,y,z,a,b,c,RobotTCP::UPLOAD};
    return pos;
}

void IntToSeamName(int n,string& str)
{
    string s = DecToBin(n);
    int count = 16- s.size();
    for(int i = 0; i<count;i++)
        s = "0" + s;
    string low = s.substr(11);
    string mid = s.substr(5,6);
    string high = s.substr(0,5);
    int l = BinToDec(low);
    int m = BinToDec(mid);
    int h = BinToDec(high);
    str = "seam"+to_string(h)+"_"+to_string(m)+"_"+to_string(l);

}

int SeamNameToInt(string& seamname)
{
    try
    {
        string s = seamname.substr(4);
        auto pos = s.find_first_of("_");

        string high = s.substr(0,pos);
        string s1 = s.substr(pos+1);
        auto pos1 = s1.find_first_of("_");
        if(pos1 == string::npos)
            pos1 = 0;
        string mid = s1.substr(0,pos1);
        string low = s1.substr(pos1+1);

        string h = DecToBin(stoi(high));
        int count = 0;
        count = 5- h.size();
        for(int i = 0; i<count;i++)
            h = "0" + h;

        string m = DecToBin(stoi(mid));
        count = 6- m.size();
        for(int i = 0; i<count;i++)
            m = "0" + m;

        string l = DecToBin(stoi(low));
        count = 5- l.size();
        for(int i = 0; i<count;i++)
            l = "0" + l;

        string data  = h+m+l;
        int r =  BinToDec(data);
        return r;
    }
    catch(...){
        throw "form error";
    }

}

void itranf(const cv::Mat &img,cv::Mat &red,int w,int h)
{
    cv::Mat res,bs= cv::Mat::zeros(1024,(1280-w)/2,CV_8UC1);
    cv::Mat bgr[3];
    cv::split(img,bgr);
    cv::resize(bgr[2],res,cv::Size(w,h));
    vector<cv::Mat> matrices= { bs,res(cv::Rect(0,0,w,1024)),bs,};
    cv::hconcat(matrices,red);
    return;
}

void ptranf(cv::Point2d &uv,int w)
{
    uv.x-= (1280-w)/2;
    uv.x*= 720.0/w;
    uv.y/= 2;
    return;
}
