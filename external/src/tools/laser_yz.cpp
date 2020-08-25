//激光跟踪(单相机三维算法)
#include "walgo/lasermodel3.h"
#include "walgo/3dutils.h"
#include <iostream>
#include <string>
#include <fstream>
using namespace std;
using namespace walgo;

static lasermodel3 lmodel[2]; // 跟踪用

bool initLaserYZ( string str,int mode)
{
//   std::ofstream ofs ;
//   ofs.open("pos45_yz.txt", std::ios::binary) ;
   pairmap<float> pm;
   std::map<int, std::shared_ptr<iopair<float>>>().swap(pm._map);
   if (!readDataLaser(str.c_str(), pm))
   {
       //cout << "loading CalibrateData failed" << endl;
       return false;
   }
   lmodel[mode].setData(pm);
   lmodel[mode].calibrate();
//   if(mode == 1)  // 用格点验证自身逻辑
//   {
//       auto it2 = pm._map.begin();
//       while(it2 != pm._map.end())
//       {
//           auto puv = it2->second->_img;
//           auto pyz = it2->second->_obj;
//           float y,z;
//           lmodel[1].imgToObj1(puv._u, puv._v, y, z);
//           cout<<y-pyz._y<<"\t"<<z-pyz._z<<endl;
//           it2++;
//       }
//   }
   return true ;
}

bool laser_yz(float u, float v, float &y, float &z,int mode)
{
    lmodel[mode].imgToObj1(u, v, y, z);
    return true;
}
