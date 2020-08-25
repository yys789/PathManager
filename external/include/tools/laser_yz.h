//激光跟踪(单相机三维算法)
#include <string>

bool initLaserYZ( std::string str,int mode) ;

bool laser_yz(float u, float v, float &y, float &z, int mode=0);
