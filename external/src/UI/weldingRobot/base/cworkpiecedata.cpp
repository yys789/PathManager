#include "cworkpiecedata.h"
#include <algorithm>
#include "base/errorcode.h"
#include <boost/filesystem.hpp>
#include "tools.h"

bool comp(seamMark& sm1,seamMark& sm2)
{
    return sm1.orderID < sm2.orderID;
}

CWorkPieceData::CWorkPieceData()
{
    init();
}

CWorkPieceData::~CWorkPieceData(){}

void CWorkPieceData::init()
{
    release_pair_vec();
    release_map_pair();
    release_AllSeamPathMap();
}

void CWorkPieceData::release_pair_vec()
{
    vector<SensorDataPack>().swap(pair_vec); /// Erases MEMORY
    cout<<"[release_memory][OVER]"<<endl;
}

void CWorkPieceData::release_map_pair()
{
    auto it = map_pair.begin();
    while(it != map_pair.end())
    {
        vector<SensorDataPack>().swap(it->second);
        it++;
    }
    std::map<int,vector<SensorDataPack>>().swap(map_pair);
    cout<<"[release_memory1][OVER]"<<endl;
}

void CWorkPieceData::release_AllSeamPathMap()
{
    auto it = AllSeamPathMap.begin();
    while(it != AllSeamPathMap.end())
    {
        std::vector<RobotPos>().swap(it->second);
        it++;
    }
    std::map<int,std::vector<RobotPos>>().swap(AllSeamPathMap);
    cout<<"[release_AllSeamPathMap][OVER]"<<endl;
}

void CWorkPieceData::saveImagesAll()
{
    try
    {
        auto it = map_pair.begin();
        while( it != map_pair.end())
        {
            string path = "testLog/"+ m_sCADID + "_" + seamMap[it->first] + "_" + sCurrTime  ;
            if ( !boost::filesystem::exists(path)) {
                boost::filesystem::create_directories(path) ;
            }
            if(it->second.size() == 0)
            {
                throw ERR_CAMERA_TAKE_PICTURES_FAILED;
            }
            ofstream fs;
            fs.open(path + "/scanPath.txt");
            for(int i = 0; i < it->second.size(); i++)
            {
                saveImg( it->second[i].m, path, to_string(i) + ".png" );
                fs<<i<<" "<<it->second[i].r.toStr()<<endl;
            }
            fs.close();
            it++;
        }
        cout<<"[saveImages][OVER]"<<endl;
    }
    catch(...)
    {
        throw ERR_FUNC_SAVE_IMAGES_FAILED;
    }
}

void CWorkPieceData::saveImages()
{
    try
    {
        if(pair_vec.size() == 0)
            throw ERR_CAMERA_TAKE_PICTURES_FAILED;

        string path = "testLog/"+ m_sCADID + "_" + to_string(sMark.seamID) + "_" + sCurrTime;
        if ( !boost::filesystem::exists(path)) {
            boost::filesystem::create_directories(path) ;
        }

        ofstream fs;
        fs.open(path + "/scanPath.txt");
        for(int i = 0; i < pair_vec.size(); i++)
        {
            saveImg( pair_vec[i].m, path, to_string(i) + ".png" );
            fs<<pair_vec[i].r.toStr()<<endl;
        }
        fs.close();

        cout<<"[saveImages1][OVER]"<<endl;
    }
    catch(...)
    {
        throw ERR_FUNC_SAVE_IMAGES_FAILED;
    }
}

