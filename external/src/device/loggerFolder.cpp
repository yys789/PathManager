#include "device/loggerFolder.h"
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <time.h>

using namespace std;

loggerFolder::Ptr loggerFolder::m_Instance_ptr = nullptr;
std::mutex loggerFolder::m_mutex;

bool loggerFolder::checkDirectory()
{
    mLogPath.clear();
    mLogDirPath.clear();
    mLogFilePath.clear();
    readConfig(mLogPath,"etc/logPath.info");
    string currDate = getDate();
    auto it = mLogPath.begin();
    while(it != mLogPath.end())
    {
        string module = it->first;
        string path = it->second;
        if(module == "CORRECTION" || module == "SEAM")
        {
            path = path + "/" + currDate;
            createDirectories(path);
            mLogDirPath.insert(make_pair(it->first,path));
        }
        else{
            path = path + "/" + currDate + ".log";
            createFile(path);
            mLogFilePath.insert(make_pair(it->first,path));
        }
        it++;
    }
}

bool loggerFolder::createFile(const std::string &path, const std::string &file_name)
{
    if(!fs::exists(path))
        createDirectories(path);
    fstream  file_out;
    file_out.open(path+ "/" +file_name,ios::app);
    file_out.close();
}

bool loggerFolder::createFile(const std::string &file_name) const
{
    string path;
    int pos = file_name.find_last_of("/");
    if(pos != string::npos)
    {
        path = file_name.substr(0,pos);
        cout<<path<<endl;
        if(!fs::exists(path))
            createDirectories(path);
    }
    fstream  file_out;
    file_out.open(file_name,ios::app);
    file_out.close();
}


bool loggerFolder::deleteFile(const std::string &file_name) const
{
    return fs::remove(file_name);
}

bool loggerFolder::deletePath(const std::string &file_name) const
{
    return fs::remove_all(file_name);
}

void loggerFolder::createDirectory(const string& path) const
{
    if(fs::exists(path))
        return;
    fs::create_directory(path);
}

void loggerFolder::createDirectories(const string& path) const
{
    if(fs::exists(path))
        return;
    fs::create_directories(path);
}


string loggerFolder::getCurrPath()
{
    return fs::current_path().string();
}

void loggerFolder::rename(const string& from_p,const string& to_p) const
{
    fs::rename(from_p,to_p);
}

string getDate()
{
    time_t timep;
    time(&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d",localtime(&timep) );
    return tmp;
}

void readConfig(std::map<std::string,std::string>& mPath,const std::string& file_name)
{
    ifstream fin(file_name,ios::in);
    string line;
    while(getline(fin,line))
    {
        if(strlen(line.c_str()) == 0)
            continue;
        string temp = line;
        int pos = temp.find_first_of('=');
        string key = temp.substr(0,pos);
        string value = temp.substr(pos+1);
        mPath.insert(make_pair(key,value));
    }
    auto it = mPath.begin();
    while(it != mPath.end())
    {
        cout<<it->first<<" : "<<it->second<<endl;
        it++;
    }
}

