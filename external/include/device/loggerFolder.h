#ifndef LOGGERFOLDER_H
#define LOGGERFOLDER_H
#include <iostream>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <boost/filesystem.hpp>

using namespace std;
using namespace boost;
namespace fs = boost::filesystem;

class loggerFolder
{
public:

    typedef std::shared_ptr<loggerFolder> Ptr;

    ~loggerFolder(){
        cout<<"loggerFolder destructor callled."<<endl;
    }

    loggerFolder(loggerFolder&)=delete;
    loggerFolder& operator=(const loggerFolder&)=delete;

    static Ptr getInstance(){
        if(m_Instance_ptr == nullptr){
            std::lock_guard<mutex> lck(m_mutex);
            if(m_Instance_ptr == nullptr){
                m_Instance_ptr = std::shared_ptr<loggerFolder>(new loggerFolder);
            }
        }
        return m_Instance_ptr;
    }

private:

    loggerFolder(){
        cout<<"loggerFolder costructor callled."<<endl;
    }

    static Ptr m_Instance_ptr;
    static std::mutex m_mutex;

public:

    std::string getCurrPath();
    bool checkDirectory();

    void createDirectory(const string& path) const;
    void createDirectories(const string& path) const;

    bool deleteFile(const std::string &file_name) const;
    bool deletePath(const std::string &path) const;

    bool createFile(const std::string &file_name) const;
    bool createFile(const std::string &path, const std::string &file_name);

    void rename(const std::string& from_p,const std::string& to_p) const;

public:

    std::map<std::string,std::string> mLogPath;
    std::map<std::string,std::string> mLogFilePath;
    std::map<std::string,std::string> mLogDirPath;

};


string getDate();
void readConfig(std::map<std::string,std::string>& logPath,const std::string& file);

#endif // LOGGERFOLDER_H
