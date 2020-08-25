#ifndef MYSQLITEAPI_H
#define MYSQLITEAPI_H
#include <sqlite3.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <initializer_list>
#include <vector>
#include <map>

#include <thread>
#include <mutex>

using namespace std;

enum DBACTION{SELECT,UPDATE,DELETE,INSERT};

class mySqliteApi
{
public:
    mySqliteApi();

public:
    void openDB();
    void closeDB();

    void deleteSQL(const string);
    void deleteSQL1(const string,initializer_list<string>);
    void selectSQL(const string);
    void selectSQL1(const string,initializer_list<string>);
    void selectSQL2(const string,initializer_list<string>,initializer_list<string>);
    void selectSQLOrder(const string,initializer_list<string>,initializer_list<string>,initializer_list<string>);
    void selectSQLAllOrder(const string,initializer_list<string>,initializer_list<string>);
    void updateSQL(const string,initializer_list<string>,initializer_list<string>);
    void deleteSQL(const string,initializer_list<string>);
    void insertSQL(const string,initializer_list<string>);
    void EXEC_SQL();
    void print_result();
    void exec();
    static int callback(void* data, int ncols, char** values, char** headers);
    void setSQL(string);
    void getLastInsertRowId(const string);
public:
    std::string m_strSql;
    vector<map<string,string>> m_vResult;
    int m_iRows;

private:
    std::recursive_mutex _mtx ;
    sqlite3* _db_ptr;
    char* zErrMsg;

};

#endif // MYSQLITEAPI_H
