#include "device/mysqliteapi.h"
#include <string>
#include <initializer_list>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include "base/errorcode.h"

using namespace std;

mySqliteApi::mySqliteApi()
    :_db_ptr{nullptr},
      zErrMsg{nullptr},
      m_iRows{0}
{
    openDB();
}

void mySqliteApi::openDB()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        int rc = 0;
        string DBName = "blade.db";
        rc = sqlite3_open(DBName.c_str(),&_db_ptr);
        if(rc)
        {
            cout<<"Can't open database: "<< sqlite3_errmsg(_db_ptr)<<endl;
            throw ERR_DB_OPEN_FAILED;
        }else{
            cout<<"open "<<DBName.c_str()<<" success."<<endl;
        }
    }
    catch(...)
    {
        sqlite3_close(_db_ptr);
        throw ERR_DB_OPEN_FAILED;
    }

}

void mySqliteApi::closeDB()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;
    sqlite3_close(_db_ptr);
}

void mySqliteApi::EXEC_SQL()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        zErrMsg = nullptr;
        m_iRows = 0;
        char** pResult = nullptr;
        int iRow = 0;
        int iColumn = 0;
        int rc = 0;
        cout<<"[SQL]:"<<m_strSql.c_str()<<endl;
        rc = sqlite3_get_table(_db_ptr,m_strSql.c_str(),&pResult,&iRow,&iColumn,&zErrMsg);
        cout<<"rc = "<<rc<<endl;
        if(rc != SQLITE_OK)
        {
            cout<<"SQL exec failed : "<<zErrMsg<<endl;
            sqlite3_free(zErrMsg);
            throw ERR_EXEC_SQL_GET_RESULT_FAILED;
        }else{
            m_vResult.clear();
            if(iRow > 0)
                m_iRows = iRow;
            cout<<"get result OK , rows : "<< iRow<<endl;
            int iIndex = iColumn;
            for(int i = 0; i<iRow; i++)
            {
                map<string,string> record;
                for(int j = 0; j<iColumn; j++)
                {
                    string key = pResult[j];
                    string value;
                    if(pResult[iIndex]==nullptr)
                        value = "NULL";
                    else
                        value = pResult[iIndex];
                    record[key] = value;
                    iIndex++;
                }
                m_vResult.push_back(record);
            }
        }
        sqlite3_free_table(pResult);
        cout<<"SQL exec success !"<<endl;
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::exec()
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        int rc = sqlite3_exec(_db_ptr, m_strSql.c_str(), callback, 0, &zErrMsg);
        if( rc != SQLITE_OK ){
            fprintf(stderr, "SQL error: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
            throw ERR_EXEC_SQL_GET_RESULT_FAILED;
        }else{
            fprintf(stdout, "exec successfully\n");
        }
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }

}

void mySqliteApi::setSQL(string strSql)
{
    m_strSql = strSql;
}

int mySqliteApi::callback(void* data, int ncols, char** values, char** headers)
{
    int i;
    int len =0;
    int ll=0;
    for(i=0; i < ncols; i++)
    {
        if(strlen(headers[i])>len)
            len = strlen(headers[i]);
    }
    //    map<string,string> recoed;
    for(i=0; i < ncols; i++)
    {
        ll = len-strlen(headers[i]);
        while(ll)
        {
            fprintf(stdout," ");
            --ll;
        }
        fprintf(stdout, "%s: %s\n", headers[i], values[i]);
        //        record[headers[i]] = values[i];
    }
    //    m_vResult.push_back(record);
    fprintf(stdout, "\n");
    return 0;
}

void mySqliteApi::print_result()
{
    cout<<"print result : "<<endl;
    for(auto& row:m_vResult)
    {
        auto it = row.begin();
        while(it != row.end())
        {
            cout<<it->first<<" : "<<it->second<<endl;
            it++;
        }
    }
}

void mySqliteApi::selectSQL(const string tableName)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "select * from " + tableName;
        if(!m_strSql.empty())
        {
            m_strSql += ";";
        }
        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::selectSQL1(const string tableName,initializer_list<string> condition)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "select * from " + tableName + " where ";
        int i = 0;
        int count = condition.size();
        for(auto& con:condition)
        {
            if(i%2 == 0)
                m_strSql += con + " ";
            else
                m_strSql += "= " + con + " ";
            i++;
            if(i%2 == 0 && i < count)
                m_strSql += "and ";
        }
        if(!m_strSql.empty())
        {
            m_strSql += ";";
        }
        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::deleteSQL(const string tableName)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "delete from " + tableName;
        if(!m_strSql.empty())
        {
            m_strSql += ";";
        }
        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::deleteSQL1(const string tableName,initializer_list<string> condition)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "delete from " + tableName + " where ";
        int i = 0;
        int count = condition.size();
        for(auto& con:condition)
        {
            if(i%2 == 0)
                m_strSql += con + " ";
            else
                m_strSql += "= " + con + " ";
            i++;
            if(i%2 == 0 && i < count)
                m_strSql += "and ";
        }
        if(!m_strSql.empty())
        {
            m_strSql += ";";
        }
        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::selectSQL2(const string tableName,initializer_list<string> result,initializer_list<string> condition)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "select ";
        int i = 0;
        int count = result.size();
        for(auto& str:result)
        {
            m_strSql += str + " ";
            i++;
            if(i != count)
            {
                m_strSql += ",";
            }
        }
        m_strSql += "from " + tableName + " where ";

        i = 0;
        count = condition.size();
        for(auto& con:condition)
        {
            if(i%2 == 0)
                m_strSql += con + " ";
            else
                m_strSql += "= " + con + " ";
            i++;
            if(i%2 == 0 && i < count)
                m_strSql += "and ";
        }

        if(!m_strSql.empty())
        {
            m_strSql += ";";
        }
        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::selectSQLOrder(const string tableName,initializer_list<string> result,initializer_list<string> condition,initializer_list<string>orderCondition)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "select ";
        int i = 0;
        int count = result.size();
        for(auto& str:result)
        {
            m_strSql += str + " ";
            i++;
            if(i != count)
            {
                m_strSql += ",";
            }
        }
        m_strSql += "from " + tableName + " where ";

        i = 0;
        count = condition.size();
        for(auto& con:condition)
        {
            if(i%2 == 0)
                m_strSql += con + " ";
            else
                m_strSql += "= " + con + " ";
            i++;
            if(i%2 == 0 && i < count)
                m_strSql += "and ";
        }

        m_strSql += " order by ";
        for(auto& con:orderCondition)
        {
            m_strSql += con + " ";
        }

        if(!m_strSql.empty())
        {
            m_strSql += ";";
        }

        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::selectSQLAllOrder(const string tableName,initializer_list<string> condition,initializer_list<string>orderCondition)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "select * ";
        m_strSql += "from " + tableName + " where ";

        int i = 0;
        int count = condition.size();
        for(auto& con:condition)
        {
            if(i%2 == 0)
                m_strSql += con + " ";
            else
                m_strSql += "= " + con + " ";
            i++;
            if(i%2 == 0 && i < count)
                m_strSql += "and ";
        }

        m_strSql += " order by ";
        for(auto& con:orderCondition)
        {
            m_strSql += con + " ";
        }

        if(!m_strSql.empty())
        {
            m_strSql += ";";
        }

        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}


void mySqliteApi::updateSQL(const string tableName,initializer_list<string> result,initializer_list<string> condition)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "update " + tableName + " set ";
        int i = 0;
        int count = result.size();
        for(auto& str:result)
        {
            if(i%2 == 0)
                m_strSql += str + " = ";
            else
                m_strSql += str + " ";
            i++;
            if(i%2 == 0 && i != count)
                m_strSql += ",";
        }

        m_strSql += "where ";
        i = 0;
        count = condition.size();
        for(auto& con:condition)
        {
            if(i%2 == 0)
                m_strSql += con + " ";
            else
                m_strSql += "= " + con + " ";
            i++;
            if(i%2 == 0 && i < count)
                m_strSql += "and ";
        }

        if(!m_strSql.empty())
            m_strSql += ";";
        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::deleteSQL(const string tableName,initializer_list<string> condition)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "delete from " + tableName + " where ";

        int i = 0;
        int count = condition.size();
        for(auto& con:condition)
        {
            if(i%2 == 0)
                m_strSql += con + " ";
            else
                m_strSql += "= " + con + " ";
            i++;
            if(i%2 == 0 && i < count)
                m_strSql += "and ";
        }

        if(!m_strSql.empty())
            m_strSql += ";";
        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::insertSQL(const string tableName,initializer_list<string> values)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "insert into ";
        m_strSql += tableName;
        m_strSql += " values ( ";
        int i = 0;
        int count = values.size();
        for(auto& v:values)
        {
            m_strSql += v;
            i++;
            if(i != count)
                m_strSql += ",";
        }

        m_strSql += ")";
        if(!m_strSql.empty())
            m_strSql += ";";
        EXEC_SQL();
    }
    catch(...)
    {
        throw ERR_EXEC_SQL_GET_RESULT_FAILED;
    }
}

void mySqliteApi::getLastInsertRowId(const string tableName)
{
    std::lock_guard<std::recursive_mutex> lck(_mtx) ;

    try
    {
        m_strSql = "select last_insert_rowid() as rowid from " + tableName;
        if(!m_strSql.empty())
        {
            m_strSql += ";";
        }
        EXEC_SQL();
    }
    catch(...)
    {

    }
}
