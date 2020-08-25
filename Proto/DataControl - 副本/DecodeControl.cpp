#include "DecodeControl.h"
#include <QThread>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <fstream>
#include <fcntl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

DecodeControl::DecodeControl()
{

}




/// Read from Local File ,Gen Root
bool  DecodeControl::readFromLocal(std::string filename)
{
    std::fstream ff(filename,std::ios_base::in |  std::ios::binary);
    bool ret =  root.ParseFromIstream(&ff);

    ff.close();

    return ret;
}


bool DecodeControl::readFromLocalTxt(std::string filename)
{
    std::ifstream ff(filename,std::ios_base::in |  std::ios::binary);

    std::stringstream tmp;
    tmp << ff.rdbuf();
    std::string str = tmp.str();
    bool ret =  root.ParseFromString(str);

    ff.close();
    return ret;
}

bool DecodeControl::readFromLocalFd(std::string filename)
{
    int fd = open(filename.data(),O_RDONLY);
    if(fd <= 0){
    perror("fd open false");
        exit(0);
    }
    ::google::protobuf::io::FileInputStream input(fd);
    ::google::protobuf::io::CodedInputStream decoder(&input);

    decoder.SetTotalBytesLimit(128*1024*1024);//能解析的数据最大为128M
    bool ret = root.ParseFromCodedStream(&decoder) && decoder.ConsumedEntireMessage();
    //bool ret =  root.ParseFromFileDescriptor(fd);
    return ret;
}




bool DecodeControl::insert2SQL(std::string db_name)
{
    qDebug()<<"Sqlite3:"<<db_name.c_str();
    dbT1 = QSqlDatabase::addDatabase("QSQLITE");	//连接数据库
    dbT1.setDatabaseName(db_name.c_str());	                        //设置数据库名称
    dbT1.open();

    return dbT1.isValid();
}

bool DecodeControl::insertProtoData()
{
    QSqlQuery quert(dbT1);
    QString query_seamifo("insert into ");

    progress = 0.0;
    QString _temp;
    query_seamifo += "seaminfo";
    query_seamifo += "(seamname,motorPoss,motorPose,motorAngle1s,motorAngle2s,";
    query_seamifo += "motorAngle1e,motorAngle2e,";
    query_seamifo += "motor1speed,motor2speed,motor3speed,cadid,orderid";
    query_seamifo += ") values(:seamname,:motorPoss,:motorPose,:motorAngle1s,:motorAngle2s,:motorAngle1e,:motorAngle2e,:motor1speed,:motor2speed,:motor3speed,:cadid,:orderid)";
    log.clear();

    QString query_SEAMRELATE("insert into SEAMRELATE(relate1,spacing1,distance1,relate2,spacing2,distance2,seamid) values(:relate1,:spacing1,:distance1,:relate2,:spacing2,:distance2,:seamid)");

    QString query_framepos("insert into framepos(index1,pos,seamid) values(:index1,:pos,:seamid)");

    QString query_scanpos("insert into scanepos(index1,pos,seamid) values(:index1,:pos,:seamid)");

    QString query_cad("insert into cad(cadid,midpoint,basepoint) values(:cadid,:midpoint,:basepoint)");

    QString query_seamweldinfo("insert into seamweldinfo(seamid) values(:seamid)");

    QString query_weldparams("insert into weldparams(seamid) values(:seamid)");

    QString query_modifyScanePos("update scanepos set enabled=0   where seamid=(select max(seamid) from scanepos) and index1=0 or index1=(select   max(index1) from scanepos where seamid=(select max(seamid) from scanepos))");

    log.push_back("姿态组-->" +QString::number(root.zitai_size()));
    log.push_back(QString("时间:")+root.data().c_str());

    for(int i =0;i< root.zitai_size();i++){


        /// 焊缝名称
        ///
        progress = i * 100 / root.zitai_size() - 1;

        /// 检查Cad是否已经定义，如果定义就不插入
        _temp = "select count(*)  from cad where cadid=";
        _temp+=root.zitai(i).cadid().c_str();
        quert.exec(_temp);
        while (quert.next()) {
            int ret = quert.value(0).toInt();
            if(1 > ret ){
                quert.prepare(query_cad);
                quert.bindValue( ":cadid",root.zitai(i).cadid().c_str());
                auto std_spos = root.zitai(i).cadmidpos();
                quert.bindValue( ":midpoint",Frame::proto2PosStr(&std_spos).c_str());
                std_spos = root.zitai(i).cadbasepos();
                quert.bindValue( ":basepoint",Frame::proto2PosStr(&std_spos).c_str());
                quert.exec();
                if(!quert.lastError().text().isNull())
                log.push_back("cad表" + quert.lastError().text());
            }
        }




        /// han feng
        int _size = root.zitai(i).seam_vector_size();
        log.push_back("焊缝组-->" +QString::number(_size));

        for(int j =0;j<_size;j++){
            log.push_back("扫描组-->" +QString::number(root.zitai(i).seam_vector(j).scanepos_size()));
            log.push_back("龙骨组-->" +QString::number(root.zitai(i).seam_vector(j).scanepos_size()));

            progress += ( j /_size) * 30;

            quert.prepare(query_seamifo);
            std::string  seamname = root.zitai(i).seam_vector(j).seamname();
            quert.bindValue( ":seamname",seamname.c_str());
            quert.bindValue( ":motorPoss",root.zitai(i).pos_motor_start());
            quert.bindValue( ":motorPose",root.zitai(i).pos_motor_stop());
            quert.bindValue( ":motorAngle1s",root.zitai(i).angle1_motor_start());
            quert.bindValue( ":motorAngle2s",root.zitai(i).angle2_motor_start());
            quert.bindValue( ":motorAngle1e",root.zitai(i).angle1_motor_stop());
            quert.bindValue( ":motorAngle2e",root.zitai(i).angle2_motor_stop());
            quert.bindValue( ":motor1speed",(root.zitai(i).motor1speed()));
            quert.bindValue( ":motor2speed",(root.zitai(i).motor2speed()));
            quert.bindValue( ":motor3speed",(root.zitai(i).motor3speed()));
            quert.bindValue( ":cadid",      root.zitai(i).cadid().empty() ? "custom":root.zitai(i).cadid().data());
            quert.bindValue( ":orderid",      root.zitai(i).seam_vector(j).orderindex());

            quert.exec();

            if(!quert.lastError().text().isNull())
                log.push_back("焊缝表" + quert.lastError().text());



            quert.exec("select max(SEAMID) from SEAMINFO");
            int seamid_new = -1;
            while (quert.next()) {
              seamid_new = quert.value(0).toInt();
            }

            log.push_back("最新的焊缝编号:"+QString::number(seamid_new));
            quert.prepare(query_seamweldinfo);
            quert.bindValue( ":seamid",seamid_new);
            quert.exec();

            if(!quert.lastError().text().isNull())
                log.push_back("焊缝参数表:" + quert.lastError().text());

            quert.prepare(query_weldparams);
            quert.bindValue( ":seamid",seamid_new);
            quert.exec();
            if(!quert.lastError().text().isNull())
                log.push_back("焊接参数" + quert.lastError().text());

            //quert.value()
            /// 更新关联关系表
            quert.prepare(query_SEAMRELATE);

            quert.bindValue( ":relate1",root.zitai(i).seam_vector(j).bind1());
            quert.bindValue( ":spacing1",root.zitai(i).spacing1());
            quert.bindValue( ":distance1",root.zitai(i).distance1());
            quert.bindValue( ":relate2",root.zitai(i).seam_vector(j).bind2());
            quert.bindValue( ":spacing2",root.zitai(i).spacing2());
            quert.bindValue( ":distance2",root.zitai(i).distance2());
            quert.bindValue( ":seamid",seamid_new);

            quert.exec();
            if(!quert.lastError().text().isNull())
            log.push_back("关联关系表" + quert.lastError().text());

            for(int t =0;t<root.zitai(i).seam_vector(j).scanepos_size();t++){
                quert.prepare(query_scanpos);
                auto pos = root.zitai(i).seam_vector(j).scanepos(t);
                std::string s_pos = Frame::proto2PosStr(&pos);
                quert.bindValue(":index1",t);
                quert.bindValue(":pos", s_pos.c_str());
                quert.bindValue(":seamid",seamid_new);
                quert.exec();
                if(!quert.lastError().text().isNull())
                log.push_back("ScanePos表:" + quert.lastError().text());
            }

            quert.exec(query_modifyScanePos);
            if(!quert.lastError().text().isNull())
            log.push_back("ScanePos-Modify表:" + quert.lastError().text());


            for(int t =0;t<root.zitai(i).seam_vector(j).framepos_size();t++){
                quert.prepare(query_framepos);
                auto pos = root.zitai(i).seam_vector(j).framepos(t);
                std::string s_pos = Frame::proto2PosStr(&pos);
                quert.bindValue(":index1",t);
                quert.bindValue(":pos", s_pos.c_str());
                quert.bindValue(":seamid",seamid_new);
                quert.exec();
                if(!quert.lastError().text().isNull())
                log.push_back("FramePos表:" + quert.lastError().text());
            }


        }

    }
    progress = 100.00;
    log.push_back("*****************解码完成!*********************");
    return  true;

}

void DecodeControl::closeDB()
{
    dbT1.close();
}



