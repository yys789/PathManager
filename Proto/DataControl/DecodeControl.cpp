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
    QString query_SEAMRELATE("insert into SEAMRELATE(relate1,spacing1,distance1,relate2,spacing2,distance2,seamid,seflag) values(:relate1,:spacing1,:distance1,:relate2,:spacing2,:distance2,:seamid,:seflag)");

    QString query_framepos("insert into framepos(index1,pos,seamid,weld,height,enabled) values(:index1,:pos,:seamid,:weld,:height,:enabled)");

    QString query_scanpos("insert into scanepos(index1,pos,seamid,camera) values(:index1,:pos,:seamid,:camera)");

    QString query_cad("insert into cad(cadid,midpoint,basepoint,safeloc,safeangle1) values(:cadid,:midpoint,:basepoint,:safeloc,:safeangle1)");

    QString query_seamweldinfo("insert into seamweldinfo(seamid,weldorder,offset,baseoffset) values(:seamid,:weldorder,:offset,:baseoffset)");

    QString query_weldparams("insert into weldparams(seamid,s_w,i_w,v_w,t_end,i_end,v_end,range,axis_x,axis_y) values(:seamid,:s_w,:i_w,:v_w,:t_end,:i_end,:v_end,:range,:axis_x,:axis_y)");

    QString u_ScanePos("update scanepos set enabled=0   where seamid=(select max(seamid) from scanepos) and index1=0 or index1=(select   max(index1) from scanepos where seamid=(select max(seamid) from scanepos))");


    QString addSeaminfo("update seaminfo set MAIN=?,RANK=?,SEAMTYPE=?,DEPARTLEN=?,MIDPOINT=?,MIDDLESTART=?,OPTION=?,AUTOFIT=?,TRACING=?,WIDTH=?,HEIGHT=?,CORNERLEN=?,CORNERANGLE=?,ENABLED=? ");
    addSeaminfo+= "where seamid=(select max(seamid) from seaminfo)";

    QString query_seamifo("insert into ");
    query_seamifo += "seaminfo";
    query_seamifo += "(seamname,motorPoss,motorPose,motorAngle1s,motorAngle2s,";
    query_seamifo += "motorAngle1e,motorAngle2e,";
    query_seamifo += "motor1speed,motor2speed,motor3speed,cadid,orderid";
    query_seamifo += ") values(:seamname,:motorPoss,:motorPose,:motorAngle1s,:motorAngle2s,:motorAngle1e,:motorAngle2e,:motor1speed,:motor2speed,:motor3speed,:cadid,:orderid)";

    mapInsertQuery.insert("SEAMRELATE",query_SEAMRELATE);
    mapInsertQuery.insert("FRAMEPOS"  ,query_framepos);
    mapInsertQuery.insert("SCANEPOS"  ,query_scanpos);
    mapInsertQuery.insert("CAD"       ,query_cad);
    mapInsertQuery.insert("SEAMWELDINFOS",query_seamweldinfo);
    mapInsertQuery.insert("WELDPARAMS",query_weldparams);
    mapInsertQuery.insert("SEAMINFO",   query_seamifo);

    mapUpdateQuery.insert("SCANEPOS_ENABLED",u_ScanePos);     // 修改某一项的数据
    mapUpdateQuery.insert("SEAMINFO_MODIFY",addSeaminfo);     // 修改某一项的数据

    mapGetQuery.insert("SEAMINFO_SEAMID_MAX","select max(SEAMID) from SEAMINFO");
    mapGetQuery.insert("CAD_CADID_COUNT","select count(*)  from cad where cadid=");


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
    dbT1 = QSqlDatabase::addDatabase("QSQLITE",QString::number(connectName));	//连接数据库
    dbT1.setDatabaseName(db_name.c_str());	                        //设置数据库名称
    dbT1.open();

    connectName++;
    return dbT1.isValid();
}

bool DecodeControl::insertProtoData()
{
    // QSqlQuery quert(dbT1);

    //queryP.reset(new QSqlQuery(dbT1));

    progress = 0.0;

    log.clear();

    log.push_back(root.date().c_str()); //输入创建时间

    auto rng = dbT1.transaction(); //开启事务
    log.push_back(rng ? "开启数据库事务成功!" : "开启数据库事务失败!");
    log.push_back(QString("时间:")+root.date().c_str());


    /* 异步检查并合并CAD表 */
    auto cad = std::async(std::launch::async,[this](){
        std::lock_guard<mutex> lg(g_mtx);
        QScopedPointer<QSqlQuery> queryP;
        queryP.reset(new QSqlQuery(dbT1));
        for(int index =0;index< root.cad_size();index++){
            // 检查Cad是否已经定义，如果定义就不插入
            QString _temp = mapGetQuery["CAD_CADID_COUNT"] + root.cad(index).cadid().c_str();
            queryP.data()->exec(_temp);
            while (queryP.data()->next()) {
                int ret = queryP.data()->value(0).toInt();
                if(1 > ret ){   // 此处代表没有找到CAD表重复定义的数据
                    queryP.data()->prepare(mapInsertQuery["CAD"]);
                    queryP.data()->bindValue( ":cadid",root.seams(index).cadid().c_str());
                    auto std_spos = root.cad(index).cadmidpoint();
                    queryP.data()->bindValue( ":midpoint",std_spos);
                    auto tyty = root.cad(index).cadbasepos();
                    auto safeloc = root.cad(index).safeloc();
                    auto safeangle1 = root.cad(index).safeangle1();
                    queryP.data()->bindValue( ":basepoint",Frame::proto2PosStr(&tyty).c_str());
                    queryP.data()->bindValue( ":safeloc",safeloc);
                    queryP.data()->bindValue( ":safeangle1",safeangle1);
                    queryP.data()->exec();
                    if(!queryP.data()->lastError().text().isEmpty())
                    log.push_back("cad表" + queryP.data()->lastError().text());
                }
            }
        }

    });
    cad.get();

    for(int i =0;i< root.seams_size();i++){

        progress = i * 100 / root.seams_size(); // 初始化进度数据



        /* 异步焊缝信息表 */
        auto seamInfo = std::async(std::launch::async,[this](int index){
            std::lock_guard<mutex> lg(g_mtx);
            QScopedPointer<QSqlQuery> queryP;
            queryP.reset(new QSqlQuery(dbT1));
            queryP.data()->exec(mapGetQuery["SEAMINFO_SEAMID_MAX"]);
            while (queryP.data()->next()) {
                new_seamid.exchange( queryP.data()->value(0).toInt() + 1);
            }
            if(!queryP.data()->lastError().text().isEmpty())
                log.push_back("获取最新的焊缝异常:" + queryP.data()->lastError().text());


            log.push_back("最新的焊缝编号:"+QString::number(new_seamid.load()));


            queryP.data()->prepare(mapInsertQuery["SEAMINFO"]);
            std::string  seamname = root.seams(index).seamname();
            queryP.data()->bindValue( ":seamname",seamname.c_str());
            queryP.data()->bindValue( ":motorPoss",root.seams(index).machine().pos_motor_start());
            queryP.data()->bindValue( ":motorPose",root.seams(index).machine().pos_motor_stop());
            queryP.data()->bindValue( ":motorAngle1s",root.seams(index).machine().angle1_motor_start());
            queryP.data()->bindValue( ":motorAngle2s",root.seams(index).machine().angle2_motor_start());
            queryP.data()->bindValue( ":motorAngle1e",root.seams(index).machine().angle1_motor_stop());
            queryP.data()->bindValue( ":motorAngle2e",root.seams(index).machine().angle2_motor_stop());
            queryP.data()->bindValue( ":motor1speed",(root.seams(index).machine().motor1speed()));
            queryP.data()->bindValue( ":motor2speed",(root.seams(index).machine().motor2speed()));
            queryP.data()->bindValue( ":motor3speed",(root.seams(index).machine().motor3speed()));
            queryP.data()->bindValue( ":cadid",   root.seams(index).cadid().empty() ? "custom":root.seams(index).cadid().data());
            queryP.data()->bindValue( ":orderid", root.seams(index).orderindex());

            queryP.data()->exec();

            if(!queryP.data()->lastError().text().isEmpty())
                log.push_back("焊缝表" + queryP.data()->lastError().text());

            queryP.data()->prepare(mapUpdateQuery["SEAMINFO_MODIFY"]);
            queryP.data()->addBindValue( root.seams(index).modify().main());
            queryP.data()->addBindValue( root.seams(index).modify().rank());
            queryP.data()->addBindValue( root.seams(index).modify().seamtype().c_str());
            queryP.data()->addBindValue( root.seams(index).modify().departlen());
            queryP.data()->addBindValue( root.seams(index).modify().midpoint());
            queryP.data()->addBindValue( root.seams(index).modify().middlestart());
            queryP.data()->addBindValue( root.seams(index).modify().option1().c_str());
            queryP.data()->addBindValue( root.seams(index).modify().autofit());
            queryP.data()->addBindValue( root.seams(index).modify().tracing());
            queryP.data()->addBindValue( root.seams(index).modify().width());
            queryP.data()->addBindValue( root.seams(index).modify().height());
            queryP.data()->addBindValue( root.seams(index).modify().width());
            queryP.data()->addBindValue( root.seams(index).modify().width());
            queryP.data()->addBindValue( root.seams(index).modify().enabled());

            queryP.data()->exec();
            if(!queryP.data()->lastError().text().isEmpty())
                log.push_back("附加seaminfo" + queryP.data()->lastError().text());
        },i);

        seamInfo.get();     // 必须在此处阻塞以便获取最新的seamid

        /* 异步更新龙骨 */
        auto framepos = std::async(std::launch::async,[this](int index){
            std::lock_guard<mutex> lg(g_mtx);
            QScopedPointer<QSqlQuery> queryP;
            queryP.reset(new QSqlQuery(dbT1));
            for(int i = 0;i<root.seams(index).framepos_size();i++){
                queryP.data()->prepare(mapInsertQuery["FRAMEPOS"]);
                auto pos = root.seams(index).framepos(i).pos();
                queryP.data()->bindValue(":index1",i);
                queryP.data()->bindValue(":pos",Frame::proto2PosStr(&pos).c_str());
                queryP.data()->bindValue(":seamid",new_seamid.load());
                auto weld = root.seams(index).framepos(i).weld();
                auto height = root.seams(index).framepos(i).height();
                queryP.data()->bindValue(":weld",weld);
                queryP.data()->bindValue(":height",height);
                queryP.data()->bindValue(":angle1",QVariant(root.seams(index).framepos(i).angle1()));
                queryP.data()->bindValue(":angle2",QVariant(root.seams(index).framepos(i).angle2()));
                queryP.data()->bindValue(":angle3",QVariant(root.seams(index).framepos(i).angle3()));
                queryP.data()->bindValue(":enabled",root.seams(index).framepos(i).enabled());
                queryP.data()->exec();
                if(!queryP.data()->lastError().text().isEmpty())
                log.push_back("FramePos表:" + queryP.data()->lastError().text());
            }

        },i);

        framepos.get();

        /* 异步更新ScanePos */
        auto scanepos = std::async(std::launch::async,[this](int index){
            std::lock_guard<mutex> lg(g_mtx);
            QScopedPointer<QSqlQuery> queryP;
            queryP.reset(new QSqlQuery(dbT1));
            for(int i = 0;i<root.seams(index).scanepos_size();i++){
                queryP.data()->prepare(mapInsertQuery["SCANEPOS"]);
                auto pos = root.seams(index).scanepos(i).pos();
                queryP.data()->bindValue(":index1",i);
                queryP.data()->bindValue(":pos",Frame::proto2PosStr(&pos).c_str());
                queryP.data()->bindValue(":seamid",new_seamid.load());
                queryP.data()->bindValue(":camera",root.seams(index).scanepos(i).camera());
                queryP.data()->bindValue(":angle1",QVariant(root.seams(index).scanepos(i).angle1()));
                queryP.data()->bindValue(":angle2",QVariant(root.seams(index).scanepos(i).angle2()));
                queryP.data()->bindValue(":angle3",QVariant(root.seams(index).scanepos(i).angle3()));
                queryP.data()->exec();
                if(!queryP.data()->lastError().text().isEmpty())
                    log.push_back("ScanePos表:" + queryP.data()->lastError().text());

                queryP.data()->exec(mapUpdateQuery["SCANEPOS_ENABLED"]);
                if(!queryP.data()->lastError().text().isEmpty())
                log.push_back("ScanePos表:设置enabled失败!\n" + queryP.data()->lastError().text());
            }


        },i);

        scanepos.get();

        /* 异步更新关联关系表 */
        auto relate = std::async(std::launch::async,[this](int index){
            std::lock_guard<mutex> lg(g_mtx);
            QScopedPointer<QSqlQuery> queryP;
            queryP.reset(new QSqlQuery(dbT1));
            queryP.data()->prepare(mapInsertQuery["SEAMRELATE"]);

            queryP.data()->bindValue( ":relate1",root.seams(index).relate().bind1());
            queryP.data()->bindValue( ":spacing1",root.seams(index).relate().spacing1());
            queryP.data()->bindValue( ":distance1",root.seams(index).relate().distance1());
            queryP.data()->bindValue( ":relate2",root.seams(index).relate().bind2());
            queryP.data()->bindValue( ":spacing2",root.seams(index).relate().spacing2());
            queryP.data()->bindValue( ":distance2",root.seams(index).relate().distance2());
            queryP.data()->bindValue( ":seamid",new_seamid.load());
            queryP.data()->bindValue( ":seamid",root.seams(index).relate().seflag());

            queryP.data()->exec();
            if(!queryP.data()->lastError().text().isEmpty())
            log.push_back("关联关系表" + queryP.data()->lastError().text());
        },i);
        relate.get();


        /* 异步焊接参数表 */
        auto weldPara = std::async(std::launch::async,[this](int index){
            std::lock_guard<mutex> lg(g_mtx);
            QScopedPointer<QSqlQuery> queryP;
            queryP.reset(new QSqlQuery(dbT1));
            queryP.data()->prepare(mapInsertQuery["WELDPARAMS"]);
            queryP.data()->bindValue( ":seamid",new_seamid.load());
            queryP.data()->bindValue( ":s_w",root.seams(index).weldpara().s_w());
            queryP.data()->bindValue( ":i_w",root.seams(index).weldpara().i_w());
            queryP.data()->bindValue( ":v_w",root.seams(index).weldpara().v_w());
            queryP.data()->bindValue( ":t_end",root.seams(index).weldpara().t_end());
            queryP.data()->bindValue( ":i_end",root.seams(index).weldpara().i_end());
            queryP.data()->bindValue( ":v_end",root.seams(index).weldpara().v_end());
            queryP.data()->bindValue( ":range",root.seams(index).weldpara().range());
            queryP.data()->bindValue( ":axis_x",root.seams(index).weldpara().axis_x());
            queryP.data()->bindValue( ":axis_y",root.seams(index).weldpara().axis_y());
            queryP.data()->exec();
            if(!queryP.data()->lastError().text().isEmpty())
            log.push_back("WELDPARAMS:" + queryP.data()->lastError().text());
        },i);
        weldPara.get();

        /* 异步焊缝焊接信息表 */
        auto weldInfo = std::async(std::launch::async,[this](int index){
            std::lock_guard<mutex> lg(g_mtx);
            QScopedPointer<QSqlQuery> queryP;
            queryP.reset(new QSqlQuery(dbT1));
            queryP.data()->prepare(mapInsertQuery["SEAMWELDINFOS"]);
            queryP.data()->bindValue( ":seamid",new_seamid.load());
            queryP.data()->bindValue( ":weldorder",root.seams(index).weldinfo().weldorder());
            auto pp = root.seams(index).weldinfo().offset();
            auto ppb = root.seams(index).weldinfo().baseoffset();
            queryP.data()->bindValue( ":offset",Frame::proto2PosStr(&pp).c_str());
            queryP.data()->bindValue( ":baseoffset",Frame::proto2PosStr(&pp).c_str());
            queryP.data()->bindValue( ":s_w",root.seams(index).weldinfo().weldorder());
            queryP.data()->exec();
            if(!queryP.data()->lastError().text().isEmpty())
            log.push_back("SEAMWELDINFOS:" + queryP.data()->lastError().text());
        },i);
        weldInfo.get();

    }

    auto edg = dbT1.commit(); //开启事务
    log.push_back(edg ? "数据库事务处理成功!" : "数据库事务处理失败!");

    progress = 100.00;
    log.push_back("解码完成!--------------------------");
    return  true;

}

void DecodeControl::closeDB()
{
    QSqlDatabase::removeDatabase(QString::number(connectName-1));
    dbT1.close();
}



