#include "InputControl.h"
#include <QProcess>

#ifdef Q_OS_LINUX
    #include <sys/stat.h>
    #include <unistd.h>
#endif

#include <fcntl.h>
#include <sys/types.h>


InputControl::InputControl()
{


}

void CreatDir(){
    #ifdef Q_OS_LINUX
        ::umask(0);     /// 创建文件夹如果没有这个，会出现权限问题  无法读写
        ::mkdir("Gen",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    #elif defined (Q_OS_WIN)
        QProcess process;
        process.setProgram("cmd");
        QStringList argument;
        argument<<"/c"<<"md " <<"Gen";
        process.setArguments(argument);
        process.start();
        // Wait for it to start
        process.waitForStarted();
        process.waitForFinished();//等待程序关闭
        QString temp=QString::fromLocal8Bit(process.readAllStandardOutput()); //程序输出信息
        if(!temp.isEmpty()) qDebug()<<temp;
    #endif
}



string InputControl::CreatSerial(RepeatData &data)
{
    //GOOGLE_PROTOBUF_VERIFY_VERSION;
    frame::Frame root;
    root.set_date(data.date);       //获取时间


    for(auto j:data.cad){
        auto cad = root.add_cad();
        cad->set_cadid(j.cadid);
        cad->set_safeangle1(j.safeAngle1);
        cad->set_cadmidpoint(j.cadMidPoint);
        cad->set_safeloc(j.safeLoc);

        cad->mutable_cadbasepos()->set_x(j.cadBasePos.x);
        cad->mutable_cadbasepos()->set_y(j.cadBasePos.y);
        cad->mutable_cadbasepos()->set_z(j.cadBasePos.z);
        cad->mutable_cadbasepos()->set_a(j.cadBasePos.a);
        cad->mutable_cadbasepos()->set_b(j.cadBasePos.b);
        cad->mutable_cadbasepos()->set_c(j.cadBasePos.c);
    }


    for(auto j:data.seams){
        auto adedseam = root.add_seams();
        adedseam->set_cadid(j.cadid);
        adedseam->set_seamname(j.seamName);
        adedseam->set_seamindex(j.seamID);
        adedseam->set_orderindex(j.orderIndex);

        /***********************变位机*********************************/
        auto machine = adedseam->mutable_machine();
        machine->set_motor1speed(j.machine.motor1speed);
        machine->set_motor2speed(j.machine.motor2speed);
        machine->set_motor3speed(j.machine.motor3speed);
        machine->set_angle1_motor_start(j.machine.angle1_motor_start);
        machine->set_angle1_motor_stop(j.machine.angle1_motor_stop);
        machine->set_angle2_motor_start(j.machine.angle2_motor_start);
        machine->set_angle2_motor_stop(j.machine.angle2_motor_stop);
        machine->set_pos_motor_start(j.machine.pos_motor_start);
        machine->set_pos_motor_stop(j.machine.pos_motor_stop);

        /***********************seaminfo******************************/
        auto modify = adedseam->mutable_modify();
        modify->set_main(j.main);
        modify->set_rank(j.rank);
        modify->set_width(j.width);
        modify->set_height(j.height);
        modify->set_option1(j.option);
        modify->set_autofit(j.autoFit);
        modify->set_tracing(j.tracing);
        modify->set_enabled(j.enabled);
        modify->set_midpoint(j.midPoint);
        modify->set_seamtype(j.seamType);
        modify->set_cornerlen(j.CORNERLEN);
        modify->set_cornerangle(j.CORNERANGLE);
        modify->set_departlen(j.DEPARTLEN);
        modify->set_middlestart(j.MIDDLESTART);

        /********************龙骨数据********************/
        foreach (auto pos, j.framePos) {
            auto framepos = adedseam->add_framepos();
            framepos->set_weld(pos.weld);
            framepos->set_height(pos.height);
            framepos->set_index1(pos.index1);
            framepos->set_enabled(pos.enabled);

            framepos->set_angle1(pos.angle1);
            framepos->set_angle2(pos.angle2);
            framepos->set_angle3(pos.angle3);

            auto pp = framepos->mutable_pos();
            pp->set_x(pos.pos.x);
            pp->set_y(pos.pos.y);
            pp->set_z(pos.pos.z);
            pp->set_a(pos.pos.a);
            pp->set_b(pos.pos.b);
            pp->set_c(pos.pos.c);
        }

        /********************扫描数据********************/
        foreach (auto pos, j.scanePos) {
            auto scanepos = adedseam->add_scanepos();
            scanepos->set_camera(pos.camera);
            scanepos->set_index1(pos.index1);
            scanepos->set_enabled(pos.enabled);
            auto pp = scanepos->mutable_pos();
            pp->set_x(pos.pos.x);
            pp->set_y(pos.pos.y);
            pp->set_z(pos.pos.z);
            pp->set_a(pos.pos.a);
            pp->set_b(pos.pos.b);
            pp->set_c(pos.pos.c);
        }

         /****************焊缝关联************************/
         auto relate = adedseam->mutable_relate();
         relate->set_bind1(j.relate.bind1);
         relate->set_bind2(j.relate.bind2);
         relate->set_spacing1(j.relate.spacing1);
         relate->set_spacing2(j.relate.spacing2);
         relate->set_distance1(j.relate.distance1);
         relate->set_distance2(j.relate.distance2);
         relate->set_seflag(j.relate.seFlag);

         /*****************焊接信息***********************/
         auto weldinfo = adedseam->mutable_weldinfo();
         weldinfo->set_weldorder(j.weldinfo.weldorder);

         weldinfo->mutable_offset()->set_x(j.weldinfo.offset.x);
         weldinfo->mutable_offset()->set_y(j.weldinfo.offset.y);
         weldinfo->mutable_offset()->set_z(j.weldinfo.offset.z);
         weldinfo->mutable_offset()->set_a(j.weldinfo.offset.a);
         weldinfo->mutable_offset()->set_b(j.weldinfo.offset.b);
         weldinfo->mutable_offset()->set_c(j.weldinfo.offset.c);

         weldinfo->mutable_baseoffset()->set_x(j.weldinfo.baseoffset.x);
         weldinfo->mutable_baseoffset()->set_y(j.weldinfo.baseoffset.y);
         weldinfo->mutable_baseoffset()->set_z(j.weldinfo.baseoffset.z);
         weldinfo->mutable_baseoffset()->set_a(j.weldinfo.baseoffset.a);
         weldinfo->mutable_baseoffset()->set_b(j.weldinfo.baseoffset.b);
         weldinfo->mutable_baseoffset()->set_c(j.weldinfo.baseoffset.c);


         /*****************焊接参数***********************/
         auto weldpara = adedseam->mutable_weldpara();
         weldpara->set_i_w(j.weldPara.i_w);
         weldpara->set_s_w(j.weldPara.s_w);
         weldpara->set_v_w(j.weldPara.v_w);
         weldpara->set_t_end(j.weldPara.t_end);
         weldpara->set_i_end(j.weldPara.i_end);
         weldpara->set_v_end(j.weldPara.v_end);
         weldpara->set_range(j.weldPara.range);
         weldpara->set_axis_x(j.weldPara.axis_x);
         weldpara->set_axis_y(j.weldPara.axis_y);

    }

    string msg;

    if(root.SerializePartialToString(&msg)){
        qDebug()<<"msg序列化成功!";
    }


    std::fstream  out("Gen/Out.gen",std::fstream::out | std::ios::binary);
    if(root.SerializePartialToOstream(&out)){
        out.flush();
        out.close();
        out.clear();
        qDebug()<<"gen序列化成功!";
    }



    int fd = open("Gen/Out.fd",O_CREAT|O_TRUNC|O_RDWR,0644);
    if(fd <= 0){
        perror("Gen/Out.fd false!");
        exit(0);
    }
    if(root.SerializePartialToFileDescriptor(fd)){
        qDebug()<<"fd序列化成功!";
    }

    close(fd);

    DecodeControl control;
    control.root = root;
    std::thread ([&control](){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        qDebug()<< control.log;
    }).detach();
    qDebug()<<"准备数据库:"<<control.insert2SQL("clear/blade.db");
    qDebug()<<"数据库解码:"<<control.insertProtoData();
    control.closeDB();


    // google::protobuf::ShutdownProtobufLibrary();
    return msg;
}

RepeatData InputControl::TestCreat()
{
    RepeatData  dd;
    dd.date = "2020-04 20";
    ACad id;
    id.cadid = "dsdfsdk";
    id.cadMidPoint=1;
    id.cadBasePos=Pos(2,2,2,2,2,2);
    dd.cad.push_back(id);
    dd.cad.push_back(id);
    dd.cad.push_back(id);

    SeamInfo info;
    info.cadid = "dsdfsdk";
    info.seamID=64;
    info.orderIndex = 8;
    info.machine.motor1speed = 3;
    info.machine.angle1_motor_start = 3;


    AFramePos pp;
    AScanePos df;

    for(int i = 0;i<20;i++){
        info.framePos.push_back(pp);
        info.scanePos.push_back(df);
    }

    for(int i = 0;i<12;i++){
        dd.seams.push_back(info);
    }



    qDebug()<<"生成数据成功!";
    return dd;
}



//std::string InputControl::CreatSerial(RepeatData &data)
//{
//    /// 2.定义焊缝信息

//    GOOGLE_PROTOBUF_VERIFY_VERSION;
//    frame::Frame root;
//    std::string msg;
//    root.set_data("20200306");

//    qDebug()<<"姿态-->"<< data.size();
//    qDebug()<<"焊缝个数-->"<< data.at(0).posArray.size();
//    qDebug()<<"Scane Pos个数-->"<< data.at(0).posArray.at(0).scanePos.size();
//    qDebug()<<"Frame Pos个数-->"<< data.at(0).posArray.at(0).framePos.size();

//    // foreach()


//    foreach(Zitai_Info zitai_info, data){
//        ///

//        auto aZitai = root.add_zitai();
//        RepeatSeamArray seamList = zitai_info.posArray;


//        for(int i =0;i<seamList.size();i++){
//            auto aSeam = aZitai->add_seam_vector();
//            aSeam->set_bind1(seamList.at(i).bind1);
//            aSeam->set_bind2(seamList.at(i).bind2);
//            aSeam->set_seamname(seamList.at(i).seamName);
//            aSeam->set_seamindex(seamList.at(i).seamIndex);
//            aSeam->set_orderindex(seamList.at(i).orderIndex);
//            aSeam->set_isscan(seamList.at(i).isScan);

//            for(int j =0;j<seamList.at(i).scanePos.size();j++){
//                auto pos = aSeam->add_scanepos();
//                pos->set_x(seamList.at(i).scanePos.at(j).x);
//                pos->set_y(seamList.at(i).scanePos.at(j).y);
//                pos->set_z(seamList.at(i).scanePos.at(j).z);
//                pos->set_a(seamList.at(i).scanePos.at(j).a);
//                pos->set_b(seamList.at(i).scanePos.at(j).b);
//                pos->set_c(seamList.at(i).scanePos.at(j).c);
//            }
//            for(int j =0;j<seamList.at(i).framePos.size();j++){
//                auto pos = aSeam->add_framepos();
//                pos->set_x(seamList.at(i).framePos.at(j).x);
//                pos->set_y(seamList.at(i).framePos.at(j).y);
//                pos->set_z(seamList.at(i).framePos.at(j).z);
//                pos->set_a(seamList.at(i).framePos.at(j).a);
//                pos->set_b(seamList.at(i).framePos.at(j).b);
//                pos->set_c(seamList.at(i).framePos.at(j).c);
//            }

//        }

//        aZitai->set_spacing1(zitai_info.spacing1);
//        aZitai->set_spacing2(zitai_info.spacing2);
//        aZitai->set_distance1(zitai_info.distance1);
//        aZitai->set_distance2(zitai_info.distance2);
//        aZitai->set_pos_motor_start(zitai_info.pos_motor_start);
//        aZitai->set_pos_motor_stop(zitai_info.pos_motor_stop);

//        aZitai->set_angle1_motor_start(zitai_info.angle1_motor_start);
//        aZitai->set_angle1_motor_stop(zitai_info.angle1_motor_stop);
//        aZitai->set_angle2_motor_stop(zitai_info.angle2_motor_stop);
//        aZitai->set_angle2_motor_start(zitai_info.angle2_motor_start);
//        aZitai->set_cadid(zitai_info.cadid);


//        aZitai->set_motor1speed(zitai_info.motor1speed);
//        aZitai->set_motor2speed(zitai_info.motor2speed);
//        aZitai->set_motor3speed(zitai_info.motor3speed);

//        aZitai->mutable_cadmidpos()->set_x(zitai_info.cadMidPos.x);
//        aZitai->mutable_cadmidpos()->set_y(zitai_info.cadMidPos.y);
//        aZitai->mutable_cadmidpos()->set_z(zitai_info.cadMidPos.z);
//        aZitai->mutable_cadmidpos()->set_a(zitai_info.cadMidPos.a);
//        aZitai->mutable_cadmidpos()->set_b(zitai_info.cadMidPos.b);
//        aZitai->mutable_cadmidpos()->set_c(zitai_info.cadMidPos.c);

//        aZitai->mutable_cadbasepos()->set_x(zitai_info.cadBasePos.x);
//        aZitai->mutable_cadbasepos()->set_y(zitai_info.cadBasePos.y);
//        aZitai->mutable_cadbasepos()->set_z(zitai_info.cadBasePos.z);
//        aZitai->mutable_cadbasepos()->set_a(zitai_info.cadBasePos.a);
//        aZitai->mutable_cadbasepos()->set_b(zitai_info.cadBasePos.b);
//        aZitai->mutable_cadbasepos()->set_c(zitai_info.cadBasePos.c);
//    }



//    std::fstream  outStr("Gen/Out.txt",std::fstream::out | std::ios::binary);
//    if(root.SerializePartialToString(&msg)){
//        outStr<<msg;
//        outStr<<std::endl;
//        outStr.close();
//        qDebug()<<"序列化成功!";
//    }


//    std::fstream  out("Gen/Out.gen",std::fstream::out | std::ios::binary);
//    if(root.SerializePartialToOstream(&out)){
//        out.flush();
//        out.close();
//        out.clear();
//        qDebug()<<"序列化成功!";
//    }



//    int fd = open("Gen/Out.fd",O_CREAT|O_TRUNC|O_RDWR,0644);
//    if(fd <= 0){
//        perror("Gen/Out.fd false!");
//        exit(0);
//    }
//    if(root.SerializePartialToFileDescriptor(fd)){
//        qDebug()<<"序列化成功!";
//    }

//    close(fd);

////    DecodeControl control;
////    control.root = root;
////    qDebug()<<"准备数据库:"<<control.insert2SQL("clear/blade.db");
////    qDebug()<<"数据库解码:"<<control.insertProtoData();
////    control.closeDB();

//    google::protobuf::ShutdownProtobufLibrary();
//    return msg;
//}
