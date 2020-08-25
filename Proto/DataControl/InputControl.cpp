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
            scanepos->set_angle1(pos.angle1);
            scanepos->set_angle2(pos.angle2);
            scanepos->set_angle3(pos.angle3);
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

//    DecodeControl control;
//    control.root = root;
//    std::thread ([&control](){
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//        qDebug()<< control.log;
//    }).detach();
//    qDebug()<<"准备数据库:"<<control.insert2SQL("clear/blade.db");
//    qDebug()<<"数据库解码:"<<control.insertProtoData();
//    control.closeDB();


    google::protobuf::ShutdownProtobufLibrary();
    return msg;
}
