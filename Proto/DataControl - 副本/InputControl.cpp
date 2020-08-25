#include "InputControl.h"
#include <QProcess>

#ifdef Q_OS_LINUX
    #include <sys/stat.h>
#endif
#include <fcntl.h>

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

RepeatData& InputControl::TestCreat()
{
    CreatDir();
    RepeatData *test = new RepeatData;
    Zitai_Info child;

    child.spacing1 = 10.0;
    child.spacing2 = 10.0;
    child.distance1 = 10.0;
    child.distance2 = 10.0;
    child.motor1speed = 10.0;
    child.motor2speed = 10.0;
    child.motor3speed = 10.0;


    Seaminfo abo;
    abo.bind1 = 1;
    abo.seamIndex =2;
    abo.bind2 =3;
    abo.seamName= "seam_0_1";
    abo.scanePos <<Pos(6.0,6.0,6.0,6.0,6.0,6.0)<<Pos(5.5,5.5,5.5,5.5,5.5,5.5)<<Pos(4.4,4.4,4.4,4.4,4.4,4.4);
    abo.framePos <<Pos(6.0,6.0,6.0,6.0,6.0,6.0)<<Pos(5.5,5.5,5.5,5.5,5.5,5.5)<<Pos(4.4,4.4,4.4,4.4,4.4,4.4);
    child.posArray<<abo<<abo<<abo<<abo<<abo;

    child.pos_motor_start = 10;		//地轨开始位置
    child.pos_motor_stop = 11;		//地轨开始位置

    child.angle1_motor_start = 12;	//变位机开始角度
    child.angle1_motor_stop = 13;	//变位机停止角度

    child.angle2_motor_start = 14;	//变位机开始角度
    child.angle2_motor_stop = 15;	//变位机停止角度

    *test<<child<<child<<child<<child;

    return  *test;
}




std::string InputControl::CreatSerial(RepeatData &data)
{
    /// 2.定义焊缝信息

    GOOGLE_PROTOBUF_VERIFY_VERSION;
    frame::Frame root;
    std::string msg;
    root.set_data("20200306");

    qDebug()<<"姿态-->"<< data.size();
    qDebug()<<"焊缝个数-->"<< data.at(0).posArray.size();
    qDebug()<<"Scane Pos个数-->"<< data.at(0).posArray.at(0).scanePos.size();
    qDebug()<<"Frame Pos个数-->"<< data.at(0).posArray.at(0).framePos.size();

    foreach(Zitai_Info zitai_info, data){
        ///

        auto aZitai = root.add_zitai();
        RepeatSeamArray seamList = zitai_info.posArray;


        for(int i =0;i<seamList.size();i++){
            auto aSeam = aZitai->add_seam_vector();
            aSeam->set_bind1(seamList.at(i).bind1);
            aSeam->set_bind2(seamList.at(i).bind2);
            aSeam->set_seamname(seamList.at(i).seamName);
            aSeam->set_seamindex(seamList.at(i).seamIndex);
            aSeam->set_orderindex(seamList.at(i).orderIndex);
            aSeam->set_isscan(seamList.at(i).isScan);

            for(int j =0;j<seamList.at(i).scanePos.size();j++){
                auto pos = aSeam->add_scanepos();
                pos->set_x(seamList.at(i).scanePos.at(j).x);
                pos->set_y(seamList.at(i).scanePos.at(j).y);
                pos->set_z(seamList.at(i).scanePos.at(j).z);
                pos->set_a(seamList.at(i).scanePos.at(j).a);
                pos->set_b(seamList.at(i).scanePos.at(j).b);
                pos->set_c(seamList.at(i).scanePos.at(j).c);
            }
            for(int j =0;j<seamList.at(i).framePos.size();j++){
                auto pos = aSeam->add_framepos();
                pos->set_x(seamList.at(i).framePos.at(j).x);
                pos->set_y(seamList.at(i).framePos.at(j).y);
                pos->set_z(seamList.at(i).framePos.at(j).z);
                pos->set_a(seamList.at(i).framePos.at(j).a);
                pos->set_b(seamList.at(i).framePos.at(j).b);
                pos->set_c(seamList.at(i).framePos.at(j).c);
            }

        }

        aZitai->set_spacing1(zitai_info.spacing1);
        aZitai->set_spacing2(zitai_info.spacing2);
        aZitai->set_distance1(zitai_info.distance1);
        aZitai->set_distance2(zitai_info.distance2);
        aZitai->set_pos_motor_start(zitai_info.pos_motor_start);
        aZitai->set_pos_motor_stop(zitai_info.pos_motor_stop);

        aZitai->set_angle1_motor_start(zitai_info.angle1_motor_start);
        aZitai->set_angle1_motor_stop(zitai_info.angle1_motor_stop);
        aZitai->set_angle2_motor_stop(zitai_info.angle2_motor_stop);
        aZitai->set_angle2_motor_start(zitai_info.angle2_motor_start);
        aZitai->set_cadid(zitai_info.cadid);


        aZitai->set_motor1speed(zitai_info.motor1speed);
        aZitai->set_motor2speed(zitai_info.motor2speed);
        aZitai->set_motor3speed(zitai_info.motor3speed);

        aZitai->mutable_cadmidpos()->set_x(zitai_info.cadMidPos.x);
        aZitai->mutable_cadmidpos()->set_y(zitai_info.cadMidPos.y);
        aZitai->mutable_cadmidpos()->set_z(zitai_info.cadMidPos.z);
        aZitai->mutable_cadmidpos()->set_a(zitai_info.cadMidPos.a);
        aZitai->mutable_cadmidpos()->set_b(zitai_info.cadMidPos.b);
        aZitai->mutable_cadmidpos()->set_c(zitai_info.cadMidPos.c);

        aZitai->mutable_cadbasepos()->set_x(zitai_info.cadBasePos.x);
        aZitai->mutable_cadbasepos()->set_y(zitai_info.cadBasePos.y);
        aZitai->mutable_cadbasepos()->set_z(zitai_info.cadBasePos.z);
        aZitai->mutable_cadbasepos()->set_a(zitai_info.cadBasePos.a);
        aZitai->mutable_cadbasepos()->set_b(zitai_info.cadBasePos.b);
        aZitai->mutable_cadbasepos()->set_c(zitai_info.cadBasePos.c);
    }



//    qDebug()<<"C姿态-->"<< root.zitai_size();
//    qDebug()<<"C焊缝个数-->"<< root.zitai(0).seam_vector_size();
//    qDebug()<<"CPos个数-->"<< root.zitai(0).seam_vector(0).scanepos_size();

    std::fstream  outStr("Gen/Out.txt",std::fstream::out | std::ios::binary);
    if(root.SerializePartialToString(&msg)){
        outStr<<msg;
        outStr<<std::endl;
        outStr.close();
        qDebug()<<"序列化成功!";
    }


    std::fstream  out("Gen/Out.gen",std::fstream::out | std::ios::binary);
    if(root.SerializePartialToOstream(&out)){
        out.flush();
        out.close();
        out.clear();
        qDebug()<<"序列化成功!";
    }



    int fd = open("Gen/Out.fd",O_CREAT|O_TRUNC|O_RDWR,0644);
    if(fd <= 0){
        perror("Gen/Out.fd false!");
        exit(0);
    }
    if(root.SerializePartialToFileDescriptor(fd)){
        qDebug()<<"序列化成功!";
    }

    close(fd);

    DecodeControl control;
    control.root = root;
    qDebug()<<"准备数据库:"<<control.insert2SQL("clear/blade.db");
    qDebug()<<"数据库解码:"<<control.insertProtoData();
    control.closeDB();

    google::protobuf::ShutdownProtobufLibrary();
    return msg;
}
