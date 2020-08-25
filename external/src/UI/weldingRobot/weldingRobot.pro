
#-------------------------------------------------
#
# Project created by QtCreator 2017-05-02T10:58:54
#
#-------------------------------------------------

QT       += core gui network serialport serialbus

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = weldingRobot
TEMPLATE = app
CONFIG += c++1y


# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
#DEFINES += QT_DEPRECATED_WARNINGS WELDDEBUG
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

PROROOT=/home/weldroid/robotics/weldroidProject
PROJ_HOME=/home/weldroid/robotics
OPENCV_HOME=/home/developer/opencv320
BOOST_HOME=/home/developer/boost1640
MVCAM_COMMON_HOME=/opt/MVS
MODBUS_HOME=/home/developer/modbus


MOC_DIR = $$PWD/moc
UI_DIR = $$PWD/ui
OBJECTS_DIR = $$PWD/obj

src_dir = $${PROROOT}/src/UI/weldingRobot/etc
dst_dir = $$OUT_PWD/etc

!exists($$dst_dir) {
  system(cp -R $$src_dir $$dst_dir)
}

QMAKE_CXXFLAGS = -std=c++1z

INCLUDEPATH += $${OPENCV_HOME}/include \
               $${PROROOT}/include \
               $${PROROOT}/include/walgo \
               $${MODBUS_HOME}/include \
               /home/developer/boost1640/include \
                $${MVCAM_COMMON_HOME}/include \


LIBS += -L$${OPENCV_HOME}/lib \
        -L$${PROROOT}/lib \
        -L$${BOOST_HOME}/lib/debug \
        -L$${MODBUS_HOME}/lib \
        -ltools -lbasic -lgeometry -l3dvision -limage \
        -lbase -ldevice \
        -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_tracking \
        -latomic \
        -lm3api \
        -lboost_filesystem -lboost_system -lboost_date_time \
        -lpcan \
        -lnlopt \
        -lmodbus -lsqlite3 \
        -L$${MVCAM_COMMON_HOME}/lib/64 -lGCBase_gcc421_v3_0 -lGenApi_gcc421_v3_0 -lXmlParser_gcc421_v3_0 -lMvCameraControl

SOURCES += main.cpp \
        mainwindow.cpp \
    common/tmessagebox.cpp \
    base/adjustui.cpp \
    base/robotui.cpp \
    base/settingui.cpp \
    base/sensorui.cpp \
    base/imagelabel.cpp \
#    common/expCheck.cpp \
    modbusControl/mbControl.cpp \
    modbusControl/mbCommon.cpp \
    base/bladeui.cpp \
    base/tracing.cpp \
    base/cworkstation.cpp \
    base/cworkpiecedata.cpp \
    base/griddev.cpp \
    getuvonshell/getsbuva.cc \
    getuvonshell/getsbuvj.cc \
    getuvonshell/getsbuvz.cc \
    getuvonshell/getsbuvi.cc \
    getuvonshell/histimage.cc \
    getuvonshell/utils.cc \
    getuvonshell/getsbuvs.cc \
    getuvonshell/getsbuvvw.cc \
    base/fitCorrelation.cpp \
    getuvonshell/getsbuve.cc \
    getuvonshell/getsbuvm.cc \
    common/dbinterface.cpp \
    getuvonshell/self_adaption.cpp \
    getuvonshell/getsbuvw.cc \
    fit/sepmodel3d.cc \
    getuvonshell/getsbuvjs.cc \
    base/tracing_hg.cpp \
    getuvonshell/getsbuvse.cc \
    base/seamhandler.cpp \
    getuvonshell/getsbuvwc.cc

HEADERS  += mainwindow.h \
    common/tmessagebox.h \
    base/adjustui.h \
    base/robotui.h \
    base/sensorui.h \
    base/imagelabel.h \
    base/settingui.h \
#    common/expCheck.h \
    modbusControl/mbControl.h \
    modbusControl/mbCommon.h \
    base/bladeui.h \
    base/cworkstation.h \
    base/cworkpiecedata.h \
    base/griddev.h \
    getuvonshell/getsbuva.h \
    getuvonshell/histimage.h \
    getuvonshell/utils.h \
    base/fitCorrelation.h \
    common/dbinterface.h \
    getuvonshell/self_adaption.h \
    base/tracing.h \
    fit/sepmodel3d.h \
    base/seamhandler.h \
    robot/as_udp.h

FORMS    += mainwindow.ui \
    base/adjust.ui \
    base/adjustui.ui \
    base/robotui.ui \
    base/sensorui.ui \
    base/settingui.ui \
    base/bladeui.ui \
    base/tracing.ui \
    base/griddev.ui

