QT       += core gui sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

DEFINES += QT_DEPRECATED_WARNINGS
include($$PWD/pri/developer.pri)


TARGET = PathManagement
#TEMPLATE = app


SOURCES += \
    mainwindow.cpp \
    main.cpp\
    mainwindow2.cpp \
    dialog.cpp \
    Proto/DataControl/DecodeControl.cpp \
    Proto/DataControl/Frame.cpp \
    Proto/DataControl/InputControl.cpp \
    src/curveFit.cpp \
    src/circleFit.cpp \
    src/matrix.cpp \
    fitSrc/cubicspline.cc \
    fitSrc/smoothspline.cc \
    fitSrc/smoothspline3d.cc

HEADERS += \
    mainwindow.h \
    Proto/DataControl/DecodeControl.h \
    Proto/DataControl/Frame.h \
    Proto/DataControl/InputControl.h \
    mainwindow2.h \
    dialog.h \
    mainwindow.h \
    Proto/DataControl/baseTypes.h \
    include/curveFit.h \
    include/circleFit.h \
    include/defines.h \
    include/matrix.h \
    fitHead/bench.h \
    fitHead/cubicspline.h \
    fitHead/point.h \
    fitHead/smoothspline.h \
    fitHead/smoothspline3d.h

FORMS += \
    mainwindow.ui \
    mainwindow2.ui \
    dialog.ui



SOURCES +=Proto/Google/Frame.pb.cc
HEADERS += Proto/Google/Frame.pb.h


# LIBS += D:\OpenCV\opencv\build\x64\vc14\lib
#LIBS += D:\boost_1_72_0\libs \
#        -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_tracking \
#        -latomic \
#        -lboost_filesystem -lboost_system -lboost_date_time \
#LIBS += -LD:/Boost_msvc_static/lib


INCLUDEPATH += D:\Eigen\eigen-eigen
INCLUDEPATH += D:\boost_1_72_0
INCLUDEPATH += D:\OpenCV\opencv\build\include
#INCLUDEPATH += D:\Qt\project\PathManagement\external\include
#INCLUDEPATH += D:\Qt\project\PathManagement\external\src
