TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    getsbuva.cc \
    #getxuvv.cc \
    #getxuve.cc \
    histimage.cc \
    #getxuvb.cc
    utils.cc \
    getsbuvj.cc \
    getsbuvz.cc \
    getsbuvs.cc


OPENCV_HOME = /home/developer/opencv320
walgo_home = /home/yongxiang/robotics/release

INCLUDEPATH += $${OPENCV_HOME}/include \
                $${walgo_home}/include \
                /home/developer/boost1640/include

LIBS += -L$${OPENCV_HOME}/lib \
        -L$${walgo_home}/lib  \
        -L/home/developer/boost1640/lib/debug \
        -limage -l3dvision -lbasic -lgeometry\
        #-lpubfunc -ldevice -lcamera_image -limage -l3dvision -lbasic -lgeometry  -lalgorithm -ltools -lbase \
        -lboost_filesystem -lboost_system -lboost_date_time \
        -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_tracking -lopencv_video -pthread -lnlopt


HEADERS += \
    getsbuva.h \
    histimage.h \
    utils.h
