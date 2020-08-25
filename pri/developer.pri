# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


win32{
    INCLUDEPATH += $$PWD/../third/Google/libs/win/mingw
    DEPENDPATH += $$PWD/../third/Google/libs/win/mingw
    INCLUDEPATH += "C:\SDK\protobuf-3.11.4\src"
    PRE_TARGETDEPS += $$PWD/../third/Google/libs/win/mingw/libprotobuf.a
    LIBS += -L$$PWD/../third/Google/libs/win/mingw/ -lprotobuf
    LIBS += -LC:\SDK\build -lprotobuf
}




linux-g++*{
    PROTO_HOME = ~/zxx/work/protobuf
    INCLUDEPATH += $${PROTO_HOME}/src
    LIBS += -lprotobuf
}
