QT       += core gui widgets
CONFIG   +=c++11


TARGET = Location3D
TEMPLATE = app


##########defines#############
DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += WIN32

#CONFIG += WITH_BZERO

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
    QMAKE_CXXFLAGS += -EHsc
    QMAKE_CXXFLAGS += -GR

}
############Boost#############
WITH_BZERO {
    DEFINES += WITH_BZERO
    BOOST_INCLUDE_DIRS="D:/library/boost"
    BOOST_LIB_DIRS = "$$BOOST_INCLUDE_DIRS/lib64-msvc-14.0"
    B0_INCLUDE_DIRS="$$PWD/include/bluezero"
    B0_LIBS = "$$PWD/lib/b0.lib"
    INCLUDEPATH += $$BOOST_INCLUDE_DIRS
    INCLUDEPATH += $$B0_INCLUDE_DIRS
    LIBS +=  $$B0_LIBS
    LIBS += -L$$BOOST_LIB_DIRS
}

############hanclon#############
HALCONROOT="C:/Programs/Halcon"
INCLUDEPATH   += "$$(HALCONROOT)/include"
INCLUDEPATH   += "$$(HALCONROOT)/include/halconcpp"
QMAKE_LIBDIR  += "$$(HALCONROOT)/lib/$$(HALCONARCH)"
LIBS    += "$$(HALCONROOT)/lib/x64-win64/halconcpp.lib" \
           "$$(HALCONROOT)/lib/x64-win64/halcon.lib"

############realsense#############
#REALSENSEROOT="C:/Program Files (x86)/Intel RealSense SDK 2.0"
#INCLUDEPATH   += "$$quote(C:/Program Files (x86)/Intel RealSense SDK 2.0/include)"
#INCLUDEPATH   += "$$quote(C:/Program Files (x86)/Intel RealSense SDK 2.0/samples)"
#QMAKE_LIBDIR  += "$$quote(C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64)"
#LIBS+="$$quote(C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64/realsense2.lib)"

##########msgpack-rpc##############
INCLUDEPATH +="$$PWD/include/rpclib"
LIBS+= "$$PWD/lib/rpc.lib"



SOURCES += \
        main.cpp \
        mainwindow.cpp \
    camsetting.cpp \
    configloader.cpp \
    camcalibration.cpp

HEADERS += \
        mainwindow.h \
    camsetting.h \
    configloader.h \
    camcalibration.h \
    image.h

FORMS += \
        mainwindow.ui \
    camsetting.ui \
    camcalibration.ui





