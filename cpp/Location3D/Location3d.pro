#-------------------------------------------------
#
# Project created by QtCreator 2017-10-10T15:34:06
#
#-------------------------------------------------

QT       += core gui
CONFIG   +=c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Location3D
TEMPLATE = app


DEFINES += QT_DEPRECATED_WARNINGS



##########defines#############
HALCONROOT="C:/Programs/Halcon"
#REALSENSEROOT="C:/Program Files (x86)/Intel RealSense SDK 2.0"

win32:DEFINES += WIN32

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}


############hanclon#############
INCLUDEPATH   += "$$(HALCONROOT)/include"
INCLUDEPATH   += "$$(HALCONROOT)/include/halconcpp"
QMAKE_LIBDIR  += "$$(HALCONROOT)/lib/$$(HALCONARCH)" \
                  "$$(REALSENSEROOT)/lib/x64"
LIBS    += "$$(HALCONROOT)/lib/x64-win64/halconcpp.lib" \
           "$$(HALCONROOT)/lib/x64-win64/halcon.lib"

############realsense#############
#INCLUDEPATH   += "$$(REALSENSEROOT)/include"
#INCLUDEPATH   += "$$(REALSENSEROOT)/samples"
#LIBS+="$$(REALSENSEROOT)/lib/x64/realsense2.lib"
#QMAKE_LIBDIR  += "$$(REALSENSEROOT)/lib/x64"




SOURCES += \
        main.cpp \
        mainwindow.cpp \
    camsetting.cpp \
    configloader.cpp \
    camcalibration.cpp

HEADERS += \
        mainwindow.h \
        imgprocess.hpp \
    camsetting.h \
    configloader.h \
    camcalibration.h

FORMS += \
        mainwindow.ui \
    camsetting.ui \
    camcalibration.ui
