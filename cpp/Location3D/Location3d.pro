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

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


#HALCONROOT="C:/Programs/Halcon"
#REALSENSEROOT="C:/Program Files (x86)/Intel RealSense SDK 2.0"
#defines
win32:DEFINES += WIN32

#*-msvc* {
#    QMAKE_CXXFLAGS += -O2
#    QMAKE_CXXFLAGS += -W3
#}


#includes
INCLUDEPATH   += "$$(HALCONROOT)/include"
INCLUDEPATH   += "$$(HALCONROOT)/include/halconcpp"
INCLUDEPATH   += "$$(REALSENSEROOT)/include"
INCLUDEPATH   += "$$(REALSENSEROOT)/samples"
#libs
QMAKE_LIBDIR  += "$$(HALCONROOT)/lib/$$(HALCONARCH)" \
                  "$$(REALSENSEROOT)/lib/x64"

LIBS    += "$$(HALCONROOT)/lib/x64-win64/halconcpp.lib" \
           "$$(HALCONROOT)/lib/x64-win64/halcon.lib" \
           "$$(REALSENSEROOT)/lib/x64/realsense2.lib"


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
