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



##########defines#############
DEFINES += QT_DEPRECATED_WARNINGS
win32:DEFINES += WIN32


*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
############Boost#############
BOOST_INCLUDE_DIRS="D:/RawDownLoads/boost_1_62"
BOOST_LIB_DIRS = "$$BOOST_INCLUDE_DIRS/lib64-msvc-14.0"
B0_INCLUDE_DIRS="$$PWD/include/bluezero"
B0_LIBS = "$$PWD/lib/b0.lib"
INCLUDEPATH += $$BOOST_INCLUDE_DIRS
INCLUDEPATH += $$B0_INCLUDE_DIRS
LIBS +=  $$B0_LIBS
LIBS += -L$$BOOST_LIB_DIRS


############hanclon#############
HALCONROOT="C:/Programs/Halcon"
INCLUDEPATH   += "$$(HALCONROOT)/include"
INCLUDEPATH   += "$$(HALCONROOT)/include/halconcpp"
QMAKE_LIBDIR  += "$$(HALCONROOT)/lib/$$(HALCONARCH)" \
                  "$$(REALSENSEROOT)/lib/x64"
LIBS    += "$$(HALCONROOT)/lib/x64-win64/halconcpp.lib" \
           "$$(HALCONROOT)/lib/x64-win64/halcon.lib"

############realsense#############
#REALSENSEROOT="C:/Program Files (x86)/Intel RealSense SDK 2.0"
#INCLUDEPATH   += "$$quote(C:/Program Files (x86)/Intel RealSense SDK 2.0/include)"
#INCLUDEPATH   += "$$quote(C:/Program Files (x86)/Intel RealSense SDK 2.0/samples)"
#QMAKE_LIBDIR  += "$$quote(C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64)"
#LIBS+="$$quote(C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64/realsense2.lib)"




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


