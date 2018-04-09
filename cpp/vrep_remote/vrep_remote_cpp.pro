QT -= core
QT -= gui

TEMPLATE = app
DEFINES -= UNICODE
#CONFIG   += console
CONFIG   -= app_bundle

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}

win32 {
    LIBS += -lwinmm
    LIBS += -lWs2_32
}


win32:DEFINES += WIN32
DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255

#Remote API
INCLUDEPATH+="./include"
INCLUDEPATH+="./remoteApi"

#OpenCV
INCLUDEPATH+=$(OPENCV)/include \
            $(OPENCV)/include/opencv\
            $(OPENCV)/include/opencv2

#Halcon
INCLUDEPATH   += $(HALCONROOT)/include
INCLUDEPATH   += $(HALCONROOT)/include/halconcpp
#libs
QMAKE_LIBDIR  += $(HALCONROOT)/lib/$(HALCONARCH)
win32:LIBS    += $(HALCONROOT)/lib/$(HALCONARCH)/halconcpp.lib \
               $(HALCONROOT)/lib/$(HALCONARCH)/halcon.lib




SOURCES += ./remoteApi/extApi.c \
           ./remoteApi/extApiPlatform.c\
           main.cpp

HEADERS +=./remoteApi/extApi.h \
    ./remoteApi/extApiPlatform.h

win32:CONFIG(release, debug|release): LIBS += -L$(OPENCV)/x64/vc12/lib/ -lopencv_world310
else:win32:CONFIG(debug, debug|release): LIBS += -L$(OPENCV)/x64/vc12/lib/ -lopencv_world310d


win32:CONFIG(release, debug|release): LIBS += -L$(OPENCV)/x64/vc12/bin/ -lopencv_world310
else:win32:CONFIG(debug, debug|release): LIBS += -L$(OPENCV)/x64/vc12/bin/ -lopencv_world310d
