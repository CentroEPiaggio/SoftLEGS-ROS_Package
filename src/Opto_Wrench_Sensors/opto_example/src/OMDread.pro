#-------------------------------------------------
#
# Project created by QtCreator 2014-06-06T11:01:14
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = OMDread
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

HEADERS += \
    omd/sensorconfig.h \
    omd/optoports.h \
    omd/optopackage.h \
    omd/optodaq.h \
    omd/opto.h \
    omd/optopackage6d.h



#----------
#unix:!macx: LIBS += -L"/home/optoforce/API_Qt_DYN/DYNAMIC" -lOMD

#unix:!macx: LIBS += -L$$PWD/../../API/64bit/lib/ -lOMD

#INCLUDEPATH += $$PWD/../../API/64bit/include
#DEPENDPATH += $$PWD/../../API/64bit/include
#----------

unix:!macx: LIBS += -L$$PWD/../lib/ -lOMD

INCLUDEPATH += $$PWD/omd
DEPENDPATH += $$PWD/omd
