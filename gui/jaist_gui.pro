#-------------------------------------------------
#
# Project created by QtCreator 2015-10-06T17:25:28
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = jaist_gui
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    ros_launch.cpp \
    ros_thread.cpp

HEADERS  += mainwindow.h \
    header.h \
    ros_launch.h \
    ros_thread.h

FORMS    += mainwindow.ui
