#-------------------------------------------------
#
# Project created by QtCreator 2017-01-24T14:38:24
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PendulumSim
TEMPLATE = app


SOURCES += main.cpp\
        MainWindow.cpp \
    Mass.cpp \
    TwoVector.cpp \
    PendulumWidget.cpp

HEADERS  += MainWindow.h \
    Mass.h \
    TwoVector.h \
    PendulumWidget.h

FORMS    += MainWindow.ui

DISTFILES +=
