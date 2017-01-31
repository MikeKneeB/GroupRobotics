#-------------------------------------------------
#
# Project created by QtCreator 2017-01-31T14:44:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Robot
TEMPLATE = app


SOURCES += main.cpp\
        MainWindow.cpp \
    Robot.cpp \
    BodyPart.cpp \
    TwoVector.cpp

HEADERS  += MainWindow.h \
    Robot.h \
    BodyPart.h \
    TwoVector.h

FORMS    += \
    MainWindow.ui
