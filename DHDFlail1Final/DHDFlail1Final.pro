#-------------------------------------------------
#
# Project created by QtCreator 2017-03-21T17:25:47
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DHDFlail1Final
TEMPLATE = app


SOURCES += main.cpp\
        ControlWindow.cpp \
    DisplayWindow.cpp \
    pendulum.cpp \
    PendulumSystem.cpp \
    ThreeVector.cpp

HEADERS  += ControlWindow.h \
    DisplayWindow.h \
    pendulum.h \
    PendulumSystem.h \
    ThreeVector.h

FORMS    += ControlWindow.ui \
    DisplayWindow.ui
