#-------------------------------------------------
#
# Project created by QtCreator 2017-03-16T12:21:59
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = DHingeDFlail10
TEMPLATE = app


SOURCES += main.cpp\
        ControlWindow.cpp \
    chargedpendulum.cpp \
    DisplayWindow.cpp \
    heavypendulum.cpp \
    pendulum.cpp \
    PendulumSystem.cpp \
    ThreeVector.cpp

HEADERS  += ControlWindow.h \
    chargedpendulum.h \
    DisplayWindow.h \
    heavypendulum.h \
    pendulum.h \
    PendulumSystem.h \
    ThreeVector.h

FORMS    += ControlWindow.ui \
    DisplayWindow.ui
