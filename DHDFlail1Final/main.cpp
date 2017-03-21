//Written by Daniel Corless

#include "ControlWindow.h"
#include "DisplayWindow.h"
#include <QApplication>

//The simulation code is contained in the "PendulumSystem.cpp" file

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ControlWindow window;
    //Displays the control window
    window.show();

    return a.exec();
}
