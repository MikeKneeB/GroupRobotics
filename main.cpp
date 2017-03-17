#include "ControlWindow.h"
#include "DisplayWindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ControlWindow window;
    //Displays the control window
    window.show();

    return a.exec();
}
