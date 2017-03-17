#ifndef DISPLAYWINDOW_H
#define DISPLAYWINDOW_H

#include <QWidget>

#include "PendulumSystem.h"

//This class is used to display the simulation

namespace Ui {
class DisplayWindow;
}

class DisplayWindow : public QWidget
{
    Q_OBJECT

public:
    //Constructor - includes the system as a variable
    explicit DisplayWindow(PendulumSystem* system, QWidget *parent = 0);
    ~DisplayWindow(); //Destructor

private slots:
    //Draws the pendulum
    void paintEvent(QPaintEvent *);

private:
    Ui::DisplayWindow *ui;
    PendulumSystem * fPendulumSystem; //The system being simulated
    double fScale; //Scales the pendulum positions
    double ft = 0;
    double theta2=1.9;
    double t2Dot=0;
};

#endif // DISPLAYWINDOW_H
