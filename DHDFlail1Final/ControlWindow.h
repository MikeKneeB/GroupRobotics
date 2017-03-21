//Written by Daniel Corless

#ifndef CONTROLWINDOW_H
#define CONTROLWINDOW_H

#include <QMainWindow>
#include <QItemSelection>

#include "PendulumSystem.h"

//This class is used to create the control window, from where the simulation can be run

namespace Ui {
class ControlWindow;
}

class ControlWindow : public QMainWindow
{
    Q_OBJECT

public:
    //Constructor
    explicit ControlWindow(QWidget *parent = 0);
    ~ControlWindow(); //Destructor

private slots:

    void Simulate(); //Used to run the simulation

    void on_runButton_clicked(); //Clicking the button causes the simulation to start/stop

private:
    Ui::ControlWindow *ui;
    bool fRun; //Variable used to start/stop the simulation
    PendulumSystem * fSystem; //System being simulated
    Pendulum * fCurrentPendulum; //Pendulum selected on the list widget
};

#endif // CONTROLWINDOW_H
