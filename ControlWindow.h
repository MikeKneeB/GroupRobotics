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
    //Reads values of pendulum properties and puts them in the corresponding input fields
    void ReadValues(Pendulum* Pendulum);
    void Simulate(); //Used to run the simulation
    //Displays properties of the pendulum selected on the list widget
    void selectionChangedSlot(const QItemSelection & /*newSelection*/, const QItemSelection & /*oldSelection*/);

    //Used to change the properties of a selected pendulum
    void on_lineEdit_2_returnPressed();
    void on_lineEdit_3_returnPressed();
    void on_lineEdit_4_returnPressed();
    void on_lineEdit_5_returnPressed();
    void on_lineEdit_6_returnPressed();
    void on_lineEdit_7_returnPressed();
    void on_lineEdit_8_returnPressed();

    void on_runButton_clicked(); //Clicking the button causes the simulation to start/stop
    void on_disable_clicked(); //Ticking the checkbox disables interactions between pendulums

private:
    Ui::ControlWindow *ui;
    bool fRun; //Variable used to start/stop the simulation
    PendulumSystem * fSystem; //System being simulated
    Pendulum * fCurrentPendulum; //Pendulum selected on the list widget
};

#endif // CONTROLWINDOW_H
