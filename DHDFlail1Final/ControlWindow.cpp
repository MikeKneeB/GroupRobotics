//Written by Daniel Corless

#include <QTimer>

#include "PendulumSystem.h"

#include "ControlWindow.h"
#include "ui_ControlWindow.h"

#include "DisplayWindow.h"

//Constructor
ControlWindow::ControlWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ControlWindow)
{
    ui->setupUi(this);
    ui->runButton->setStyleSheet("background-color:red"); //Colour of "Run" button
    fRun = false; //Simulation initially is not running

    //Defining the system
    PendulumSystem * System = new PendulumSystem();
    fSystem = System;

    //Defining the pendulum with initial conditions
    System->AddPendulum(new Pendulum("Pendulum 1", 0.1, 1.51*52*52/(52.0)/(52.0), 0.0, 0.0, 0, 0.0, 0, 0.0, 0.1, 0.00443));

    //Initially displays the first pendulum in the system on the control window

    fCurrentPendulum = fSystem->GetPendulum(0);

    //Triggers an update every ms
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(Simulate()));
    timer->start(1);
}

ControlWindow::~ControlWindow() //Destructor
{
    delete ui;
}

//Runs the simulation with 1ms intervals
void ControlWindow::Simulate() {
    if(fRun == true) //If running then simulate the pendulums.
        fSystem->Simulate(0.001);
}

//Clicking the button causes the simulation to start/stop
void ControlWindow::on_runButton_clicked()
{
    if ((fRun == false)) {
        ui->runButton->setStyleSheet("background-color:green"); //Colour of button changes
        ui->runButton->setText("Stop"); //Changes the text to "Stop"
        //Creates the display window
        DisplayWindow* Window = new DisplayWindow(fSystem);
        Window->show();
        fRun = true; //Starts the simulation
    }
    else {
        ui->runButton->setStyleSheet("background-color:red"); //Indicates that simulation is not running
        ui->runButton->setText("Run"); //Changes the text to "Run"
        fRun = false; //Stop the simulation
    }
}
