#include <QItemSelectionModel>
#include <QListWidget>
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

    //Defining the pendulums with initial conditions and adding them to the system.
    //The initial conditions have been chosen to create the pendulum wave effect
    //Properties can be changed in the control window(e.g. changing charge and mass to cause interactions)
    //The class types below can be changed to have different pendulum types in different positions
    //and more pendulums could be added if wanted
    System->AddPendulum(new ChargedPendulum("Pendulum 1", 0.1, 1.51*52*52/(52.0)/(52.0), 0.0, 0.0, 0, 0.0, 0, 0.0, 0.1, 0.0));
    //System->AddPendulum(new ChargedPendulum("Pendulum 2", 0.1, 0.5*52*52/(53.0)/(53.0), 0.35, 0.0, 0.2, 0.0));
    //System->AddPendulum(new ChargedPendulum("Pendulum 3", 0.1, 0.5*52*52/(54.0)/(54.0), 0.35, 0.0, 0.3, 0.0));
    //System->AddPendulum(new ChargedPendulum("Pendulum 4", 0.1, 0.5*52*52/(55.0)/(55.0), 0.35, 0.0, 0.4, 0.0));
    //System->AddPendulum(new ChargedPendulum("Pendulum 5", 0.1, 0.5*52*52/(56.0)/(56.0), 0.35, 0.0, 0.5, 0.0));
    //System->AddPendulum(new ChargedPendulum("Pendulum 6", 0.1, 0.5*52*52/(57.0)/(57.0), 0.35, 0.0, 0.6, 0.0));
    //System->AddPendulum(new ChargedPendulum("Pendulum 7", 0.1, 0.5*52*52/(58.0)/(58.0), 0.35, 0.0, 0.7, 0.0));
    //System->AddPendulum(new ChargedPendulum("Pendulum 8", 0.1, 0.5*52*52/(59.0)/(59.0), 0.35, 0.0, 0.8, 0.0));
    //System->AddPendulum(new HeavyPendulum("Pendulum 9", 0.1, 0.5*52*52/(60.0)/(60.0), 0.35, 0.0, 0.9, 0.0));
    //System->AddPendulum(new HeavyPendulum("Pendulum 10", 0.1, 0.5*52*52/(61.0)/(61.0), 0.35, 0.0, 1.0, 0.0));
    //System->AddPendulum(new HeavyPendulum("Pendulum 11", 0.1, 0.5*52*52/(62.0)/(62.0), 0.35, 0.0, 1.1, 0.0));
    //System->AddPendulum(new HeavyPendulum("Pendulum 12", 0.1, 0.5*52*52/(63.0)/(63.0), 0.35, 0.0, 1.2, 0.0));
    //System->AddPendulum(new HeavyPendulum("Pendulum 13", 0.1, 0.5*52*52/(64.0)/(64.0), 0.35, 0.0, 1.3, 0.0));
    //System->AddPendulum(new HeavyPendulum("Pendulum 14", 0.1, 0.5*52*52/(65.0)/(65.0), 0.35, 0.0, 1.4, 0.0));
    //System->AddPendulum(new HeavyPendulum("Pendulum 15", 0.1, 0.5*52*52/(66.0)/(66.0), 0.35, 0.0, 1.5, 0.0));

    int NPendulums = System->GetNPendulums(); //Number of pendulums
    for (double i = 0; i < NPendulums; ++i) {

        //Adding pendulums to the system with initial conditions. The pendulum lengths have been chosen to create the pendulum wave effect
        std::string S = "Pendulum " + QString::number(i+1).toStdString();

        //Inserts pendulum names/numbers into list widget
        QListWidgetItem *PendulumItem = new QListWidgetItem;
        PendulumItem->setText(S.c_str());
        ui->pendulumList->insertItem(i+1, PendulumItem);
    }

    //Initially displays the first pendulum in the system on the control window
    ReadValues(fSystem->GetPendulum(0));
    fCurrentPendulum = fSystem->GetPendulum(0);

    //Used to select different pendulums on the list widget
    QItemSelectionModel *selectPendulum = ui->pendulumList->selectionModel();
    connect(selectPendulum, SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
            this, SLOT(selectionChangedSlot(const QItemSelection &, const QItemSelection &)));

    //Triggers an update every ms
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(Simulate()));
    timer->start(0.1);
}

ControlWindow::~ControlWindow() //Destructor
{
    delete ui;
}

//Reads values of pendulum properties and puts them in the corresponding input fields
void ControlWindow::ReadValues(Pendulum* Pendulum) {
    ui->lineEdit->setText(Pendulum->GetName().c_str());
    ui->lineEdit_2->setText(QString::number(Pendulum->GetMass()));
    ui->lineEdit_3->setText(QString::number(Pendulum->GetLength()));
    ui->lineEdit_4->setText(QString::number(Pendulum->GetTheta()));
    ui->lineEdit_5->setText(QString::number(Pendulum->GetThetaDot()));
    ui->lineEdit_6->setText(QString::number(Pendulum->GetPosition().Z()));
    ui->lineEdit_7->setText(QString::number(Pendulum->GetGamma()));
    //Only applies to charged pendulums
    if (Pendulum->GetId() == 2) {
        ui->lineEdit_8->setText(QString::number(Pendulum->GetCharge()));
    }else ui->lineEdit_8->setText("N/A"); //Charge is N/A for heavy pendulums
}

//Runs the simulation with 1ms intervals
void ControlWindow::Simulate() {
    if(fRun == true) //If running then simulate the pendulums.
        fSystem->Simulate(0.001);
}

//Clicking on a pendulum name in the list widget causes its properties to be displayed
void ControlWindow::selectionChangedSlot(const QItemSelection & /*newSelection*/, const QItemSelection & /*oldSelection*/)
{
    const QModelIndex index = ui->pendulumList->selectionModel()->currentIndex();
    QString selectedText = index.data(Qt::DisplayRole).toString();
    //Finds pendulum with name matching the selected text
    Pendulum * pendulum = fSystem->FindPendulum(selectedText.toStdString());
    fCurrentPendulum = pendulum;
    //Reads values of selected pendulum
    ReadValues(pendulum);
}

//Used to change the properties of the selected pendulum
//The method "SetValue()" is polymorphic, as the method is slightly different for the different pendulum classes
void ControlWindow::on_lineEdit_2_returnPressed()
{
    fCurrentPendulum->SetValue("Mass",ui->lineEdit_2->text().toStdString());
}

void ControlWindow::on_lineEdit_3_returnPressed()
{
    fCurrentPendulum->SetValue("Length",ui->lineEdit_3->text().toStdString());
}

void ControlWindow::on_lineEdit_4_returnPressed()
{
    fCurrentPendulum->SetValue("Theta",ui->lineEdit_4->text().toStdString());
}

void ControlWindow::on_lineEdit_5_returnPressed()
{
    fCurrentPendulum->SetValue("ThetaDot",ui->lineEdit_5->text().toStdString());
}

void ControlWindow::on_lineEdit_6_returnPressed()
{
    fCurrentPendulum->SetValue("Z",ui->lineEdit_6->text().toStdString());
}

void ControlWindow::on_lineEdit_7_returnPressed()
{
    fCurrentPendulum->SetValue("Gamma",ui->lineEdit_7->text().toStdString());
}

void ControlWindow::on_lineEdit_8_returnPressed()
{
    fCurrentPendulum->SetValue("Charge",ui->lineEdit_8->text().toStdString());
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

//Ticking the checkbox disables interactions between pendulums
void ControlWindow::on_disable_clicked()
{
    fSystem->ToggleDisable();
}
