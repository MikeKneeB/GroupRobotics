#include "MainWindow.h"
#include "ui_MainWindow.h"

#include <QTimer>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    fFramerate(50000)
{
    ui->setupUi(this);

    //Start a new timer and attach it to this window
    QTimer *timer = new QTimer(this);
    //Connect the timeout signal from the timer to the BounceWidget update slot
    connect(timer, SIGNAL(timeout()), ui->pendulumWidget, SLOT(update()));
    //Set the timeout time based on the frame rate
    timer->start(1000/fFramerate);
}

MainWindow::~MainWindow()
{
    delete ui;
}

