#include <QPainter>
#include <QTimer>
#include <QPointF>
#include <cmath>

#include "DisplayWindow.h"
#include "ui_DisplayWindow.h"

//Constructor
DisplayWindow::DisplayWindow(PendulumSystem * system, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DisplayWindow), fPendulumSystem(system), fScale(0)
{
    ui->setupUi(this);
    setStyleSheet("background-color:black"); //Background colour of display window
    setWindowTitle("Pendulum System"); //Name of display window

    //Triggers an update every ms
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(1);

}
//Destructor
DisplayWindow::~DisplayWindow()
{
    delete ui;
}

//Draws the pendulum
void DisplayWindow::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    int NPendulums = fPendulumSystem->GetNPendulums();
    for (int iComponent = 0; iComponent < NPendulums; ++iComponent) {
        ThreeVector position = fPendulumSystem->GetPendulum(iComponent)->GetPosition();
        //Scales the pendulum positions
        if (fScale == 0) fScale = fPendulumSystem->GetPendulum(0)->GetLength() / height()*2.5;

        //Defines pen used to draw pendulums
        QPen pen;
        //Makes pen cap round
        pen.setCapStyle(Qt::RoundCap);
        //Pendulum bob position
        QPointF p1(position.X()/fScale + width()/2.0, -position.Y()/fScale + height()/2.0);
        //Fixed point each pendulum is connected to
        QPointF p2(width()/2.0, height()/2.0);
        double A = 0.1;
        double theta = fPendulumSystem->GetPendulum(0)->GetTheta();
        double Omega = 3.132*2;
        double Phi = -3.142/2;
        double r0 = 1.0;

        //Up/Down Motion
        QPointF p3(r0*sin(theta)/fScale + A*sin(theta)*cos(Omega*ft+Phi)/fScale + width()/2.0, r0*cos(theta)/fScale + height()/2.0 + A*cos(theta)*cos(Omega*ft+Phi)/fScale);

        //Left/Right Motion
        //QPointF p3(position.X()/fScale + A*cos(theta)*cos(Omega*ft+Phi)/fScale + width()/2.0, -position.Y()/fScale + height()/2.0 + -A*sin(theta)*cos(Omega*ft+Phi)/fScale);

        //Choosing pen properties
        pen.setWidth(1);
        pen.setColor(Qt::gray);
        painter.setPen(pen);
        //Draws pendulum "rod"
        painter.drawLine(p1, p2);

        pen.setWidth(8);
        if (fPendulumSystem->GetPendulum(iComponent)->GetId() == 1)
            pen.setColor(Qt::yellow); //Colour for heavy pendulums
        else if (fPendulumSystem->GetPendulum(iComponent)->GetId() == 2 && fPendulumSystem->GetPendulum(iComponent)->GetCharge() == 0)
            pen.setColor(Qt::white); //Colour for neutral-charge pendulums
        else if (fPendulumSystem->GetPendulum(iComponent)->GetCharge() < 0)
            pen.setColor(Qt::cyan); //Colour for negatively charged pendulums
        else if (fPendulumSystem->GetPendulum(iComponent)->GetCharge() > 0)
            pen.setColor(Qt::red); //Colour for positively charged pendulums

        painter.setPen(pen);
        //Draws pendulum bob
        painter.drawPoint(p1);
        painter.drawPoint(p3);

        ft = ft + 0.001;
    }
}
