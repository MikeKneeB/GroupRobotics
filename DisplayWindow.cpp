#include <QPainter>
#include <QTimer>
#include <QPointF>
#include <cmath>
#include <iostream>

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

double time1=0;
double t21=0;
double t31=0;
double t41=0;
double t51=0;
double theta5 = 0;

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

        double a1 = fPendulumSystem->GetPendulum(0)->GetTheta();
        double a2 = fPendulumSystem->GetPendulum(0)->GetTheta2();
        double t1 = fPendulumSystem->GetPendulum(0)->GetTheta3();




        double d1 = 1.51;
        double d2 = 0.12;
        double l1 = 0.185;
        double l2 = 0.2115*2/3.0;
        double l3 = 0.1*0.5;
        double l4 = 0.11441*0.5;
        double l5 = 0.14809*2/3.0;
        double r2 = 0.2115;
        double r3 = 0.1;


        double A = 0.2394;
        double B = 0.8328;
        double C = 0.826214;
        double k = 0.06;
        double f = 2;
        double v = 0.3;

        //double Omega = 3.132/1.344;
        //double Phi = 0;


        /*double t2=a1-2.3209+A*cos(Omega*ft+Phi);
        double t3=a1+1.57;
        double t4=a1-2.2424+B*cos(Omega*ft+Phi);
        double t5=a1+0.836686+C*cos(Omega*ft+Phi);
*/
        if(ft==0){
            t21=a1-2.3209-A;
            t31=a1+1.57;
            t41=a1-2.2424-B;
            t51=a1+0.836686-C;
        }

        if (a1 < 0 && theta5 >= 0 && ft > 0.1 && time1 > 0.1){
            time1 = 0;
            t21=a1-2.3209-A;
            t31=a1+1.57;
            t41=a1-2.2424-B;
            t51=a1+0.836686-C;
        }
        if (a1 >= 0 && theta5 < 0 && ft > 0.1 && time1 > 0.1){
            time1 = 0;
            t21=a1-2.3209+A;
            t31=a1+1.57;
            t41=a1-2.2424+B;
            t51=a1+0.836686+C;
        }
        if (a1 < 0 && ft > 0.1){
            if (t21 < theta5-2.3209-0.0001+A){
                t21=a1-2.3209-A+f*A/(1+exp((-time1+v)/k));
                t31=a1+1.57;
                t41=a1-2.2424-B+f*B/(1+exp((-time1+v)/k));
                t51=a1+0.836686-C+f*C/(1+exp((-time1+v)/k));

            }
            else {
                t21=a1-2.3209+A;
                t31=a1+1.57;
                t41=a1-2.2424+B;
                t51=a1+0.836686+C;

            }
        }
        else if (a1 >= 0 || ft <= 0.1){
            if (t21 > theta5-2.3209+0.0001-A){
                t21=a1-2.3209+A-f*A/(1+exp((-time1+v)/k));
                t31=a1+1.57;
                t41=a1-2.2424+B-f*B/(1+exp((-time1+v)/k));
                t51=a1+0.836686+C-f*C/(1+exp((-time1+v)/k));

            }
            else {
                t21=a1-2.3209-A;
                t31=a1+1.57;
                t41=a1-2.2424-B;
                t51=a1+0.836686-C;

            }
        }


        theta5 = a1;

        QPointF p3(position.X()/fScale +d2*sin(a2)/fScale+ width()/2.0, - position.Y()/fScale + height()/2.0 +d2*cos(a2)/fScale);
        QPointF p4(position.X()/fScale +d2*sin(a2)/fScale + l1*sin(t1)/fScale+ width()/2.0, - position.Y()/fScale + height()/2.0 +d2*cos(a2)/fScale +l1*cos(t1)/fScale);
        QPointF p5(position.X()/fScale +d2*sin(a2)/fScale + l1*sin(t1)/fScale + l2*sin(t21)/fScale + width()/2.0, - position.Y()/fScale + height()/2.0 + d2*cos(a2)/fScale + l1*cos(t1)/fScale + l2*cos(t21)/fScale);
        QPointF p6(position.X()/fScale +d2*sin(a2)/fScale + l1*sin(t1)/fScale + l3*sin(t31)/fScale + width()/2.0, - position.Y()/fScale + height()/2.0 + d2*cos(a2)/fScale + l1*cos(t1)/fScale + l3*cos(t31)/fScale);
        QPointF p9(position.X()/fScale +d2*sin(a2)/fScale + l1*sin(t1)/fScale + r2*sin(t21)/fScale + width()/2.0, - position.Y()/fScale + height()/2.0 + d2*cos(a2)/fScale + l1*cos(t1)/fScale + r2*cos(t21)/fScale);
        QPointF p10(position.X()/fScale +d2*sin(a2)/fScale + l1*sin(t1)/fScale + r3*sin(t31)/fScale + width()/2.0, - position.Y()/fScale + height()/2.0 + d2*cos(a2)/fScale + l1*cos(t1)/fScale + r3*cos(t31)/fScale);

        QPointF p7(position.X()/fScale +d2*sin(a2)/fScale + l1*sin(t1)/fScale + r2*sin(t21)/fScale + l4*sin(t41)/fScale + width()/2.0, - position.Y()/fScale + height()/2.0 + d2*cos(a2)/fScale + l1*cos(t1)/fScale + r2*cos(t21)/fScale + l4*cos(t41)/fScale);
        QPointF p8(position.X()/fScale +d2*sin(a2)/fScale + l1*sin(t1)/fScale + r3*sin(t31)/fScale + l5*sin(t51)/fScale + width()/2.0, - position.Y()/fScale + height()/2.0 + d2*cos(a2)/fScale + l1*cos(t1)/fScale + r3*cos(t31)/fScale + l5*cos(t51)/fScale);



        //Choosing pen properties
        pen.setWidth(1);
        pen.setColor(Qt::gray);
        painter.setPen(pen);
        //Draws pendulum "rod"
        painter.drawLine(p1, p2);
        painter.drawLine(p1, p3);
        painter.drawLine(p3, p4);
        painter.drawLine(p4, p9);
        painter.drawLine(p4, p10);
        painter.drawLine(p9, p7);
        painter.drawLine(p10, p8);


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
        painter.drawPoint(p4);
        painter.drawPoint(p5);
        painter.drawPoint(p6);
        painter.drawPoint(p7);
        painter.drawPoint(p8);

        ft = ft + 0.001;
        time1 = time1 + 0.001;
    }
}
