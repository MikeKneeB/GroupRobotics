#include "PendulumWidget.h"
#include <QPainter>
#include <iostream>
#include <string>

PendulumWidget::PendulumWidget(QWidget *parent) :
    QWidget(parent)
{

    //Uncomment for windows - keep file in build folder
//    std::string filename="simple_pendulum_driven_length_results.dat";
//    fMass=new Mass(20,filename);
    //Uncomment for OSX - need full path name
    fMass=new Mass(20,"/Users/hargwit/Documents/Git/Robotics/build-PendulumSim-Desktop_Qt_5_7_0_clang_64bit-Debug/simple_pendulum_driven_length_results.dat");

    fIndex=0;
}

PendulumWidget::~PendulumWidget(){}

void PendulumWidget::paintEvent(QPaintEvent *event){
    if(fIndex<=fMass->getLength()){
        //Set the painter to this Widget
        QPainter painter(this);
        //Set the background color
        QColor background(255,255,255);
        //Draw over the last image with a rectangle to clear
        painter.fillRect(QRect(0, 0, width(), height()),background);
        //Set ball color
        QColor lineColor(0,0,0);

        TwoVector pos = fMass->readElement(fIndex);

        painter.setBrush(lineColor);
        painter.setPen(lineColor);

        int dis=width()/2;
        double scale=5;
        double x=pos.getX()*scale;
        double y=pos.getY()*scale;
        double length=20*scale;
        double lineScale=length/sqrt(pow(x,2)+pow(y,2));
        double lineX=x*lineScale;
        double lineY=y*lineScale;

        painter.drawLine(0+dis,0+dis,lineX+dis,lineY+dis);
        int rad=5;
        painter.drawEllipse(x+dis-rad,y+dis-rad,2*rad,2*rad);

        fIndex+=20;
    } else {
        fIndex=0;
    }
}
