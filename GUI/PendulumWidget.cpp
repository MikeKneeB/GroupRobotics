#include "PendulumWidget.h"
#include <QPainter>
#include <iostream>

PendulumWidget::PendulumWidget(QWidget *parent) :
    QWidget(parent)
{
    fMass=new Mass(20,"simple_pendulum_driven_length_results.dat");
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
        double scale=2;
        double x=pos.getX()*scale;
        double y=pos.getY()*scale;
        painter.drawLine(0+dis,0,x+dis,y+dis);

        fIndex+=10;
    }
}
