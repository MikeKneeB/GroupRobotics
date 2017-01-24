#ifndef PENDULUMWIDGET_H
#define PENDULUMWIDGET_H

#include <QWidget>
#include "Mass.h"

class PendulumWidget : public QWidget
{
    Q_OBJECT
public:
    explicit PendulumWidget(QWidget *parent = 0);
    virtual ~PendulumWidget();
    //Paints on update
    void paintEvent(QPaintEvent *event);

private:
    Mass* fMass;
    int fIndex;
};

#endif // PENDULUMWIDGET_H
