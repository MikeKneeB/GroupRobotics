
#include <QApplication>
#include <QDebug>
#include <Robot.h>
#include <iostream>
using namespace std;

int main()
{

    double pi = atan(1)*4;

    //Creating robot and body parts
    Robot * weeBot = new Robot(TwoVector(0,0));
    BodyPart * leg = new BodyPart(weeBot,1,pi/2,10);
    BodyPart * foot = new BodyPart(leg,1,0,10);

    //adding body parts to robot
    weeBot->addBodyPart(leg);
    weeBot->addBodyPart(foot);

    //robot with leg and foot in same direction
   foot->getPosition().print();

for (int i = 0; i < 1000000; i++){
    //move foot 90 degrees relative to leg
   leg->move(0,1,0.001);
}
    foot->getPosition().print();









    return 0;
}
