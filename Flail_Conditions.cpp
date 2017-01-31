
//This is for without a hinge

        double r = 1.0;
        double l2 = 0.2;
        double l3 = 0.2;
        double A = 0.6;

        //double l = 1.0;
        double B = 0.4;
        double m3 = 10;
        double Omega = 3.132*1.32;
        double Phi = -3.142/2;
        double m1 = 10;
        double m2 = 3;
        double M = m1+m2+m3;

        double g = 9.81;
        //double r0 = 1.0;
        double tDot = fPendulums[iComponent]->GetThetaDot();

		//Note that iTheta is actually alpha
        double theta2 = -A + A*cos(Omega*t)+3.142;
        double theta2DDot = -Omega*Omega*A*cos(Omega*t);
        double theta2Dot = -Omega*A*sin(Omega*t);
        double theta3 = -B + B*cos(Omega*t);
        double theta3DDot = -Omega*Omega*B*cos(Omega*t);
        double theta3Dot = -Omega*B*cos(Omega*t);

        //Up/Down Motion
        //double NewAccel = (2*m2*r0*tDot*Omega*A*sin(Omega*t+Phi) + 2*m2*tDot*Omega*A*A*sin(Omega*t+Phi)*cos(Omega*t+Phi) - m1*l*g*sin(iTheta) - m2*r0*g*sin(iTheta) - m2*g*A*cos(Omega*t+Phi)*sin(iTheta))/(m2*r0*r0 + m2*A*A*cos(Omega*t+Phi)*cos(Omega*t+Phi) + m1*l*l);

        //Left to Right Motion
        //double NewAccel = (-m1*(-2*A*A*Omega*tDot*sin(Omega*t+Phi)*cos(Omega*t+Phi) - l*A*A*Omega*Omega*cos(Omega*t+Phi) + g*l*sin(iTheta) + g*A*cos(iTheta)*cos(Omega*t+Phi)) - m2*g*l*sin(iTheta))/(m1*l*l+m2*l*l+m1*A*A*cos(Omega*t+Phi)*cos(Omega*t+Phi));

        //Flail Model
        double NewAccel = (-m2*l2*(theta2DDot*cos(iTheta-theta2)-theta2Dot*(tDot-theta2Dot)*sin(iTheta-theta2)) - m3*l3*(theta3DDot*cos(iTheta-theta3)-theta3Dot*(tDot-theta3Dot)*sin(iTheta-theta3)) - m2*l2*tDot*theta2Dot*sin(iTheta-theta2)- m3*l3*tDot*theta3Dot*sin(iTheta-theta3) - M*g*sin(iTheta))/(M*r);

