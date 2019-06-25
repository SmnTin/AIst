#ifndef __PID__
#define __PID__

#include <cmath>

class PID
{
private:
    double iSum;  // sum of errors for integrator
    // differeciator params
    double old_y; // previous mean of signal
    double cost = 0;
    int cnt = 0;
public:
    explicit PID(double pt = 0, double it = 0,double dt = 0, double MinI = 0,double MaxI = 0);
    double calculate(double error, double y);
    double getCost();

    double kp; // proportional term
    //integrator params
    double ki; // integral term
    double kd;   // differenciator term
    double iMin; // min integrator value
    double iMax;  // max integrator value

};

#endif