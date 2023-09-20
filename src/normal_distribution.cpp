#include <iostream>
#include <cmath>

double normalDistribution(double t, double t0, double sigma) {
    double standardDeviation = sigma;

    double exponent = -0.5 * pow((t - t0) / standardDeviation, 2);
    double coefficient = 1.0 / (standardDeviation * sqrt(2 * M_PI));
    double result = coefficient * exp(exponent);

    return result;
}
double superGaussian(double A,double b,double r,double d)
{

    return A*pow( b, -pow(pow(d,2)/pow(r,2),13*r) );//A*b^(-(d^2)/(r^2)^r);
}
int main() {
    // Parameters for the normal distribution function
    double t0 = 2.0; // Time when the function starts to be activated
    double sigma = 0.6; // Standard deviation

    // Plot the normal distribution function from t0 - 4*sigma to t0 + 4*sigma
    double t;
    double tip_x, tip_z;
    double r1 = 0.005, r2=0.02;
    double freq = 0.5;    
    double A=1,b=10;//A=1,b=10;
    double t0_swing = 0.6, t0_super=0.25; // DO NOT CHANGE
    double t_half_swing = 1.6 ;//;(1/2)/2+t0_super ;
    double swing_t_slot =  2*t_half_swing  ;
    for (t = 0; t < swing_t_slot; t += 0.002) {
        double y = normalDistribution(t, t0, sigma);

        // tip_x = r1*(2*M_PI*freq*(t-t0_swing)-sin(2*M_PI*freq*(t-t0_swing)));
        // tip_z = r2*(1-cos(2*M_PI*freq*(t-t0_swing)));
        
        if(t<t0_swing)
        {
            tip_x= 0.0;
            tip_z= 0.0;
        }
        else if(t>=t0_swing & t<=(t0_swing + 1/freq))
        {
            tip_x= r1*(2*M_PI*freq*(t-t0_swing)-sin(2*M_PI*freq*(t-t0_swing)));
            tip_z= r2*(1-cos(2*M_PI*freq*(t-t0_swing)));

        } 
        // else
        // {
        //     tip_x= tip_x;
        //     tip_z= tip_z;
        // }
        // tip_x = r1*(1-sin(2*M_PI*freq*t));
        // tip_z = sqrt( pow(r2*(1-cos(2*M_PI*freq*t)),2) );

        double d = t - t_half_swing;
        double s = superGaussian(A,b,t_half_swing-t0_super,d);
        std::cout << t << "," << y << ","<< tip_x << ","<< tip_z<<","<<s<<","<<d<<std::endl;
    }
    return 0;
}
