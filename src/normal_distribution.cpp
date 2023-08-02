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

    return A*pow( b, -pow(pow(d,2)/pow(r,2),r) );//A*b^(-(d^2)/(r^2)^r);
}
int main() {
    // Parameters for the normal distribution function
    double t0 = 2.0; // Time when the function starts to be activated
    double sigma = 0.6; // Standard deviation

    // Plot the normal distribution function from t0 - 4*sigma to t0 + 4*sigma
    double t;
    double tip_x, tip_z;
    double r1 = 0.01, r2=0.01;
    double freq = 0.5;
    // for (t = t0 - 4.0 * sigma; t <= t0 + 4.0 * sigma; t += 0.002) {
    //     double y = normalDistribution(t, t0, sigma);
    //     std::cout << t << "," << y << std::endl;
    // }q
    
    double A=1,b=2;
    double t_swing = 4, t0_swing = 2;

    double t0_tip = 1;
    for (t = 0; t <= 9; t += 0.002) {
        double y = normalDistribution(t, t0, sigma);

        tip_x = r1*(2*M_PI*freq*t-sin(2*M_PI*freq*t));
        tip_z = r2*(1-cos(2*M_PI*freq*t));

        double d = t - t_swing;
        double s = superGaussian(A,b,t_swing-t0_swing,d);
        std::cout << t << "," << y << ","<< tip_x << ","<< tip_z<<","<<s<<","<<d<<std::endl;
    }
    return 0;
}
