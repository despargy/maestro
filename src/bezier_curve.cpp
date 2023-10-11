//// g++ bezier_curve.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>

#include "/home/despargy/Desktop/Codes/matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

//-----------------------------------------------------------

std::tuple<std::vector<double>, std::vector<double>> computeBesierCurve2D(std::vector<double> xX, std::vector<double> yY)
{

    std::vector<double> bCurveX;
    std::vector<double> bCurveY;
    double bCurveXt;
    double bCurveYt;

    for (double t = 0.0; t <= 1; t += (0.002))
    {

        bCurveXt = std::pow((1 - t), 3) * xX[0] + 3 * std::pow((1 - t), 2) * t * xX[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * xX[2] + std::pow(t, 3) * xX[3];
        bCurveYt = std::pow((1 - t), 3) * yY[0] + 3 * std::pow((1 - t), 2) * t * yY[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * yY[2] + std::pow(t, 3) * yY[3];

        bCurveX.push_back(bCurveXt);
        bCurveY.push_back(bCurveYt);

    }

    return std::make_tuple(bCurveX, bCurveY);
}

//-----------------------------------------------------------

void plot2D(std::tuple<std::vector<double>, std::vector<double>> data)
{

    std::vector<double> xX = std::get<0>(data);
    std::vector<double> yY = std::get<1>(data);


    plt::plot(xX, yY);
    plt::show();
}

//-----------------------------------------------------------

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> computeBesierCurve3D(std::vector<double> xX, std::vector<double> yY, std::vector<double> zZ)
{

    std::vector<double> bCurveX;
    std::vector<double> bCurveY;
    std::vector<double> bCurveZ;
    double bCurveXt;
    double bCurveYt;
    double bCurveZt;

    for (double t = 0.01; t <= 1; t += 0.01)
    {

        bCurveXt = std::pow((1 - t), 3) * xX[0] + 3 * std::pow((1 - t), 2) * t * xX[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * xX[2] + std::pow(t, 3) * xX[3];
        bCurveYt = std::pow((1 - t), 3) * yY[0] + 3 * std::pow((1 - t), 2) * t * yY[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * yY[2] + std::pow(t, 3) * yY[3];
        bCurveZt = std::pow((1 - t), 3) * yY[0] + 3 * std::pow((1 - t), 2) * t * yY[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * yY[2] + std::pow(t, 3) * yY[3];

        bCurveX.push_back(bCurveXt);
        bCurveY.push_back(bCurveYt);
        bCurveZ.push_back(bCurveZt);
    }

    return std::make_tuple(bCurveX, bCurveY, bCurveZ);
}

//-----------------------------------------------------------

void plot3Dexample()
{

    std::vector<double> xX;
    std::vector<double> yY;
    std::vector<double> zZ;
    double theta;
    double r;
    double z_inc = 4.0 / 99.0;
    double theta_inc = (8.0 * M_PI) / 99.0;

    for (double i = 0; i < 100; i += 1)
    {
        theta = -4.0 * M_PI + theta_inc * i;
        zZ.push_back(-2.0 + z_inc * i);
        r = zZ[i] * zZ[i] + 1;
        xX.push_back(r * std::sin(theta));
        yY.push_back(r * std::cos(theta));
    }

    plt::plot3(xX, yY, zZ);
    plt::show();
}

//-----------------------------------------------------------

void plot3D(std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> data)
{

    std::vector<double> xX = std::get<0>(data);
    std::vector<double> yY = std::get<1>(data);
    std::vector<double> zZ = std::get<2>(data);

    plt::plot3(xX, yY, zZ);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("z");
    plt::show();
}

//-----------------------------------------------------------

double computeBinominal(int n, int k)
{

    double value = 1.0;

    for (int i = 1; i <= k; i++)
    {

        value = value * ((n + 1 - i) / i);
    }

    if (n == k){
        value = 1;
    }
    
    return value;
}

//-----------------------------------------------------------

std::tuple<std::vector<double>, std::vector<double>> computeNVertexBasierCurve2D(std::vector<double> xX, std::vector<double> yY)
{

    std::vector<double> bCurveX;
    std::vector<double> bCurveY;

    int n = xX.size() - 1;
    std::cout << "n :" << n << "\n";

    for (double t = 0.0; t <= 1.0; t += 0.01)
    {

        double bCurveXt{0};
        double bCurveYt{0};

        for (int i = 0; i <= n; ++i)
        {

            bCurveXt += computeBinominal(n, i) * std::pow((1 - t), (n - i)) * std::pow(t, i) * xX[i];
            bCurveYt += computeBinominal(n, i) * std::pow((1 - t), (n - i)) * std::pow(t, i) * yY[i];
            //std::cout << " t= "<< t<< " i=" << i << " bCurveXt=" << bCurveXt << " = " << computeBinominal(n, i)  << " * " << std::pow((1 - t), (n - i))  << " * " << std::pow(t, i) << " * " << xX[i] << std::endl;
        }

        bCurveX.push_back(bCurveXt);
        bCurveY.push_back(bCurveYt);
    }

    return std::make_tuple(bCurveX, bCurveY);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> computeNVertexBasierCurve3D(std::vector<double> xX, std::vector<double> yY, std::vector<double> zZ)
{

    std::vector<double> bCurveX;
    std::vector<double> bCurveY;
    std::vector<double> bCurveZ;

    int n = xX.size() - 1;
    std::cout << "n :" << n << "\n";

    for (double t = 0.0; t <= 1.0; t += 0.01)
    {

        double bCurveXt{0};
        double bCurveYt{0};
        double bCurveZt{0};

        for (int i = 0; i <= n; ++i)
        {

            bCurveXt += computeBinominal(n, i) * std::pow((1 - t), (n - i)) * std::pow(t, i) * xX[i];
            bCurveYt += computeBinominal(n, i) * std::pow((1 - t), (n - i)) * std::pow(t, i) * yY[i];
            bCurveZt += computeBinominal(n, i) * std::pow((1 - t), (n - i)) * std::pow(t, i) * zZ[i];
            //std::cout << " t= "<< t<< " i=" << i << " bCurveXt=" << bCurveXt << " = " << computeBinominal(n, i)  << " * " << std::pow((1 - t), (n - i))  << " * " << std::pow(t, i) << " * " << xX[i] << std::endl;
        }

        bCurveX.push_back(bCurveXt);
        bCurveY.push_back(bCurveYt);
        bCurveZ.push_back(bCurveZt);
    }

    return std::make_tuple(bCurveX, bCurveY, bCurveZ);
}


//-----------------------------------------------------------

int main()
{

    std::vector<double> xX{0.0, 0.08, 0.09, 0.07};
    std::vector<double> zZ{0.0, 0.1, 0.05, 0.0};

    std::tuple<std::vector<double>, std::vector<double>> bCurve2D = computeBesierCurve2D(xX, zZ);
    plot2D(bCurve2D);
    
    // std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> bCurve3D = computeBesierCurve3D(xX, yY, zZ);
    // plot3D(bCurve3D);
    
    // std::vector<double> xXn{2.5, 1.5, 6, 10, 7, 3};
    // std::vector<double> yYn{0.5, 5, 5, 0.5, 1.0 , 2.0};
    // std::vector<double> zZn{1, 2, 3, 4, 5, 0.1};

    // std::tuple<std::vector<double>, std::vector<double>> bCurve2DxN = computeNVertexBasierCurve2D(xXn, yYn);
    // plot2D(bCurve2DxN);
    
    // std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> bCurve3DxN = computeNVertexBasierCurve3D(xXn, yYn, zZn);
    // plot3D(bCurve3DxN);

}