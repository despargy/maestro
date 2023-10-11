#include <iostream>
#include <vector>

// Define a 3D point structure
struct Point3D {
    double x, y, z;
};

// Function to calculate a point on a Bezier curve
Point3D calculateBezierPoint(const std::vector<Point3D>& controlPoints, double t) {
    int n = controlPoints.size() - 1;
    Point3D point = {0.0, 0.0, 0.0};

    for (int i = 0; i <= n; ++i) {
        double coeff = 1.0;
        for (int j = 0; j <= n; ++j) {
            if (j == i) continue;
            coeff *= (t - (double)j) / ((double)i - (double)j);
        }
        point.x += coeff * controlPoints[i].x;
        point.y += coeff * controlPoints[i].y;
        point.z += coeff * controlPoints[i].z;
    }

    return point;
}

// Function to generate a Bezier curve trajectory
std::vector<Point3D> generateBezierTrajectory(const std::vector<Point3D>& controlPoints, int numSteps) {
    std::vector<Point3D> trajectory;

    for (int i = 0; i <= numSteps; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(numSteps);
        Point3D point = calculateBezierPoint(controlPoints, t);
        trajectory.push_back(point);
    }

    return trajectory;
}

int main() 
{

    

    // Define control points for the Bezier curve
    std::vector<Point3D> controlPoints = {
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 2.0},
        {6.0, 0.0, 4.0},
        {10.0, 0.0, 0.0}
    };

    int numSteps = 500;

    std::vector<Point3D> bezierTrajectory = generateBezierTrajectory(controlPoints, numSteps);

    // Print the generated Bezier curve trajectory
    for (const Point3D& point : bezierTrajectory) {
        std::cout <<  point.x <<"," << point.y << "," << point.z << std::endl;
    }

    return 0;
}
