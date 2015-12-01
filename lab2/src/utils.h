#include <Eigen/Dense>
#include <math.h>
#include <random>

using namespace Eigen;

double drand() { 
    return (double)rand() / RAND_MAX;
}

double constrainAngle(double theta) {
    theta = fmod(theta + M_PI, 2 * M_PI);
    if (theta < 0)
        theta += 2 * M_PI;
        
    return theta - M_PI;
}

// Evaluates the normal PDF with provided mean and covariance at a given x
template<int k>
double normpdf(Matrix<double, k, 1> pose, Matrix<double, k, 1> mu, Matrix<double, k, k> sigma) {    
    auto det = sigma.determinant();
    if (det == 0) {
        return pose == mu ? std::numeric_limits<double>::infinity() : 0;
    }
    
    auto diff = pose - mu;
    
    return (1 / sqrt(pow(2*M_PI, k) * det)) * exp(-0.5 * diff.transpose() * sigma.inverse() * diff);
}

double sample_normal(double sd) {
    double sum = 0;
    for(int i = 0; i < 12; i++) {
        sum += (drand() - 0.5) * 2 * sd;
    }
    return 0.5 * sum;
}

short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//      vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);
    
    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }
    
    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;
    
    x.push_back(x0);
    y.push_back(y0);
    
    while (x0 != x1 || y0 != y1) {
        if (s) 
            y0+=sgn(dy2); 
        else 
            x0+=sgn(dx2);
            

        if (d < 0) 
            d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }
        
        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}