
#include "ros_ceres_helper/ceres_spline.h"



int main(int argc, char * argv[]) {
    
    double knots[8] = {1,0,1,-1,-2,-1,3,1};

    cerise::TRefUniformSpline<double> S(knots+0, knots+8);

    for (double t=1;t<=6;t+=0.01) {
        double v = S.evaluate(t);
        double c = S.cum_evaluate(t);
        printf("%f\t%f\t%f\n",t,v,c);
    }

    return 0;
}
