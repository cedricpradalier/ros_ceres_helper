
#include "ros_ceres_helper/BasicOptimisationProblem.h"
#include "ros_ceres_helper/ceres_spline.h"

double x[10] = {1.2, 1.5, 2, 3.5, 4, 5.6, 5.8, 6.0, 8.8, 9};
double y[10] = {1.5, 0.5, 2.5, 3.5, 3.5, 1.5, -1.8, -2.0, 1.8, 4};

struct SplineError {
    double u,y;
    SplineError(double u, double y) : u(u), y(y) {}


    template <typename T>
        bool operator()(const T *const k1, const T* const k2, const T* const k3, const T *const k4,
                T* residuals) const {
            Eigen::Matrix<T, 4, 1> Mu = cerise::spline_B(T(u));
            Eigen::Matrix<T, 1, 4> V;
            V << k1[0],k2[0],k3[0],k4[0];
            T out = (V * Mu)(0);
            residuals[0] = out - y;
            return true;
        }
};

class SplineTestOpt : public cerise::BasicOptimisationProblem {
    protected:
        std::vector<double> knots;


    public:
        SplineTestOpt() : BasicOptimisationProblem(), knots(5,0.0) {
            for (int i=0;i<10;i++) {
                int j = floor(x[i]);
                j = std::min<int>(std::max<int>(j,1),knots.size()-3);
                double u = x[i] - j;
                ceres::LossFunction* loss_function = NULL;
                ceres::CostFunction *cost_function;
                cost_function = new ceres::AutoDiffCostFunction<SplineError,1,1,1,1,1>(
                            new SplineError(u,y[i]));
                problem->AddResidualBlock(cost_function,loss_function,
                        &knots[i-1], &knots[i], &knots[i+1], &knots[i+2]);
            }
        }

        void print() const {
            for (size_t i=0;i<knots.size();i++) {
                printf("%d %f\n",int(i),knots[i]);
            }
        }

};


int main(int argc, char * argv[]) {
    
    double knots[8] = {1,0,1,-1,-2,-1,3,1};

    cerise::TRefUniformSpline<double> S(knots+0, knots+8);

    FILE * fp=fopen("stest","w");
    for (double t=1;t<=6;t+=0.01) {
        double v = S.evaluate(t);
        double c = S.cum_evaluate(t);
        fprintf(fp,"%f\t%f\t%f\n",t,v,c);
    }
    fclose(fp);

    SplineTestOpt to;
    to.optimise();
    to.print();
    return 0;
}
