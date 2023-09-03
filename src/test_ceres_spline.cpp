
#include "ros_ceres_helper/BasicOptimisationProblem.h"
#include "ros_ceres_helper/ceres_spline.h"

double x[10] = {1.2, 1.5, 2, 3.5, 4, 5.6, 5.8, 6.0, 8.8, 9};
double y[10] = {1.5, 0.5, 2.5, 3.5, 3.5, 1.5, -1.8, -2.0, 1.8, 4};


class SplineTestOpt : public cerise::BasicOptimisationProblem {
    protected:
        cerise::UniformSpline<cerise::TRefPtrUniformSpline<double,1>> spline;
        std::vector<double> knots;

    public:
        SplineTestOpt() : BasicOptimisationProblem(),  spline(0.,10.,5), knots(5,0.0) {
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                spline.knots[i] = &knots[i];
            }
            for (int i=0;i<10;i++) {
                std::pair<size_t,double> iu = spline.warper(x[i]);
                ceres::LossFunction* loss_function = NULL;
                ceres::CostFunction *cost_function;
                cost_function = new ceres::AutoDiffCostFunction<cerise::SplineError<1>,1,1,1,1,1>(
                            new cerise::SplineError<1>(iu.second,y+i));
                problem->AddResidualBlock(cost_function,loss_function,
                        &knots[iu.first-1], &knots[iu.first], &knots[iu.first+1], &knots[iu.first+2]);
            }
        }

        void print() const {
            FILE * fp;
            fp = fopen("input","w");
            for (size_t i=0;i<10;i++) {
                fprintf(fp,"%f %f\n",x[i],y[i]);
            }
            fclose(fp);
            fp = fopen("knots","w");
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                fprintf(fp,"%f %f\n",spline.warper.knot(i),knots[i]);
            }
            fclose(fp);
            fp = fopen("spline","w");
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.01) {
                double ft=0;
                spline.evaluate(t,&ft);
                fprintf(fp,"%f %f\n",t,ft);
            }
            fclose(fp);
        }

};


int main(int argc, char * argv[]) {
    
    double knots[8] = {1,0,1,-1,-2,-1,3,1};

    cerise::GenericSpline1D S(0.,7.,8);
    S.import(knots);

    FILE * fp;
    fp=fopen("sknots","w");
    for (size_t i=0;i<S.knots.size();i++) {
        fprintf(fp,"%f %f\n",S.warper.knot(i),S.knots[i]);
    }
    fclose(fp);
    fp=fopen("stest","w");
    for (double t=1;t<=6;t+=0.01) {
        double v=0,c=0;
        S.evaluate(t,v);
        S.cum_evaluate(t,c);
        fprintf(fp,"%f\t%f\t%f\n",t,v,c);
    }
    fclose(fp);

#if 1
    SplineTestOpt to;
    to.optimise();
    to.print();
#endif
    return 0;
}
