
#include "ros_ceres_helper/BasicOptimisationProblem.h"
#include "ros_ceres_helper/ceres_spline.h"
#include "ros_ceres_helper/ceres_rotations.h"
#include "ros_ceres_helper/ceres_poses.h"

const unsigned int Nk=5;
const unsigned int Nk2=7;
const unsigned int Nkq=10;
const unsigned int Np=10;
const double t[Np] = {1, 2, 3, 4, 5, 5.5, 6, 7, 8, 9};
const double x[Np] = {1.2, 1.5, 2, 3.5, 4, 5.6, 5.8, 6.0, 8.8, 9};
const double y[Np] = {1.5, 0.5, 2.5, 3.5, 3.5, 1.5, -1.8, -2.0, 1.8, 4};
const double z[Np] = {3.5, 2.5, 1.5, 2.5, 3.5, 2.5, 1.8, 2.0, 3.8, 4};


class SplineTestOpt1D : public cerise::BasicOptimisationProblem {
    protected:
        cerise::UniformSpline<cerise::TRefPtrUniformSpline<double,1>> spline;
        std::vector<double> knots;

    public:
        SplineTestOpt1D() : BasicOptimisationProblem(),  spline(0.,10.,Nk), knots(spline.warper.n_knots,0.0) {
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                spline.knots[i] = &knots[i];
            }
            for (unsigned int i=0;i<Np;i++) {
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
            for (size_t i=0;i<Np;i++) {
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


class SplineTestOpt2D : public cerise::BasicOptimisationProblem {
    protected:
        cerise::UniformSpline<cerise::TRefPtrUniformSpline<double,2>> spline;
        double *knots;
        double *points;

    public:
        SplineTestOpt2D() : BasicOptimisationProblem(),  spline(0.,10.,Nk2) {
            knots = new double[spline.warper.n_knots*2];
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                knots[2*i+0] = knots[2*i+1] = 0;
                spline.knots[i] = knots+2*i;
            }
            points = new double[Np*2];
            for (unsigned int i=0;i<Np;i++) {
                points[2*i+0]=y[i];
                points[2*i+1]=z[i];
                std::pair<size_t,double> iu = spline.warper(x[i]);
                ceres::LossFunction* loss_function = NULL;
                ceres::CostFunction *cost_function;
                cost_function = new ceres::AutoDiffCostFunction<cerise::SplineError<2>,2,2,2,2,2>(
                            new cerise::SplineError<2>(iu.second,points+2*i));
                problem->AddResidualBlock(cost_function,loss_function,
                        &knots[2*(iu.first-1)], &knots[2*iu.first], &knots[2*(iu.first+1)], &knots[2*(iu.first+2)]);
            }
        }

        ~SplineTestOpt2D() {
            delete [] points;
            delete [] knots;
        }

        void print() const {
            FILE * fp;
            fp = fopen("input2","w");
            for (size_t i=0;i<Np;i++) {
                fprintf(fp,"%f %f %f\n",x[i],y[i],z[i]);
            }
            fclose(fp);
            fp = fopen("knots2","w");
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                fprintf(fp,"%f %f %f\n",spline.warper.knot(i),knots[2*i+0],knots[2*i+1]);
            }
            fclose(fp);
            fp = fopen("spline2","w");
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.01) {
                double ft[2]={0,0};
                spline.evaluate(t,ft);
                fprintf(fp,"%f %f %f\n",t,ft[0],ft[1]);
            }
            fclose(fp);
        }

};


class SplineTestOptQ : public cerise::BasicOptimisationProblem {
    protected:
        cerise::UniformSpline<cerise::TRefQuaternionUniformSpline<double>> spline;
        double *knots;
        double *points;

    public:
        SplineTestOptQ() : BasicOptimisationProblem(),  spline(0.,10.,Nkq) {
            double aa[3] = {0,0,0};
            knots = new double[spline.warper.n_knots*4];
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                ceres::AngleAxisToQuaternion(aa,knots+4*i);
                spline.knots[i] = knots+4*i;
            }
            points = new double[Np*4];
            for (unsigned int i=0;i<Np;i++) {
                double aa[3] = {x[i],y[i],z[i]};
                ceres::AngleAxisToQuaternion(aa,points+4*i);
                std::pair<size_t,double> iu = spline.warper(t[i]);
                ceres::LossFunction* loss_function = NULL;
                ceres::CostFunction *cost_function;
                cost_function = new ceres::AutoDiffCostFunction<cerise::SplineErrorQ,3,4,4,4,4>(
                            new cerise::SplineErrorQ(iu.second,points+4*i));
                problem->AddResidualBlock(cost_function,loss_function,
                        &knots[4*(iu.first-1)], &knots[4*iu.first], &knots[4*(iu.first+1)], &knots[4*(iu.first+2)]);
            }
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                ceres::LocalParameterization* quaternion_parameterization = NULL;
                quaternion_parameterization = new ceres::QuaternionParameterization;
                problem->SetParameterization(knots+4*i, quaternion_parameterization);
            }
        }

        ~SplineTestOptQ() {
            delete [] points;
            delete [] knots;
        }

        void print() const {
            double aa[3];
            FILE * fp;
            fp = fopen("inputq","w");
            for (size_t i=0;i<Np;i++) {
                ceres::QuaternionToAngleAxis(points+4*i,aa);
                fprintf(fp,"%f %f %f %f %f %f %f %f\n",t[i],points[4*i+0],points[4*i+1],points[4*i+2],points[4*i+3],aa[0],aa[1],aa[2]);
            }
            fclose(fp);
            fp = fopen("knotsq","w");
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                ceres::QuaternionToAngleAxis(knots+4*i,aa);
                fprintf(fp,"%f %f %f %f %f %f %f %f\n",spline.warper.knot(i),knots[4*i+0],knots[4*i+1],knots[4*i+2],knots[4*i+3],aa[0],aa[1],aa[2]);
            }
            fclose(fp);
            fp = fopen("splineq","w");
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.01) {
                double ft[4]={0,0,0,0};
                spline.evaluate(t,ft);
                ceres::QuaternionToAngleAxis(ft,aa);
                fprintf(fp,"%f %f %f %f %f %f %f %f\n",t,ft[0],ft[1],ft[2],ft[3],aa[0],aa[1],aa[2]);
            }
            fclose(fp);
        }

};

class SplineTestOptR : public cerise::BasicOptimisationProblem {
    protected:
        cerise::UniformSpline<cerise::TRefRotationUniformSpline<double>> spline;
        std::vector<cerise::Rotation> points;

    public:
        SplineTestOptR() : BasicOptimisationProblem(),  spline(0.,10.,Nkq), points(Np) {
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                spline.knots[i].setFromAngleAxis(0,0,0);
            }
            for (unsigned int i=0;i<Np;i++) {
                double aa[3] = {x[i],y[i],z[i]};
                points[i].setFromAngleAxis(aa);
                std::pair<size_t,double> iu = spline.warper(t[i]);
                ceres::LossFunction* loss_function = NULL;
                ceres::CostFunction *cost_function;
                cost_function = new ceres::AutoDiffCostFunction<cerise::SplineErrorR,3,4,4,4,4>(
                            new cerise::SplineErrorR(iu.second,points[i]));
                problem->AddResidualBlock(cost_function,loss_function,
                        spline.knots[iu.first-1].Q, spline.knots[iu.first].Q, 
                        spline.knots[iu.first+1].Q, spline.knots[iu.first+2].Q);
            }
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                ceres::LocalParameterization* quaternion_parameterization = NULL;
                quaternion_parameterization = new ceres::QuaternionParameterization;
                problem->SetParameterization(spline.knots[i].Q, quaternion_parameterization);
            }
        }

        ~SplineTestOptR() {
        }

        void print() const {
            double aa[3];
            FILE * fp;
            fp = fopen("inputr","w");
            for (size_t i=0;i<Np;i++) {
                points[i].getAngleAxis(aa);
                fprintf(fp,"%f %f %f %f %f %f %f %f\n",t[i],points[i].Q[0],points[i].Q[1],points[i].Q[2],points[i].Q[3],aa[0],aa[1],aa[2]);
            }
            fclose(fp);
            fp = fopen("knotsr","w");
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                spline.knots[i].getAngleAxis(aa);
                fprintf(fp,"%f %f %f %f %f %f %f %f\n",spline.warper.knot(i),spline.knots[i].Q[0],spline.knots[i].Q[1],spline.knots[i].Q[2],spline.knots[i].Q[3],aa[0],aa[1],aa[2]);
            }
            fclose(fp);
            fp = fopen("spliner","w");
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.01) {
                cerise::Rotation ft;
                spline.evaluate(t,ft);
                ft.getAngleAxis(aa);
                fprintf(fp,"%f %f %f %f %f %f %f %f\n",t,ft.Q[0],ft.Q[1],ft.Q[2],ft.Q[3],aa[0],aa[1],aa[2]);
            }
            fclose(fp);
        }

};

class SplineTestOptP : public cerise::BasicOptimisationProblem {
    protected:
        cerise::UniformSpline<cerise::TRefPoseUniformSpline<double>> spline;
        std::vector<cerise::Pose> points;

    public:
        SplineTestOptP() : BasicOptimisationProblem(),  spline(0.,10.,Nkq), points(Np) {
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                spline.knots[i].setFromAngleAxis(0,0,0,0,0,0);
            }
            for (unsigned int i=0;i<Np;i++) {
                double aa[3] = {x[i],y[i],z[i]};
                points[i].setFromAngleAxis(aa,aa);
                std::pair<size_t,double> iu = spline.warper(t[i]);
                ceres::LossFunction* loss_function = NULL;
                ceres::CostFunction *cost_function;
                cost_function = new ceres::AutoDiffCostFunction<cerise::SplineErrorP,3,3,4,3,4,3,4,3,4>(
                            new cerise::SplineErrorP(iu.second,points[i]));
                problem->AddResidualBlock(cost_function,loss_function,
                        spline.knots[iu.first-1].T, spline.knots[iu.first-1].Q,
                        spline.knots[iu.first].T, spline.knots[iu.first].Q, 
                        spline.knots[iu.first+1].T, spline.knots[iu.first+1].Q, 
                        spline.knots[iu.first+2].T, spline.knots[iu.first+2].Q);
            }
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                ceres::LocalParameterization* quaternion_parameterization = NULL;
                quaternion_parameterization = new ceres::QuaternionParameterization;
                problem->SetParameterization(spline.knots[i].Q, quaternion_parameterization);
            }
        }

        ~SplineTestOptP() {
        }

        void print() const {
            double aa[3];
            FILE * fp;
            fp = fopen("inputp","w");
            for (size_t i=0;i<Np;i++) {
                points[i].getAngleAxis(aa);
                fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f\n",t[i],
                        points[i].T[0],points[i].T[1],points[i].T[2],
                        points[i].Q[0],points[i].Q[1],points[i].Q[2],points[i].Q[3],
                        aa[0],aa[1],aa[2]);
            }
            fclose(fp);
            fp = fopen("knotsp","w");
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                spline.knots[i].getAngleAxis(aa);
                fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f\n",
                        spline.warper.knot(i),
                        spline.knots[i].T[0],spline.knots[i].T[1],spline.knots[i].T[2],
                        spline.knots[i].Q[0],spline.knots[i].Q[1],
                        spline.knots[i].Q[2],spline.knots[i].Q[3],
                        aa[0],aa[1],aa[2]);
            }
            fclose(fp);
            fp = fopen("splinep","w");
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.01) {
                cerise::Pose ft;
                spline.evaluate(t,ft);
                ft.getAngleAxis(aa);
                fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f\n",t,
                        ft.T[0],ft.T[1],ft.T[2],
                        ft.Q[0],ft.Q[1],ft.Q[2],ft.Q[3],
                        aa[0],aa[1],aa[2]);
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
        S.evaluate(t,&v);
        S.cum_evaluate(t,&c);
        fprintf(fp,"%f\t%f\t%f\n",t,v,c);
    }
    fclose(fp);

#if 1
    SplineTestOpt1D to;
    to.optimise();
    to.print();
#endif
#if 1
    SplineTestOpt2D t2;
    t2.optimise();
    t2.print();
#endif
#if 1
    // Useless, just for testing and forcing the compiler to compile everything.
    cerise::TRefQuaternionUniformSpline<double> qS(knots+0,knots+1,knots+2,knots+3);
    SplineTestOptQ tq;
    tq.optimise();
    tq.print();
#endif
#if 1
    cerise::Rotation qknots[4] = {cerise::Rotation(knots+0), cerise::Rotation(knots+1), 
        cerise::Rotation(knots+2), cerise::Rotation(knots+3)}; 
    // Useless, just for testing and forcing the compiler to compile everything.
    cerise::TRefRotationUniformSpline<double> qR(qknots[0],qknots[1],qknots[2],qknots[3]);
    SplineTestOptR tr;
    tr.optimise();
    tr.print();
#endif
    return 0;
}
