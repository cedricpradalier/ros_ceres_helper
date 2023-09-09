
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "ros_ceres_helper/BasicOptimisationProblem.h"
#include "ros_ceres_helper/ceres_spline.h"
#include "ros_ceres_helper/ceres_rotations.h"
#include "ros_ceres_helper/ceres_poses.h"
#include "ros_ceres_helper/markers.h"

const unsigned int Nk=5;
const unsigned int Nk2=7;
const unsigned int Nkq=10;
const unsigned int Nkp=10;
const unsigned int Np=10;
#if 1
const double t[Np] = {1, 2, 3, 4, 5, 5.5, 6, 7, 8, 9};
const double x[Np] = {1.2, 1.5, 2, 3.5, 4, 5.6, 5.8, 6.0, 8.8, 9};
const double y[Np] = {1.5, 0.5, 2.5, 3.5, 3.5, 1.5, -1.8, -2.0, 1.8, 4};
const double z[Np] = {3.5, 2.5, 1.5, 2.5, 3.5, 2.5, 1.8, 2.0, 3.8, 4};
#else
const double t[Np] = {0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5};
const double x[Np] = {0e-1,1e-1,2e-1,3e-1,4e-1,5e-1,6e-1,7e-1,8e-1,9e-1};
const double y[Np] = {0,0,0,0,0,0,0,0,0,0};
const double z[Np] = {0,0,0,0,0,0,0,0,0,0};
#endif

class PublisherWithDataParent {
    protected:
        ros::Publisher P;
    public:
        PublisherWithDataParent(const ros::Publisher & P) : P(P) {}
        virtual ~PublisherWithDataParent() {}
        virtual void publish()=0;
};

template <class T>
    class PublisherWithData : public PublisherWithDataParent {
        protected:
            T data;
        public:
            PublisherWithData(const ros::Publisher & P, const T & data):PublisherWithDataParent(P),data(data)  {}
            virtual ~PublisherWithData() {}
            virtual void publish() {
                P.publish(data);
            }
    };

typedef std::shared_ptr<PublisherWithDataParent> PublisherWithDataPtr;

template <class T>
    PublisherWithDataPtr newPub(const ros::Publisher & P, const T & data) {
        PublisherWithData<T> * pu = new PublisherWithData<T>(P,data);
        return PublisherWithDataPtr(pu);
    }

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

        void collectPublishableData(ros::NodeHandle & nh,
                std::vector<PublisherWithDataPtr> & pub) const {
            ros::Publisher P = nh.advertise<visualization_msgs::MarkerArray>("spline1d",1);
            visualization_msgs::MarkerArray ma;
            cerise::Marker m;
            m.createSphereList(0.2);
            m.setHeader("world");
            m.setColor(1,1,0);
            m.setNameId("input1",0);
            for (size_t i=0;i<Np;i++) {
                m.pushPoint(x[i],y[i],0);
            }
            ma.markers.push_back(m);

            m.createSphereList(0.2);
            m.setHeader("world");
            m.setColor(0,0,1);
            m.setNameId("knots1",0);
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                m.pushPoint(spline.warper.knot(i),knots[i],0);
            }
            ma.markers.push_back(m);
            
            m.createLineStrip(0.05);
            m.setHeader("world");
            m.setColor(0,1,0);
            m.setNameId("spline1",0);
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.01) {
                double ft=0;
                spline.evaluate(t,&ft);
                m.pushPoint(t,ft,0);
            }
            ma.markers.push_back(m);
           
            pub.push_back(newPub(P,ma));
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


        void collectPublishableData(ros::NodeHandle & nh,
                std::vector<PublisherWithDataPtr> & pub) const {
            ros::Publisher P = nh.advertise<visualization_msgs::MarkerArray>("spline2d",1);
            visualization_msgs::MarkerArray ma;
            cerise::Marker m;
            m.createSphereList(0.2);
            m.setHeader("world");
            m.setColor(1,1,0);
            m.setNameId("input2",0);
            for (size_t i=0;i<Np;i++) {
                m.pushPoint(x[i],y[i],z[i]);
            }
            ma.markers.push_back(m);

            m.createSphereList(0.2);
            m.setHeader("world");
            m.setColor(0,0,1);
            m.setNameId("knots2",0);
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                m.pushPoint(spline.warper.knot(i),knots[2*i+0],knots[2*i+1]);
            }
            ma.markers.push_back(m);
            
            m.createLineStrip(0.05);
            m.setHeader("world");
            m.setColor(0,1,0);
            m.setNameId("spline2",0);
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.01) {
                double ft[2]={0,0};
                spline.evaluate(t,ft);
                m.pushPoint(t,ft[0],ft[1]);
            }
            ma.markers.push_back(m);

            pub.push_back(newPub(P,ma));
           
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
            spline.resetUsed();
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
                spline.recordUsed(iu.first);
            }
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                if (!spline.used[i]) {
                    continue;
                }
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

        void collectPublishableData(ros::NodeHandle & nh,
                std::vector<PublisherWithDataPtr> & pub) const {
            ros::Publisher P = nh.advertise<geometry_msgs::PoseArray>("splineq_input",1);
            geometry_msgs::PoseArray pa;
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (size_t i=0;i<Np;i++) {
                double * q = points+4*i;
                geometry_msgs::Pose p;
                p.position.x=t[i]; p.position.y=0; p.position.z=0;
                p.orientation.w=q[0]; p.orientation.x=q[1]; 
                p.orientation.y=q[2]; p.orientation.z=q[3];
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

            P = nh.advertise<geometry_msgs::PoseArray>("splineq_knots",1);
            pa.poses.clear();
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                double * q = knots+4*i;
                geometry_msgs::Pose p;
                p.position.x=t[i]; p.position.y=-0.5; p.position.z=0;
                p.orientation.w=q[0]; p.orientation.x=q[1]; 
                p.orientation.y=q[2]; p.orientation.z=q[3];
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

            P = nh.advertise<geometry_msgs::PoseArray>("splineq_curve",1);
            pa.poses.clear();
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.1) {
                double q[4]={0,0,0,0};
                spline.evaluate(t,q);
                geometry_msgs::Pose p;
                p.position.x=t; p.position.y=0.55; p.position.z=0;
                p.orientation.w=q[0]; p.orientation.x=q[1]; 
                p.orientation.y=q[2]; p.orientation.z=q[3];
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

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
            std::vector<bool> used(spline.warper.n_knots,false);
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
                used[iu.first-1]=true;
                used[iu.first]=true;
                used[iu.first+1]=true;
                used[iu.first+2]=true;
            }
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                if (!used[i]) {
                    continue;
                }
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

        void collectPublishableData(ros::NodeHandle & nh,
                std::vector<PublisherWithDataPtr> & pub) const {
            ros::Publisher P = nh.advertise<geometry_msgs::PoseArray>("spliner_input",1);
            geometry_msgs::PoseArray pa;
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (size_t i=0;i<Np;i++) {
                const double * q = points[i].Q;
                geometry_msgs::Pose p;
                p.position.x=t[i]; p.position.y=0; p.position.z=0;
                p.orientation.w=q[0]; p.orientation.x=q[1]; 
                p.orientation.y=q[2]; p.orientation.z=q[3];
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

            P = nh.advertise<geometry_msgs::PoseArray>("spliner_knots",1);
            pa.poses.clear();
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                const double * q = spline.knots[i].Q;
                geometry_msgs::Pose p;
                p.position.x=t[i]; p.position.y=1; p.position.z=0;
                p.orientation.w=q[0]; p.orientation.x=q[1]; 
                p.orientation.y=q[2]; p.orientation.z=q[3];
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

            P = nh.advertise<geometry_msgs::PoseArray>("spliner_curve",1);
            pa.poses.clear();
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.1) {
                cerise::Rotation Q;
                spline.evaluate(t,Q);
                const double * q = Q.Q;
                geometry_msgs::Pose p;
                p.position.x=t; p.position.y=2; p.position.z=0;
                p.orientation.w=q[0]; p.orientation.x=q[1]; 
                p.orientation.y=q[2]; p.orientation.z=q[3];
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

        }
};

class SplineTestOptP : public cerise::BasicOptimisationProblem {
    protected:
        cerise::UniformSpline<cerise::TRefPoseUniformSpline<double>> spline;
        std::vector<cerise::Pose> points;

    public:
        SplineTestOptP() : BasicOptimisationProblem(),  spline(0.,10.,Nkp), points(Np) {
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                spline.knots[i].setFromAngleAxis(0,0,0,0,0,0);
            }
            std::vector<bool> used(spline.warper.n_knots,false);
            for (unsigned int i=0;i<Np;i++) {
                double aa[3] = {x[i],y[i],z[i]};
                points[i].setFromAngleAxis(aa,aa);
                std::pair<size_t,double> iu = spline.warper(t[i]);
                ceres::LossFunction* loss_function = NULL;
                ceres::CostFunction *cost_function;
                cost_function = new ceres::AutoDiffCostFunction<cerise::SplineErrorP,6,3,4,3,4,3,4,3,4>(
                            new cerise::SplineErrorP(iu.second,points[i]));
                problem->AddResidualBlock(cost_function,loss_function,
                        spline.knots[iu.first-1].T, spline.knots[iu.first-1].Q,
                        spline.knots[iu.first].T, spline.knots[iu.first].Q, 
                        spline.knots[iu.first+1].T, spline.knots[iu.first+1].Q, 
                        spline.knots[iu.first+2].T, spline.knots[iu.first+2].Q);
                used[iu.first-1]=true;
                used[iu.first]=true;
                used[iu.first+1]=true;
                used[iu.first+2]=true;
            }
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                if (!used[i]) {
                    continue;
                }
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


        void collectPublishableData(ros::NodeHandle & nh,
                std::vector<PublisherWithDataPtr> & pub) const {
            ros::Publisher P = nh.advertise<geometry_msgs::PoseArray>("splinep_input",1);
            geometry_msgs::PoseArray pa;
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (size_t i=0;i<Np;i++) {
                const double * t = points[i].T;
                const double * q = points[i].Q;
                geometry_msgs::Pose p;
                p.position.x=t[0]; p.position.y=t[1]; p.position.z=t[2];
                p.orientation.w=q[0]; p.orientation.x=q[1]; 
                p.orientation.y=q[2]; p.orientation.z=q[3];
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

            P = nh.advertise<geometry_msgs::PoseArray>("splinep_knots",1);
            pa.poses.clear();
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (size_t i=0;i<spline.warper.n_knots;i++) {
                const double * t = spline.knots[i].T;
                const double * q = spline.knots[i].Q;
                geometry_msgs::Pose p;
                p.position.x=t[0]; p.position.y=t[1]; p.position.z=t[2];
                p.orientation.w=q[0]; p.orientation.x=q[1]; 
                p.orientation.y=q[2]; p.orientation.z=q[3];
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

            P = nh.advertise<geometry_msgs::PoseArray>("splinep_curve",1);
            pa.poses.clear();
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (double t=spline.warper.min();t<spline.warper.max();t+=0.1) {
                cerise::Pose Q;
                spline.evaluate(t,Q);
                const double * q = Q.Q;
                geometry_msgs::Pose p;
                p.position.x=Q.T[0]; p.position.y=Q.T[1]; p.position.z=Q.T[2];
                p.orientation.w=q[0]; p.orientation.x=q[1]; 
                p.orientation.y=q[2]; p.orientation.z=q[3];
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

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

    ros::init(argc,argv,"test_ceres_spline");
    ros::NodeHandle nh("~");
    std::vector<PublisherWithDataPtr> pubdata;

#if 1
    printf("\n\n SplineTestOpt1D \n\n");
    SplineTestOpt1D to;
    to.optimise();
    to.print();
    to.collectPublishableData(nh,pubdata);
#endif
#if 1
    printf("\n\n SplineTestOpt2D \n\n");
    SplineTestOpt2D t2;
    t2.optimise();
    t2.print();
    t2.collectPublishableData(nh,pubdata);
#endif
#if 1
    printf("\n\n SplineTestOptQ \n\n");
    SplineTestOptQ tq;
    tq.optimise();
    tq.print();
    tq.collectPublishableData(nh,pubdata);
#endif
#if 1
    printf("\n\n SplineTestOptR \n\n");
    SplineTestOptR tr;
    tr.optimise();
    tr.print();
    tr.collectPublishableData(nh,pubdata);
#endif

#if 1
    printf("\n\n SplineTestOptP \n\n");
    SplineTestOptP tp;
    tp.optimise();
    tp.print();
    tp.collectPublishableData(nh,pubdata);
#endif

    printf("Collected %d publishers\n",int(pubdata.size()));
    ros::Rate rate(5);
    while (ros::ok()) {
        for (size_t i=0;i<pubdata.size();i++) {
            // printf("Publish %d\n",int(i));
            pubdata[i]->publish();
        }
        rate.sleep();
    }
    return 0;
}
