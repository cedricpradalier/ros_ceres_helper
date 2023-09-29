
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <sophus/ceres_manifold.hpp>
#include <sophus/spline.hpp>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

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


template <template <typename> class LieGroup_>
struct SplineFunctor {
    template <typename T>
        using LieGroup = LieGroup_<T>;
    using LieGroupd = LieGroup<double>;



    template <typename T>
        bool operator()(const T* P0, const T* P1, const T* P2, const T* P3, 
                T* residuals) const {
            using LieGroupT = LieGroup<T>;
            if (segment_case != Sophus::SegmentCase::normal) {
                std::cerr << "Invalid segment_case in spline functor (4)" << std::endl;
                return false;
            }
            Sophus::BasisSplineSegment<LieGroupT> s(segment_case,P0,P1,P2,P3);
            LieGroupT pred = s.parent_T_spline(u);
            LieGroupT diff = y.inverse() * pred;
            using Mapper = Sophus::Mapper<typename LieGroupT::Tangent>;
            typename Mapper::Map diff_log = Mapper::map(residuals);

            // Jet LieGroup multiplication with LieGroupd
            diff_log = diff.log();
            return true;
        }


    template <typename T>
        bool operator()(const T* P0, const T* P1, const T* P2, 
                T* residuals) const {
            using LieGroupT = LieGroup<T>;
            LieGroupT pred;
            switch (segment_case) {
                case Sophus::SegmentCase::first:
                    { 
                        Sophus::BasisSplineSegment<LieGroupT> s(segment_case,P0,P0,P1,P2);
                        pred = s.parent_T_spline(u);
                    }
                    break;
                case Sophus::SegmentCase::last:
                    { 
                        Sophus::BasisSplineSegment<LieGroupT> s(segment_case,P0,P1,P2,P2);
                        pred = s.parent_T_spline(u);
                    }
                    break;
                default:
                    std::cerr << "Invalid segment_case in spline functor (3)" << std::endl;
                    return false;
            }
            LieGroupT diff = y.inverse() * pred;
            using Mapper = Sophus::Mapper<typename LieGroupT::Tangent>;
            typename Mapper::Map diff_log = Mapper::map(residuals);

            // Jet LieGroup multiplication with LieGroupd
            diff_log = diff.log();
            return true;
        }


    SplineFunctor(Sophus::SegmentCase scase, double u, const LieGroupd & yin) : 
        segment_case(scase), u(u), y(yin){
        }
    
    Sophus::SegmentCase segment_case;
    double u;
    const LieGroupd y;

};




template <template <typename, int = 0> class LieGroup_>
class SplineTestOptLieGroup : public cerise::BasicOptimisationProblem {
    protected:
        template <typename T>
            using LieGroup = LieGroup_<T>;
        using LieGroupd = LieGroup<double>;
    protected:
        
        Sophus::BasisSpline<LieGroupd> spline;
        std::vector<LieGroupd> points;

        virtual LieGroupd getPoint(size_t i) const = 0;

        typedef enum {input, knots, curve} DataType;

        virtual void fprintf(FILE * fp, DataType dt, const LieGroupd & d) const = 0;
        
        // virtual void updateMarker(visualization_msgs::Marker & m, const LieGroupd & d) const = 0;
        virtual geometry_msgs::Pose toPose(DataType dt, double t, const LieGroupd & d) const = 0;

    public:

        SplineTestOptLieGroup(size_t nk) : BasicOptimisationProblem(),  spline(std::vector<LieGroupd>(nk),0.,10./(nk-1)) {}

        void initialize() {
            


            auto parametrization = new Sophus::Manifold<LieGroup_>;

            for (auto v : spline.parent_Ts_control_point()) {

                problem->AddParameterBlock(v.data(), LieGroupd::num_parameters, parametrization);
            }

            for (unsigned int i=0;i<Np;i++) {
                LieGroupd py = this->getPoint(i);
                points.push_back(py);
                Sophus::KnotsAndU ku = spline.knots_and_u(t[i]);

                ceres::CostFunction *cost;
                switch (ku.segment_case) {
                    case Sophus::SegmentCase::first:
                        cost = new ceres::AutoDiffCostFunction<SplineFunctor<LieGroup_>, LieGroupd::DoF,
                             LieGroupd::num_parameters, LieGroupd::num_parameters, LieGroupd::num_parameters>(
                                     new SplineFunctor(ku.segment_case,ku.u,py));
                        problem->AddResidualBlock(cost, nullptr, 
                                spline.parent_Ts_control_point()[ku.idx_0].data(),
                                spline.parent_Ts_control_point()[ku.idx_1].data(),
                                spline.parent_Ts_control_point()[ku.idx_2].data());
                        break;
                    case Sophus::SegmentCase::normal:
                        cost = new ceres::AutoDiffCostFunction<SplineFunctor<LieGroup_>, LieGroupd::DoF,
                             LieGroupd::num_parameters, LieGroupd::num_parameters, LieGroupd::num_parameters, LieGroupd::num_parameters>(
                                     new SplineFunctor(ku.segment_case,ku.u,py));
                        problem->AddResidualBlock(cost, nullptr, 
                                spline.parent_Ts_control_point()[ku.idx_prev].data(),
                                spline.parent_Ts_control_point()[ku.idx_0].data(),
                                spline.parent_Ts_control_point()[ku.idx_1].data(),
                                spline.parent_Ts_control_point()[ku.idx_2].data());
                        break;
                    case Sophus::SegmentCase::last:
                        cost = new ceres::AutoDiffCostFunction<SplineFunctor<LieGroup_>, LieGroupd::DoF,
                             LieGroupd::num_parameters, LieGroupd::num_parameters, LieGroupd::num_parameters>(
                                     new SplineFunctor(ku.segment_case,ku.u,py));
                        problem->AddResidualBlock(cost, nullptr, 
                                spline.parent_Ts_control_point()[ku.idx_prev].data(),
                                spline.parent_Ts_control_point()[ku.idx_0].data(),
                                spline.parent_Ts_control_point()[ku.idx_1].data());
                        break;
                }
            }
        }

        virtual ~SplineTestOptLieGroup() {
        }

        void print(const std::string & suffix) const {
            FILE * fp;
            fp = fopen(("input" + suffix).c_str(),"w");
            for (size_t i=0;i<Np;i++) {
                ::fprintf(fp,"%f ",t[i]);
                this->fprintf(fp,input,points[i]);
                ::fprintf(fp,"\n");
            }
            fclose(fp);
            fp = fopen(("knots" + suffix).c_str(),"w");
            for (size_t i=0;i<spline.parent_Ts_control_point().size();i++) {
                LieGroupd g = spline.parent_Ts_control_point()[i];
                double tknot = spline.t0() + i*spline.delta_t();
                ::fprintf(fp,"%f ",tknot);
                this->fprintf(fp,knots,g);
                ::fprintf(fp,"\n");
            }
            fclose(fp);
            fp = fopen(("spline" + suffix).c_str(),"w");
            for (double t=spline.t0();t<spline.tmax();t+=0.01) {
                LieGroupd ft = spline.parent_T_spline(t);
                ::fprintf(fp,"%f ",t);
                this->fprintf(fp,curve,ft);
                ::fprintf(fp,"\n");
            }
            fclose(fp);
        }

        void collectPublishableData(const std::string & suffix, ros::NodeHandle & nh,
                std::vector<PublisherWithDataPtr> & pub) const {
            ros::Publisher P = nh.advertise<geometry_msgs::PoseArray>("spline"+suffix+"_input",1);
            geometry_msgs::PoseArray pa;
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (size_t i=0;i<Np;i++) {
                geometry_msgs::Pose p = this->toPose(input,t[i],points[i]);
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

            P = nh.advertise<geometry_msgs::PoseArray>("spline"+suffix+"_knots",1);
            pa.poses.clear();
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (size_t i=0;i<spline.parent_Ts_control_point().size();i++) {
                LieGroupd g = spline.parent_Ts_control_point()[i];
                double tknot = spline.t0() + i*spline.delta_t();
                geometry_msgs::Pose p = this->toPose(knots,tknot,g);
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

            P = nh.advertise<geometry_msgs::PoseArray>("spline"+suffix+"_curve",1);
            pa.poses.clear();
            pa.header.frame_id = "world";
            pa.header.stamp = ros::Time::now();
            for (double t=spline.t0();t<spline.tmax();t+=0.1) {
                // Sophus::IndexAndU iu = spline.index_and_u(t);
                // Sophus::KnotsAndU ku = spline.knots_and_u(t);
                // printf("%f %d %f | %d %d %d %d %f (%d)\n", t, iu.i, iu.u, 
                //         ku.idx_prev, ku.idx_0, ku.idx_1, ku.idx_2, ku.u, int(ku.segment_case));
                LieGroupd g = spline.parent_T_spline(t);
                geometry_msgs::Pose p = this->toPose(curve,t,g);
                pa.poses.push_back(p);
            }
            pub.push_back(newPub(P,pa));

        }
};


class SplineTestOptQ : public SplineTestOptLieGroup<Sophus::SO3> {
    protected:
        using Parent = SplineTestOptLieGroup<Sophus::SO3>;
        using SO3d = Sophus::SO3d;
        using DataType = Parent::DataType;
        virtual SO3d getPoint(size_t i) const {
            return SO3d::exp(SO3d::Point(x[i],y[i],z[i]));
        }

        virtual void fprintf(FILE * fp, DataType dt, const SO3d & d) const {
            const double * q = d.data();
            auto aa = d.log();
            ::fprintf(fp,"%e %e %e %e %e %e %e",q[3],q[0],q[1],q[2],aa[0],aa[1],aa[2]);
        }
        
        virtual geometry_msgs::Pose toPose(DataType dt, double t, const SO3d & d) const {
            geometry_msgs::Pose p;
            const double * q = d.data();
            p.position.x=t; 
            switch (dt) {
                case DataType::input:
                    p.position.y=0; 
                    break;
                case DataType::knots:
                    p.position.y=0.5; 
                    break;
                case DataType::curve:
                    p.position.y=-0.5; 
                    break;
            }
            p.position.z=1;
            p.orientation.w=q[3]; p.orientation.x=q[0]; 
            p.orientation.y=q[1]; p.orientation.z=q[2];
            return p;
        }

    public:
        SplineTestOptQ() : SplineTestOptLieGroup<Sophus::SO3>(Nkq) {
        }

        ~SplineTestOptQ() {
        }

        void print() const {
            Parent::print("qs");
        }

        void collectPublishableData(ros::NodeHandle & nh,
                std::vector<PublisherWithDataPtr> & pub) const {
            Parent::collectPublishableData("q",nh,pub);
        }
};


#if 1
cerise::Pose lie2pose(const Sophus::SE3d & p) {
    const double * q = p.so3().data();
    auto t = p.translation();
    cerise::Pose P;
    P.T[0]=t[0]; P.T[1]=t[1]; P.T[2]=t[2];
    P.Q[0]=q[3]; P.Q[1]=q[0]; P.Q[2]=q[1]; P.Q[3]=q[2];
    return P;
}

Sophus::SE3d pose2lie(const cerise::Pose & p) {
    double aa[3];
    p.getAngleAxis(aa);
    Sophus::SE3d::Point Paa(aa[0],aa[1],aa[2]);
    Sophus::SE3d::Point Ptt(p.T[0],p.T[1],p.T[2]);
    return Sophus::SE3d(Sophus::SO3d::exp(Paa),Ptt);
}

void print(const Sophus::SE3d & ft, const std::string & prefix, const std::string & postfix) {
    auto p = ft.params();
    printf("%sQ %f %f %f | %f T %f %f %f%s",prefix.c_str(),
            p[0], p[1], p[2], p[3], p[4], p[5], p[6], postfix.c_str());
}


void print(const cerise::Pose & ft2, const std::string & prefix, const std::string & postfix) {
    printf("%sQ %f %f %f | %f T %f %f %f%s",prefix.c_str(),
            ft2.Q[1], ft2.Q[2], ft2.Q[3], ft2.Q[0], ft2.T[0], ft2.T[1], ft2.T[2], postfix.c_str());
}

void testSplineEval(double u, const Sophus::SE3d & k0, const Sophus::SE3d & k1, const Sophus::SE3d & k2, const Sophus::SE3d & k3) {
    printf("\n\n==========================\n");
    printf("SplineEval\n");
    Sophus::SE3d sK[4] = {k0,k1,k2,k3};
    cerise::Pose cK[4] = {lie2pose(k0),lie2pose(k1),lie2pose(k2),lie2pose(k3)};
    for (size_t i=0;i<4;i++) {
        print(sK[i],"SE3 ","  ");
        print(cK[i],"POSE ","\n");
    }

    Sophus::SE3d dsK[3] = {sK[0].inverse()*sK[1],sK[1].inverse()*sK[2],sK[2].inverse()*sK[3]};
    cerise::Pose dcK[3] = {cK[0].inverse()*cK[1],cK[1].inverse()*cK[2],cK[2].inverse()*cK[3]};
    for (size_t i=0;i<3;i++) {
        print(dsK[i],"dSE3 ","  ");
        print(dcK[i],"dPOSE ","\n");
    }

    Eigen::Vector4d U; U << 1, u, u*u, u*u*u;
    Eigen::Vector4d cB = cerise::SplineBaseMatrixC * U;

    Sophus::Vector3<double> sB = Sophus::SplineBasisFunction<double>::B(u);
    std::cout << "cB " << cB.transpose() << std::endl;
    std::cout << "sB " << sB.transpose() << std::endl;

    Sophus::SE3d edsK[3] = {
        Sophus::SE3d::exp(sB[0] * dsK[0].log()),
        Sophus::SE3d::exp(sB[1] * dsK[1].log()),
        Sophus::SE3d::exp(sB[2] * dsK[2].log())
    };

    cerise::DataDescriptorPose<double> D;
    cerise::DataDescriptorPose<double>::LogVarType ldcK[3];
    D.log(dcK[0], ldcK[0]);
    D.log(dcK[1], ldcK[1]);
    D.log(dcK[2], ldcK[2]);
    cerise::Pose edcK[3];
    D.exp(ldcK[0]*cB[1],D.writable(edcK[0]));
    D.exp(ldcK[1]*cB[2],D.writable(edcK[1]));
    D.exp(ldcK[2]*cB[3],D.writable(edcK[2]));
    for (size_t i=0;i<3;i++) {
        std::cout << "lSE3 " << dsK[i].log().transpose() << " lPOSE " << ldcK[i].transpose() << std::endl;
    }
    for (size_t i=0;i<3;i++) {
        print(edsK[i],"eSE3 ","  ");
        print(edcK[i],"ePOSE ","\n");
    }

    Sophus::SE3d sres = sK[0] * edsK[0] * edsK[1] * edsK[2];
    cerise::Pose cres = cK[0] * edcK[0] * edcK[1] * edcK[2];
    print(sres,"RSE3 ","  ");
    print(cres,"RPOSE ","\n");
}

class SplineTestOptP : public SplineTestOptLieGroup<Sophus::SE3> {
    protected:
        struct NormalizeCB : public ceres::IterationCallback {
            SplineTestOptP * that; 
            NormalizeCB(SplineTestOptP * that) : that(that) {}

            ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
                that->normalize();
                return ceres::SOLVER_CONTINUE;
            }
        };

        virtual void updateOptions(ceres::Solver::Options * options) {
            options->update_state_every_iteration = true;
            options->callbacks.push_back(new NormalizeCB(this));
        }


    protected:
        using Parent = SplineTestOptLieGroup<Sophus::SE3>;
        using SE3d = Sophus::SE3d;
        using Point = SE3d::Point;
        virtual SE3d getPoint(size_t i) const {
            Point P(x[i],y[i],z[i]);
            SE3d res(Sophus::SO3d::exp(P),P);
#if 0
            double aa[3] = {x[i],y[i],z[i]};
            cerise::Pose ft2;
            ft2.setFromAngleAxis(aa,aa);
            SE3d res1 = pose2lie(ft2);
            auto p = res.params();
            auto p1 = res1.params();
            printf("input %d\n",int(i));
            printf("Q %f %f %f %f %f %f %f\n",
                    p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
            printf("Q1 %f %f %f %f %f %f %f\n",
                    p1[0], p1[1], p1[2], p1[3], p1[4], p1[5], p1[6]);
            printf("Q2 %f %f %f %f %f %f %f\n",
                    ft2.Q[1], ft2.Q[2], ft2.Q[3], ft2.Q[0], ft2.T[0], ft2.T[1], ft2.T[2]);
#endif
            return res;
        }

        virtual void fprintf(FILE * fp, DataType dt, const SE3d & d) const {
            auto t = d.translation();
            const double * q = d.so3().data();
            auto aa = d.so3().log();
            ::fprintf(fp,"%e %e %e %e %e %e %e %e %e %e",t[0],t[1],t[2],
                    q[3],q[0],q[1],q[2],aa[0],aa[1],aa[2]);
        }
        
        virtual geometry_msgs::Pose toPose(DataType dt, double t, const SE3d & d) const {
            geometry_msgs::Pose p;
            auto T = d.translation();
            const double * q = d.so3().data();
            p.position.x=T[0]; 
            p.position.y=T[1]; 
            p.position.z=T[2];
            p.orientation.w=q[3]; p.orientation.x=q[0]; 
            p.orientation.y=q[1]; p.orientation.z=q[2];
            return p;
        }


    public:
        SplineTestOptP() : SplineTestOptLieGroup<Sophus::SE3>(Nkp) {
        }

        ~SplineTestOptP() {
        }

        void print() const {
            Parent::print("ps");
        }

        void collectPublishableData(ros::NodeHandle & nh,
                std::vector<PublisherWithDataPtr> & pub) const {
            Parent::collectPublishableData("p",nh,pub);
        }

        void normalize() {
            for (size_t i=0;i<spline.parent_Ts_control_point().size();i++) {
                spline.parent_Ts_control_point()[i].normalize();
            }
        }

        void compareTest() const { 
            cerise::UniformSpline<cerise::TRefPoseUniformSpline<double>> spline2(0.,10.,Nkp);
            assert(spline2.warper.n_knots == spline.parent_Ts_control_point().size());
            for (size_t i=0;i<spline2.warper.n_knots;i++) {
                const SE3d & ft = spline.parent_Ts_control_point()[i];
                auto p = ft.params();
                spline2.knots[i] = lie2pose(ft);
                const cerise::Pose & ft2 = spline2.knots[i];
                printf("Knot %d\n",int(i));
                printf("Q1 %f %f %f %f %f %f %f\n",
                    p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
                printf("Q2 %f %f %f %f %f %f %f\n",
                    ft2.Q[1], ft2.Q[2], ft2.Q[3], ft2.Q[0], ft2.T[0], ft2.T[1], ft2.T[2]);
            }
            for (double t=spline.t0();t<spline.tmax();t+=0.1) {

                SE3d ft = spline.parent_T_spline(t);
                ft.normalize();
                auto p = ft.params();
                Sophus::KnotsAndU ku = spline.knots_and_u(t);
                Sophus::BasisSplineSegment<SE3d> segment(ku.segment_case,
                        spline.parent_Ts_control_point()[ku.idx_prev].data(),
                        spline.parent_Ts_control_point()[ku.idx_0].data(),
                        spline.parent_Ts_control_point()[ku.idx_1].data(),
                        spline.parent_Ts_control_point()[ku.idx_2].data());
                SE3d ft1 = segment.parent_T_spline(ku.u);
                ft1.normalize();
                auto p1 = ft1.params();

                cerise::TRefPoseUniformSpline<double> segment2(
                        lie2pose(spline.parent_Ts_control_point()[ku.idx_prev]),
                        lie2pose(spline.parent_Ts_control_point()[ku.idx_0]),
                        lie2pose(spline.parent_Ts_control_point()[ku.idx_1]),
                        lie2pose(spline.parent_Ts_control_point()[ku.idx_2]));
                cerise::Pose ft2; segment2.cum_evaluate(ku.u,ft2);
                printf("Q %f %f %f %f %f %f %f\n",
                    p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
                printf("Q1 %f %f %f %f %f %f %f\n",
                    p1[0], p1[1], p1[2], p1[3], p1[4], p1[5], p1[6]);
                printf("Q2 %f %f %f %f %f %f %f\n",
                    ft2.Q[1], ft2.Q[2], ft2.Q[3], ft2.Q[0], ft2.T[0], ft2.T[1], ft2.T[2]);

                testSplineEval(ku.u, 
                        spline.parent_Ts_control_point()[ku.idx_prev],
                        spline.parent_Ts_control_point()[ku.idx_0],
                        spline.parent_Ts_control_point()[ku.idx_1],
                        spline.parent_Ts_control_point()[ku.idx_2]);
            }
            printf("\n\n======================\n");
            for (double t=spline.t0();t<spline.tmax();t+=0.1) {

                Sophus::KnotsAndU ku = spline.knots_and_u(t);
                SE3d ft = spline.parent_T_spline(t);
                ft.normalize();
                cerise::TRefPoseUniformSpline<double> segment2(
                        lie2pose(spline.parent_Ts_control_point()[ku.idx_prev]),
                        lie2pose(spline.parent_Ts_control_point()[ku.idx_0]),
                        lie2pose(spline.parent_Ts_control_point()[ku.idx_1]),
                        lie2pose(spline.parent_Ts_control_point()[ku.idx_2]));
                cerise::Pose ft2; segment2.cum_evaluate(ku.u,ft2);
                printf("%f %f %f\n",t,ft.translation()[0],ft2.T[0]);
            }
        }

};

#endif

int main(int argc, char * argv[]) {
    
    ros::init(argc,argv,"test_ceres_spline_sophus");
    ros::NodeHandle nh("~");
    std::vector<PublisherWithDataPtr> pubdata;

#if 1
    printf("\n\n SplineTestOptQ \n\n");
    SplineTestOptQ tq;
    tq.initialize();
    tq.optimise();
    tq.print();
    tq.collectPublishableData(nh,pubdata);
#endif


#if 1
    printf("\n\n SplineTestOptP \n\n");
    SplineTestOptP tp;
    tp.initialize();
    tp.optimise();
    tp.normalize();
    tp.print();
    // tp.compareTest();
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
