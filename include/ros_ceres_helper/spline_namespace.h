#ifndef SPLINE_NAMESPACE_H
#define SPLINE_NAMESPACE_H

#include <ros_ceres_helper/data_descriptor.h>
#include <Eigen/Dense>


namespace cerise{ 

    template <class Descriptor>
        class TSplineNamespace {
            public:
            typedef Descriptor DescriptorType;
            typedef typename Descriptor::DataType T;
            protected:
            const Eigen::Matrix4d & M;
            const Eigen::Matrix4d & C;
            Descriptor D;
            public:

            TSplineNamespace(const Descriptor & D = Descriptor()) : M(SplineBaseMatrixM), C(SplineBaseMatrixC), D(D) { }

            typename Descriptor::VarType create() const { return D.create(); }

            void destroy(typename Descriptor::RefType v) const {D.destroy(v);}

            void set(typename Descriptor::ConstRefType v, typename Descriptor::WritableType v3) const { D.set(v,v3); }

            typename Descriptor::WritableType writable(typename Descriptor::RefType v) const { return D.writable(v); }

            void exp(typename Descriptor::ConstRefLogType x, typename Descriptor::WritableType y) const { D.exp(x,y);}

            void log(typename Descriptor::ConstRefType x, typename Descriptor::LogWritableType y) const { D.log(x,y);}
            
            void add(typename Descriptor::ConstRefType v1, typename Descriptor::ConstRefType v2, typename Descriptor::WritableType v3) const { D.add(v1,v2,v3); }

            void sub(typename Descriptor::ConstRefType v1, typename Descriptor::ConstRefType v2, typename Descriptor::WritableType v3) const { D.sub(v1,v2,v3); }

            static Eigen::Matrix<T, 4, 1> spline_B(T u) {
                Eigen::Matrix<T, 4, 1> U(T(1.0), u, u * u, u * u * u);
                return (U.transpose() * SplineBaseMatrixM.cast<T>()).transpose();
            }

            static Eigen::Matrix<T, 4, 1> spline_Bprim(T u, double dt) {
                Eigen::Matrix<T, 4, 1> U(T(0.0), T(1.0), T(2) * u, T(3) * u * u);
                return (U.transpose() * SplineBaseMatrixM.cast<T>()).transpose() / T(dt);
            }

            static Eigen::Matrix<T, 4, 1> spline_Bbis(T u, double dt) {
                double dt2 = dt * dt;
                Eigen::Matrix<T, 4, 1> U(T(0.0), T(0.0), T(2 / dt2), T(6) * u / T(dt2));
                return (U.transpose() * SplineBaseMatrixM.cast<T>()).transpose();
            }


            static Eigen::Matrix<T, 4, 1> cum_spline_B(T u) {
                Eigen::Matrix<T, 4, 1> U(T(1.0), u, u * u, u * u * u);
                return SplineBaseMatrixC.cast<T>() * U;
            }

            static Eigen::Matrix<T, 4, 1> cum_spline_Bprim(T u, double dt) {
                Eigen::Matrix<T, 4, 1> U(T(0.0), T(1.0), T(2) * u, T(3) * u * u);
                return SplineBaseMatrixC.cast<T>() * U / T(dt);
            }

            static Eigen::Matrix<T, 4, 1> cum_spline_Bbis(T u, double dt) {
                double dt2 = dt * dt;
                Eigen::Matrix<T, 4, 1> U(T(0.0), T(0.0), T(2 / dt2), T(6) * u / T(dt2));
                return SplineBaseMatrixC.cast<T>() * U;
            }
        };


}


#endif // SPLINE_NAMESPACE_H
