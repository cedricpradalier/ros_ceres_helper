#ifndef CERES_SPLINES_H
#define CERES_SPLINES_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <vector>


namespace cerise{ 

    const Eigen::Matrix4d SplineBaseMatrixM = (Eigen::Matrix4d() << 
            1.0 / 6.0 , 4.0 / 6.0 , 1.0 / 6.0  , 0.0 / 6.0  ,
            -3.0 / 6.0 , 0.0 / 6.0 , 3.0 / 6.0 ,  0.0 / 6.0 ,
            3.0 / 6.0 , -6.0 / 6.0 , 3.0 / 6.0  , 0.0 / 6.0 ,
            -1.0 / 6.0 , 3.0 / 6.0  , -3.0 / 6.0 ,  1.0 / 6.0).finished();

    const Eigen::Matrix4d SplineBaseMatrixC = (Eigen::Matrix4d() << 
            6.0 / 6.0 , 0.0       , 0.0        , 0.0        ,
            5.0 / 6.0 , 3.0 / 6.0 , -3.0 / 6.0 , 1.0 / 6.0  ,
            1.0 / 6.0 , 3.0 / 6.0 , 3.0 / 6.0  , -2.0 / 6.0 ,
            0.0       , 0.0       , 0.0        ,  1.0 / 6.0).finished();

    template <typename T>
        struct TSplineNamespace {
            const Eigen::Matrix4d & M;
            const Eigen::Matrix4d & C;
            TSplineNamespace() : M(SplineBaseMatrixM), C(SplineBaseMatrixC) {}

            static Eigen::Matrix<T, 4, 1> spline_B(T u) {
                Eigen::Matrix<T, 4, 1> U(T(1.0), u, u * u, u * u * u);
                return (U.transpose() * M.cast<T>()).transpose();
            }

            static Eigen::Matrix<T, 4, 1> spline_Bprim(T u, double dt) {
                Eigen::Matrix<T, 4, 1> U(T(0.0), T(1.0), T(2) * u, T(3) * u * u);
                return (U.transpose() * M.cast<T>()).transpose() / T(dt);
            }

            static Eigen::Matrix<T, 4, 1> spline_Bbis(T u, double dt) {
                double dt2 = dt * dt;
                Eigen::Matrix<T, 4, 1> U(T(0.0), T(0.0), T(2 / dt2), T(6) * u / T(dt2));
                return (U.transpose() * M.cast<T>()).transpose();
            }


            static Eigen::Matrix<T, 4, 1> cum_spline_B(T u) {
                Eigen::Matrix<T, 4, 1> U(T(1.0), u, u * u, u * u * u);
                return C.cast<T>() * U;
            }

            static Eigen::Matrix<T, 4, 1> cum_spline_Bprim(T u, double dt) {
                Eigen::Matrix<T, 4, 1> U(T(0.0), T(1.0), T(2) * u, T(3) * u * u);
                return C.cast<T>() * U / T(dt);
            }

            static Eigen::Matrix<T, 4, 1> cum_spline_Bbis(T u, double dt) {
                double dt2 = dt * dt;
                Eigen::Matrix<T, 4, 1> U(T(0.0), T(0.0), T(2 / dt2), T(6) * u / T(dt2));
                return C.cast<T>() * U;
            }
        };




    template <typename DT>
        struct TRef1DUniformSpline : public TSplineNamespace<DT> {
            const DT & K0;
            const DT & K1;
            const DT & K2;
            const DT & K3;
            TRef1DUniformSpline(const DT & k0, const DT & k1, const DT & k2, const DT & k3) 
                : K0(k0), K1(k1), K2(k2), K3(k3)
            { }

            DT evaluate(DT u) const {
                // int i = floor(t);
                // i = std::min<int>(std::max<int>(i,1),knot.size()-3);
                // IT u = t - IT(i);
                Eigen::Matrix<DT, 4, 1> Mu = spline_B(u);
                Eigen::Matrix<DT, 1, 4> V;
                V << K0, K1, K2, K3
                    DT out = (V * Mu)(0);
                return out;
            }

            DT cum_evaluate(DT u) const {
                Eigen::Matrix<DT, 4, 1> Mu = cum_spline_B(u);
                Eigen::Matrix<DT, 1, 4> V;
                V << DT(0), K1-K0, K2-K1, K3-K2;
                DT out = K0 + (V * Mu)(0);
                return out;
            }
        };

    template <typename DT, int dim>
        struct TRefEigenUniformSpline  : public TSplineNamespace<DT> {
            typedef Eigen::Matrix<DT,dim,1> EigenType;
            const EigenType & K0;
            const EigenType & K1;
            const EigenType & K2;
            const EigenType & K3;
            TRef1DUniformSpline(const EigenType & k0, const EigenType & k1, const EigenType & k2, const EigenType & k3) 
                : K0(k0), K1(k1), K2(k2), K3(k3)
            { }

            EigenType evaluate(DT u) const {
                Eigen::Matrix<DT, 4, 1> Mu = spline_B(u);
                Eigen::Matrix<DT, dim, 4> V;
                V.col(0) = K0;
                V.col(1) = K1;
                V.col(2) = K2;
                V.col(3) = K3;
                EigenType out = (V * Mu)(0);
                return out;
            }

            EigenType cum_evaluate(DT u) const {
                Eigen::Matrix<DT, 4, 1> Mu = cum_spline_B(u);
                Eigen::Matrix<DT, dim, 3> V;
                V.col(0) = K1-K0;
                V.col(1) = K2-K1;
                V.col(2) = K3-K2;
                EigenType out = K0 + (V * Mu.block<3,1>(0,1))(0);
                return out;
            }
        };

    // TODO: Time Warper
    // TODO: Multi-knot Warper
    // TODO: Quaternion Spline
}


#endif // CERES_SPLINES_H
