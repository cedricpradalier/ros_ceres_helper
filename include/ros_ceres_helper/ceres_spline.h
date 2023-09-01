#ifndef CERES_SPLINES_H
#define CERES_SPLINES_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <vector>


namespace cerise{ 


    const Eigen::Matrix4d M = (Eigen::Matrix4d() << 
            1.0 / 6.0 , 4.0 / 6.0 , 1.0 / 6.0  , 0.0 / 6.0  ,
            -3.0 / 6.0 , 0.0 / 6.0 , 3.0 / 6.0 ,  0.0 / 6.0 ,
            3.0 / 6.0 , -6.0 / 6.0 , 3.0 / 6.0  , 0.0 / 6.0 ,
            -1.0 / 6.0 , 3.0 / 6.0  , -3.0 / 6.0 ,  1.0 / 6.0).finished();

    const Eigen::Matrix4d C = (Eigen::Matrix4d() << 
            6.0 / 6.0 , 0.0       , 0.0        , 0.0        ,
            5.0 / 6.0 , 3.0 / 6.0 , -3.0 / 6.0 , 1.0 / 6.0  ,
            1.0 / 6.0 , 3.0 / 6.0 , 3.0 / 6.0  , -2.0 / 6.0 ,
            0.0       , 0.0       , 0.0        ,  1.0 / 6.0).finished();

    template<typename T>
        Eigen::Matrix<T, 4, 1> spline_B(T u) {
            Eigen::Matrix<T, 4, 1> U(T(1.0), u, u * u, u * u * u);
            return (U.transpose() * M.cast<T>()).transpose();
        }

    template<typename T>
        Eigen::Matrix<T, 4, 1> spline_Bprim(T u, double dt) {
            Eigen::Matrix<T, 4, 1> U(T(0.0), T(1.0), T(2) * u, T(3) * u * u);
            return (U.transpose() * M.cast<T>()).transpose() / T(dt);
        }

    template<typename T>
        Eigen::Matrix<T, 4, 1> spline_Bbis(T u, double dt) {
            double dt2 = dt * dt;
            Eigen::Matrix<T, 4, 1> U(T(0.0), T(0.0), T(2 / dt2), T(6) * u / T(dt2));
            return (U.transpose() * M.cast<T>()).transpose();
        }


    template<typename T>
        Eigen::Matrix<T, 4, 1> cum_spline_B(T u) {
            Eigen::Matrix<T, 4, 1> U(T(1.0), u, u * u, u * u * u);
            return C.cast<T>() * U;
        }

    template<typename T>
        Eigen::Matrix<T, 4, 1> cum_spline_Bprim(T u, double dt) {
            Eigen::Matrix<T, 4, 1> U(T(0.0), T(1.0), T(2) * u, T(3) * u * u);
            return C.cast<T>() * U / T(dt);
        }

    template<typename T>
        Eigen::Matrix<T, 4, 1> cum_spline_Bbis(T u, double dt) {
            double dt2 = dt * dt;
            Eigen::Matrix<T, 4, 1> U(T(0.0), T(0.0), T(2 / dt2), T(6) * u / T(dt2));
            return C.cast<T>() * U;
        }




    template <typename DT>
    struct TRefUniformSpline {
        std::vector<DT> knot;
        TRefUniformSpline(unsigned int n_knots) {
            knot.resize(n_knots,DT(0)); 
        }

        template <typename IT> 
            TRefUniformSpline(const IT & begin, const IT & end) {
                for (IT it=begin; it!=end; it++) {
                    knot.push_back(DT(*it));
                }
            }

        template <typename IT> 
            void setKnot(size_t i, const IT & v) {
                assert(i<knot.size());
                knot[i] = DT(v);
            }

        DT getKnot(size_t i) const { 
            assert(i<knot.size());
            return knot[i];
        }
        
        template <typename IT>
            DT evaluate(IT t) const {
                int i = floor(t);
                i = std::min<int>(std::max<int>(i,1),knot.size()-3);
                IT u = t - IT(i);
                Eigen::Matrix<DT, 4, 1> Mu = spline_B(DT(u));
                Eigen::Matrix<DT, 1, 4> V;
                V << knot[i-1], knot[i], knot[i+1], knot[i+2];
                DT out = (V * Mu)(0);
                return out;
            }

        template <typename IT>
            DT cum_evaluate(IT t) const {
                int i = floor(t);
                i = std::min<int>(std::max<int>(i,1),knot.size()-3);
                IT u = t - IT(i);
                Eigen::Matrix<DT, 4, 1> Mu = cum_spline_B(DT(u));
                Eigen::Matrix<DT, 1, 4> V;
                V << DT(0), knot[i]-knot[i-1], knot[i+1]-knot[i], knot[i+2]-knot[i+1];
                DT out = knot[i-1] + (V * Mu)(0);
                return out;
            }
    };
}


#endif // CERES_SPLINES_H
