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


    struct TimeWarper {
        double t0;
        double delta;
        size_t n_knots;
        TimeWarper(double t0, double delta, size_t n_knots) :
            t0(t0), delta(delta), n_knots(n_knots) {}

        template <typename IT> 
            std::pair<size_t, IT> operator()(IT t) const {
                int i = floor(t);
                i = std::min<int>(std::max<int>(i,1),n_knots-3);
                IT u = t - IT(i);
                return std::pair<size_t, IT>(i,u);
            }

        template <typename IT> 
            size_t iknot(IT t) const {
                int i = floor(t);
                i = std::min<int>(std::max<int>(i,1),n_knots-3);
                return i;
            }

        double knot(size_t i) const {
            return t0 + std::min(i,n_knots-1)*delta;
        }

        double min() const {
            return t0;
        }
        double max() const {
            return t0 + n_knots*delta;
        }
    };



    template <typename DT>
        struct TRef1DUniformSpline : public TSplineNamespace<DT> {
            typedef DT DataType;
            typedef DT StorageType;
            typedef const DT & ConstStorageType;

            ConstStorageType K0;
            ConstStorageType K1;
            ConstStorageType K2;
            ConstStorageType K3;
            TRef1DUniformSpline(ConstStorageType k0, ConstStorageType k1, ConstStorageType k2, ConstStorageType k3) 
                : K0(k0), K1(k1), K2(k2), K3(k3)
            { }

            bool evaluate(DT u, StorageType & fu) const {
                // int i = floor(t);
                // i = std::min<int>(std::max<int>(i,1),knot.size()-3);
                // IT u = t - IT(i);
                Eigen::Matrix<DT, 4, 1> Mu = TSplineNamespace<DT>::spline_B(u);
                Eigen::Matrix<DT, 1, 4> V;
                V << K0, K1, K2, K3;
                fu = (V * Mu)(0);
                // std::cout << "Evaluate " << u << " " << V << " " << Mu.transpose() << " " << fu << std::endl;
                return true;
            }

            bool cum_evaluate(DT u, StorageType & fu) const {
                Eigen::Matrix<DT, 4, 1> Mu = TSplineNamespace<DT>::cum_spline_B(u);
                Eigen::Matrix<DT, 1, 4> V;
                V << DT(0), K1-K0, K2-K1, K3-K2;
                fu = K0 + (V * Mu)(0);
                return true;
            }
        };

    template <typename DT, int dim>
        struct TRefPtrUniformSpline  : public TSplineNamespace<DT> {
            typedef DT DataType;
            typedef DT* StorageType;
            typedef const DT* ConstStorageType;
            ConstStorageType K0;
            ConstStorageType K1;
            ConstStorageType K2;
            ConstStorageType K3;
            TRefPtrUniformSpline(ConstStorageType k0, ConstStorageType k1, ConstStorageType k2, ConstStorageType k3) 
                : K0(k0), K1(k1), K2(k2), K3(k3)
            { }

            bool evaluate(DT u, StorageType fu/*, DT *dfdu, DT *d2fdu2*/) const {
                Eigen::Matrix<DT, 4, 1> Mu = TSplineNamespace<DT>::spline_B(u);
                Eigen::Matrix<DT, dim, 4> V;
                for (int j=0;j<dim;j++) {
                    V(j,0) = K0[j];
                    V(j,1) = K1[j];
                    V(j,2) = K2[j];
                    V(j,3) = K3[j];
                }
                Eigen::Matrix<DT, dim, 1>  res = V * Mu;
                for (int j=0;j<dim;j++) {
                    fu[j]=res(j,0);
                }
                return true;
            }

            bool cum_evaluate(DT u, StorageType fu) const {
                Eigen::Matrix<DT, 4, 1> Mu = TSplineNamespace<DT>::cum_spline_B(u);
                Eigen::Matrix<DT, dim, 3> V;
                Eigen::Matrix<DT, dim, 1> res;
                for (int j=0;j<dim;j++) {
                    V(j,0) = K1[j]-K0[j];
                    V(j,1) = K2[j]-K1[j];
                    V(j,2) = K3[j]-K2[j];
                    res(j,0) = K0[j];
                }
                res += (V * Mu.block<3,1>(0,1));
                for (int j=0;j<dim;j++) {
                    fu[j]=res(j,0);
                }
                return true;
            }
        };

    template <typename DT, int dim>
        struct TRefEigenUniformSpline  : public TSplineNamespace<DT> {
            typedef DT DataType;
            typedef Eigen::Matrix<DT,dim,1> StorageType;
            typedef const Eigen::Matrix<DT,dim,1> & ConstStorageType;
            ConstStorageType K0;
            ConstStorageType K1;
            ConstStorageType K2;
            ConstStorageType K3;
            TRefEigenUniformSpline(ConstStorageType k0, ConstStorageType k1, ConstStorageType k2, ConstStorageType k3) 
                : K0(k0), K1(k1), K2(k2), K3(k3)
            { }

            bool evaluate(DT u, StorageType & fu) const {
                Eigen::Matrix<DT, 4, 1> Mu = TSplineNamespace<DT>::spline_B(u);
                Eigen::Matrix<DT, dim, 4> V;
                V.col(0) = K0;
                V.col(1) = K1;
                V.col(2) = K2;
                V.col(3) = K3;
                fu = V * Mu;
                return true;
            }

            bool cum_evaluate(DT u , StorageType & fu) const {
                Eigen::Matrix<DT, 4, 1> Mu = TSplineNamespace<DT>::cum_spline_B(u);
                Eigen::Matrix<DT, dim, 3> V;
                V.col(0) = K1-K0;
                V.col(1) = K2-K1;
                V.col(2) = K3-K2;
                fu = K0 + (V * Mu.block<3,1>(0,1));
                return true;
            }
        };

    template <class SplineType> 
        struct UniformSpline {
            std::vector<typename SplineType::StorageType> knots;
            TimeWarper warper;

            UniformSpline(double tmin, double tmax, size_t n_knots) : knots(n_knots), warper(tmin, (tmax-tmin)/(n_knots-1) , n_knots) {}
            UniformSpline(double tmin, size_t n_knots, double delta) : knots(n_knots), warper(tmin, delta , n_knots) {}
            UniformSpline(TimeWarper warper) : knots(warper.n_knots), warper(warper) {}
            
            template <class IT>
                void import(IT begin) {
                    std::copy(begin,begin+knots.size(),knots.begin());
                }

            bool evaluate(typename SplineType::DataType t, typename SplineType::StorageType && fu) const {
                std::pair<size_t,typename SplineType::DataType> iu = warper(t);
                // printf("S Evaluate t=%f, i=%d, u=%f\n",t,int(iu.first),iu.second);
                SplineType s(knots[iu.first-1],knots[iu.first],knots[iu.first+1],knots[iu.first+2]);
                return s.evaluate(iu.second, fu);
            }

            bool cum_evaluate(typename SplineType::DataType t, typename SplineType::StorageType && fu) const {
                std::pair<size_t,typename SplineType::DataType> iu = warper(t);
                SplineType s(knots[iu.first-1],knots[iu.first],knots[iu.first+1],knots[iu.first+2]);
                return s.cum_evaluate(iu.second, fu);
            }

            bool evaluate(typename SplineType::DataType t, typename SplineType::StorageType & fu) const {
                std::pair<size_t,typename SplineType::DataType> iu = warper(t);
                // printf("S Evaluate t=%f, i=%d, u=%f\n",t,int(iu.first),iu.second);
                SplineType s(knots[iu.first-1],knots[iu.first],knots[iu.first+1],knots[iu.first+2]);
                return s.evaluate(iu.second, fu);
            }

            bool cum_evaluate(typename SplineType::DataType t, typename SplineType::StorageType & fu) const {
                std::pair<size_t,typename SplineType::DataType> iu = warper(t);
                SplineType s(knots[iu.first-1],knots[iu.first],knots[iu.first+1],knots[iu.first+2]);
                return s.cum_evaluate(iu.second, fu);
            }

        };

    typedef UniformSpline<TRef1DUniformSpline<double>> GenericSpline1D;

    template <int dim> 
        struct SplineError {
            double u;
            double y[dim];

            SplineError(double u, const double *yin) : u(u) {
                std::copy(yin+0,yin+dim,y+0);
            }


            template <typename T>
                bool operator()(const T *const k1, const T* const k2, const T* const k3, const T *const k4,
                        T* residuals) const {
                    TRefPtrUniformSpline<T,dim> s(k1,k2,k3,k4);
                    T pred[dim];
                    s.evaluate(T(u),pred);
                    for (int i=0;i<dim;i++) {
                        residuals[i] = pred[i] - y[i];
                    }
                    return true;
                }
        };



    // TODO: Quaternion Spline
}


#endif // CERES_SPLINES_H
