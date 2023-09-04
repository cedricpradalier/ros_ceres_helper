#ifndef CERES_SPLINES_H
#define CERES_SPLINES_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
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

    template <typename DT, int dim, int localdim>
        struct DataDescriptor {
            static const unsigned int dimension = dim;
            static const unsigned int local_dimension = localdim;
            typedef DT DataType;

            typedef Eigen::Matrix<DT,localdim,1> LogVarType;
            typedef const Eigen::Matrix<DT,localdim,1> & ConstRefLogType;
            typedef Eigen::Matrix<DT,localdim,1>& LogWritableType;

            DataDescriptor() {}
            virtual ~DataDescriptor() {}

           
            /*
             * Must provide the following:
             *
            typedef DT DataType;
            typedef DT VarType;
            typedef DT & RefType;
            typedef const DT & ConstRefType;
            typedef DT * WritableType;

            virtual VarType create() const = 0;

            virtual destroy(RefType v) const = 0;

            virtual void set(ConstRefType x, WritableType y) const = 0;

            virtual WritableType writable(RefType x) const = 0;

            virtual void exp(ConstRefLogType x, WritableType y) const = 0;

            virtual void log(ConstRefType x, LogWritableType y) const = 0;
            
            virtual void add(ConstRefType v1, ConstRefType v2, WritableType v3) const = 0;

            virtual void sub(ConstRefType v1, ConstRefType v2, WritableType v3) const = 0;
            */

        };

    template <typename DT>
        struct DataDescriptor1D : public DataDescriptor<DT, 1, 1> {
            typedef DT DataType;
            typedef DT VarType;
            typedef DT & RefType;
            typedef const DT & ConstRefType;
            typedef DT * WritableType;

            typedef typename DataDescriptor<DT,1,1>::LogVarType LogVarType;
            typedef typename DataDescriptor<DT,1,1>::ConstRefLogType ConstRefLogType;
            typedef typename DataDescriptor<DT,1,1>::LogWritableType LogWritableType;

            virtual VarType create() const {
                return DT(0);
            }

            virtual void destroy(RefType v) const {}

            virtual void set(ConstRefType x, WritableType y) const {
                *y = x;
            }

            virtual WritableType writable(RefType x) const {
                return &x;
            }

            virtual void exp(ConstRefLogType x, WritableType y) const  {
                *y = x(0);
            }

            virtual void log(ConstRefType x, LogWritableType y) const {
                y(0) = x;
            }
            
            virtual void add(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                *v3 = v1 + v2;
            }

            virtual void sub(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                *v3 = v1 - v2;
            }
        };

    template <typename DT, int dim>
        struct DataDescriptorPtr : public DataDescriptor<DT, dim, dim> {
            typedef DT DataType;
            typedef DT* VarType;
            typedef DT* RefType;
            typedef const DT * ConstRefType;
            typedef DT * WritableType;

            typedef typename DataDescriptor<DT,dim,dim>::LogVarType LogVarType;
            typedef typename DataDescriptor<DT,dim,dim>::ConstRefLogType ConstRefLogType;
            typedef typename DataDescriptor<DT,dim,dim>::LogWritableType LogWritableType;

            virtual VarType create() const {
                return new DT[dim];
            }

            virtual void destroy(RefType v) const {
                delete [] v;
            }


            virtual void set(ConstRefType x, WritableType y) const {
                for (int i=0;i<dim;i++) {
                    y[i] = x[i];
                }
            }

            virtual WritableType writable(RefType x) const {
                return x;
            }


            virtual void exp(ConstRefLogType x, WritableType y) const  {
                for (int i=0;i<dim;i++) {
                    y[i] = x(i,0);
                }
            }

            virtual void log(ConstRefType x, LogWritableType y) const {
                for (int i=0;i<dim;i++) {
                    y(i,0) = x[i];
                }
            }
            
            virtual void add(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                for (int i=0;i<dim;i++) {
                    v3[i] = v1[i] + v2[i];
                }
            }

            virtual void sub(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                for (int i=0;i<dim;i++) {
                    v3[i] = v1[i] - v2[i];
                }
            }
        };

    template <typename DT>
        struct DataDescriptorQuaternion : public DataDescriptor<DT, 4, 3> {
            typedef DT DataType;
            typedef DT* VarType;
            typedef DT* RefType;
            typedef const DT * ConstRefType;
            typedef DT * WritableType;

            typedef typename DataDescriptor<DT,4,3>::LogVarType LogVarType;
            typedef typename DataDescriptor<DT,4,3>::ConstRefLogType ConstRefLogType;
            typedef typename DataDescriptor<DT,4,3>::LogWritableType LogWritableType;

            virtual VarType create() const {
                return new DT[4];
            }

            virtual void destroy(RefType v) const {
                delete [] v;
            }


            virtual void set(ConstRefType x, WritableType y) const {
                for (int i=0;i<4;i++) {
                    y[i] = x[i];
                }
            }

            virtual WritableType writable(RefType x) const {
                return x;
            }


            virtual void exp(ConstRefLogType x, WritableType y) const  {
                DT aa[3] = {x(0,0), x(1,0), x(2,0)};
                ceres::AngleAxisToQuaternion(aa,y);
            }

            virtual void log(ConstRefType x, LogWritableType y) const {
                DT aa[3];
                ceres::QuaternionToAngleAxis(x,aa);
                y << aa[0], aa[1], aa[2];
            }
            
            virtual void add(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                ceres::QuaternionProduct(v1,v2,v3);
            }

            virtual void sub(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                DT v2inv[4] = {v2[0],-v2[1],-v2[2],-v2[3]};
                ceres::QuaternionProduct(v1,v2inv,v3);
            }
        };


    template <typename DT, int dim>
        struct DataDescriptorEigen : public DataDescriptor<DT, dim, dim> {
            typedef DT DataType;
            typedef Eigen::Matrix<DT,dim,1> VarType;
            typedef VarType& RefType;
            typedef const VarType & ConstRefType;
            typedef VarType * WritableType;

            typedef typename DataDescriptor<DT,dim,dim>::LogVarType LogVarType;
            typedef typename DataDescriptor<DT,dim,dim>::ConstRefLogType ConstRefLogType;
            typedef typename DataDescriptor<DT,dim,dim>::LogWritableType LogWritableType;

            virtual VarType create() const {
                return VarType::Zero();
            }

            virtual void destroy(RefType v) const {}


            virtual void set(ConstRefType x, WritableType y) const {
                *y = x;
            }

            virtual WritableType writable(RefType x) const {
                return &x;
            }

            virtual void exp(ConstRefLogType x, WritableType y) const  {
                *y = x;
            }

            virtual void log(ConstRefType x, LogWritableType y) const {
                *y = x;
            }
            
            virtual void add(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                *v3 = v1 + v2;
            }

            virtual void sub(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                *v3 = v1 - v2;
            }
        };


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


    struct TimeWarper {
        double t0;
        double delta;
        size_t n_knots;
        TimeWarper(double t0, double delta, size_t n_knots) :
            t0(t0), delta(delta), n_knots(n_knots) {}

        // Scaling factor for derivative of u(t) 
        double dudt_scale() const {
            return 1/delta;
        }

        // Scaling factor for 2nd derivative of u(t)
        double d2udt2_scale() const {
            return 1/(delta*delta);
        }

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


    template <typename DescriptorType>
        struct TRefUniformSpline : public TSplineNamespace<DescriptorType> {
            typedef typename DescriptorType::DataType DataType;
            typedef typename DescriptorType::VarType VarType;
            typedef typename DescriptorType::LogVarType LogVarType;
            typedef typename DescriptorType::ConstRefType ConstRefType;
            typedef typename DescriptorType::WritableType WritableType;
            typedef typename DescriptorType::LogWritableType LogWritableType;

            ConstRefType K0;
            ConstRefType K1;
            ConstRefType K2;
            ConstRefType K3;
            LogVarType lK[4], dK[3];

            TRefUniformSpline(ConstRefType k0, ConstRefType k1, ConstRefType k2, ConstRefType k3) 
                : K0(k0), K1(k1), K2(k2), K3(k3)
            { 
                VarType diff=this->create();
                this->log(K0,lK[0]); 
                this->log(K1,lK[1]); 
                this->log(K2,lK[2]); 
                this->log(K3,lK[3]);
                this->sub(K1,K0,this->writable(diff)); this->log(diff,dK[0]);
                this->sub(K2,K1,this->writable(diff)); this->log(diff,dK[1]);
                this->sub(K3,K2,this->writable(diff)); this->log(diff,dK[2]);
                this->destroy(diff);
            }

            bool evaluate(DataType u, WritableType fu) const {
                Eigen::Matrix<DataType, 4, 1> Mu = TSplineNamespace<DescriptorType>::spline_B(u);
                VarType R = this->create();
                this->exp(LogVarType::Zero(),this->writable(R));
                VarType ex = this->create();
                for (int i=0;i<4;i++) {
                    this->exp(Mu(i,0)*lK[i], this->writable(ex));
                    this->add(R,ex,this->writable(R));
                }
                this->set(R, fu);
                this->destroy(ex);
                this->destroy(R);
                // std::cout << "Evaluate " << u << " " << V << " " << Mu.transpose() << " " << fu << std::endl;
                return true;
            }

            bool evaluate(DataType u, WritableType fu, LogWritableType dfudt, LogWritableType d2fudt2) const {
                Eigen::Matrix<DataType, 4, 1> Mu = TSplineNamespace<DescriptorType>::spline_B(u);
                VarType R = this->create();
                this->exp(LogVarType::Zero(),this->writable(R));
                VarType ex = this->create();
                for (int i=0;i<4;i++) {
                    this->exp(Mu(i,0)*lK[i], this->writable(ex));
                    this->add(R,ex,this->writable(R));
                }
                this->set(R, fu);
                this->destroy(ex);
                this->destroy(R);
                // std::cout << "Evaluate " << u << " " << V << " " << Mu.transpose() << " " << fu << std::endl;
                return true;
            }

            bool cum_evaluate(DataType u, WritableType fu) const {
                Eigen::Matrix<DataType, 4, 1> Mu = TSplineNamespace<DescriptorType>::cum_spline_B(u);
                VarType R = this->create();
                this->set(K0,this->writable(R));
                VarType ex = this->create();
                for (int i=1;i<4;i++) {
                    this->exp(Mu(i,0)*dK[i-1], this->writable(ex));
                    this->add(R,ex,this->writable(R));
                }
                this->set(R, fu);
                this->destroy(ex);
                this->destroy(R);
                return true;
            }
        };

    template <typename DT> 
        struct TRef1DUniformSpline : public TRefUniformSpline<DataDescriptor1D<DT>> {
            typedef typename TRefUniformSpline<DataDescriptor1D<DT>>::ConstRefType ConstRefType;
            TRef1DUniformSpline(ConstRefType k0, ConstRefType k1, ConstRefType k2, ConstRefType k3) :
                TRefUniformSpline<DataDescriptor1D<DT>>(k0,k1,k2,k3) {}
        };

    template <typename DT, int dim> 
        struct TRefPtrUniformSpline : public TRefUniformSpline<DataDescriptorPtr<DT,dim>> {
            typedef typename TRefUniformSpline<DataDescriptorPtr<DT,dim>>::ConstRefType ConstRefType;
            TRefPtrUniformSpline(ConstRefType k0, ConstRefType k1, ConstRefType k2, ConstRefType k3) :
                TRefUniformSpline<DataDescriptorPtr<DT,dim>>(k0,k1,k2,k3) {}
        };

    template <typename DT> 
        struct TRefQuaternionUniformSpline : public TRefUniformSpline<DataDescriptorQuaternion<DT>> {
            typedef typename TRefUniformSpline<DataDescriptorQuaternion<DT>>::ConstRefType ConstRefType;
            TRefQuaternionUniformSpline(ConstRefType k0, ConstRefType k1, ConstRefType k2, ConstRefType k3) :
                TRefUniformSpline<DataDescriptorQuaternion<DT>>(k0,k1,k2,k3) {}
        };

    template <typename DT, int dim> 
        struct TRefEigenUniformSpline : public TRefUniformSpline<DataDescriptorEigen<DT,dim>> {
            typedef typename TRefUniformSpline<DataDescriptorEigen<DT,dim>>::ConstRefType ConstRefType;
            TRefEigenUniformSpline(ConstRefType k0, ConstRefType k1, ConstRefType k2, ConstRefType k3) :
                TRefUniformSpline<DataDescriptorEigen<DT,dim>>(k0,k1,k2,k3) {}
        };

    template <class SplineType> 
        struct UniformSpline {
            std::vector<typename SplineType::VarType> knots;
            TimeWarper warper;

            UniformSpline(double tmin, double tmax, size_t n_knots) : knots(n_knots), warper(tmin, (tmax-tmin)/(n_knots-1) , n_knots) {}
            UniformSpline(double tmin, size_t n_knots, double delta) : knots(n_knots), warper(tmin, delta , n_knots) {}
            UniformSpline(TimeWarper warper) : knots(warper.n_knots), warper(warper) {}
            
            template <class IT>
                void import(IT begin) {
                    std::copy(begin,begin+knots.size(),knots.begin());
                }

            bool evaluate(typename SplineType::DataType t, typename SplineType::WritableType fu) const {
                std::pair<size_t,typename SplineType::DataType> iu = warper(t);
                // printf("S Evaluate t=%f, i=%d, u=%f\n",t,int(iu.first),iu.second);
                SplineType s(knots[iu.first-1],knots[iu.first],knots[iu.first+1],knots[iu.first+2]);
                return s.evaluate(iu.second, fu);
            }

            bool evaluate(typename SplineType::DataType t, typename SplineType::WritableType fu,
                    typename SplineType::LogWritableType dfudt,  typename SplineType::LogWritableType d2fudt2) const {
                std::pair<size_t,typename SplineType::DataType> iu = warper(t);
                // printf("S Evaluate t=%f, i=%d, u=%f\n",t,int(iu.first),iu.second);
                SplineType s(knots[iu.first-1],knots[iu.first],knots[iu.first+1],knots[iu.first+2]);
                if (!s.evaluate(iu.second, fu, dfudt, d2fudt2)) {
                    return false;
                }
                dfudt *= warper.dudt_scale();
                d2fudt2 *= warper.d2udt2_scale();
                return true;
            }

            bool cum_evaluate(typename SplineType::DataType t, typename SplineType::WritableType fu) const {
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
            double weight;
            bool cum_eval;

            SplineError(double u, const double *yin, double weight=1, bool cum_eval=false) : u(u), weight(weight), cum_eval(cum_eval) {
                std::copy(yin+0,yin+dim,y+0);
            }


            template <typename T>
                bool operator()(const T *const k1, const T* const k2, const T* const k3, const T *const k4,
                        T* residuals) const {
                    TRefPtrUniformSpline<T,dim> s(k1,k2,k3,k4);
                    T pred[dim];
                    if (cum_eval) {
                        s.cum_evaluate(T(u),pred);
                    } else {
                        s.evaluate(T(u),pred);
                    }

                    for (int i=0;i<dim;i++) {
                        residuals[i] = (pred[i] - T(y[i]))/T(weight);
                    }
                    return true;
                }
        };



    // TODO: Split file for readability
    // TODO: Add eval with derivative
}


#endif // CERES_SPLINES_H
