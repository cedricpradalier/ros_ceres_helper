#ifndef REF_UNIFORM_SPLINES_H
#define REF_UNIFORM_SPLINES_H

#include <ros_ceres_helper/data_descriptor.h>
#include <ros_ceres_helper/spline_namespace.h>


namespace cerise{ 

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

}


#endif // REF_UNIFORM_SPLINES_H
