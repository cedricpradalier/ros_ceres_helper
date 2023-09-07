#ifndef DATA_DESCRIPTOR_EIGEN_H
#define DATA_DESCRIPTOR_EIGEN_H

#include <ros_ceres_helper/data_descriptor.h>


namespace cerise{ 


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


}


#endif // DATA_DESCRIPTOR_EIGEN_H
