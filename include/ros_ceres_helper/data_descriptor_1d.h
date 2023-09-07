#ifndef DATA_DESCRIPTOR_1D_H
#define DATA_DESCRIPTOR_1D_H

#include <ros_ceres_helper/data_descriptor.h>


namespace cerise{ 


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

}


#endif // DATA_DESCRIPTOR_1D_H
