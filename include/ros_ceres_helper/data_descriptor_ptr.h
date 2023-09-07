#ifndef DATA_DESCRIPTOR_PTR_H
#define DATA_DESCRIPTOR_PTR_H

#include <ros_ceres_helper/data_descriptor.h>
#include <Eigen/Dense>


namespace cerise{ 

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
                DT * P = new DT[dim];
                for (int i=0;i<dim;i++) {
                    P[i] = DT(0);
                }
                return P;
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

}


#endif // DATA_DESCRIPTOR_PTR_H
