#ifndef DATA_DESCRIPTOR_QUATERNION_H
#define DATA_DESCRIPTOR_QUATERNION_H

#include <ceres/rotation.h>
#include <ros_ceres_helper/data_descriptor.h>



namespace cerise{ 

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
                DT aa[3] = {DT(0),DT(0),DT(0)};
                DT * P = new DT[4];
                ceres::AngleAxisToQuaternion(aa,P);
                return P;
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

}


#endif // DATA_DESCRIPTOR_QUATERNION_H
