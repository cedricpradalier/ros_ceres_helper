#ifndef DATA_DESCRIPTOR_POSE_H
#define DATA_DESCRIPTOR_POSE_H

#include <ceres/rotation.h>
#include <ros_ceres_helper/ceres_template_poses.h>
#include <ros_ceres_helper/data_descriptor.h>



namespace cerise{ 

    template <typename DT>
        struct DataDescriptorPose : public DataDescriptor<DT, 7, 6> {
            typedef DT DataType;
            typedef TPose<DT> VarType;
            typedef TPose<DT> & RefType;
            typedef const TPose<DT> & ConstRefType;
            typedef TPose<DT> & WritableType;

            typedef typename DataDescriptor<DT,7,6>::LogVarType LogVarType;
            typedef typename DataDescriptor<DT,7,6>::ConstRefLogType ConstRefLogType;
            typedef typename DataDescriptor<DT,7,6>::LogWritableType LogWritableType;

            virtual VarType create() const {
                return TPose<DT>();
            }

            virtual void destroy(RefType v) const {
            }


            virtual void set(ConstRefType x, WritableType y) const {
                y = x;
            }

            virtual WritableType writable(RefType x) const {
                return x;
            }


            virtual void exp(ConstRefLogType x, WritableType y) const  {
                y.T[0] = x(0,0);
                y.T[1] = x(1,0);
                y.T[2] = x(2,0);
                DT aa[3] = {x(3,0), x(4,0), x(5,0)};
                ceres::AngleAxisToQuaternion(aa,y.Q);
            }

            virtual void log(ConstRefType x, LogWritableType y) const {
                DT aa[3];
                ceres::QuaternionToAngleAxis(x.Q,aa);
                y << x.T[0], x.T[1], x.T[2], aa[0], aa[1], aa[2];
            }
            
            virtual void add(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                v3 = v2 * v1;
            }

            virtual void sub(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                v3 = v2.inverse() * v1;
            }
        };

}


#endif // DATA_DESCRIPTOR_POSE
