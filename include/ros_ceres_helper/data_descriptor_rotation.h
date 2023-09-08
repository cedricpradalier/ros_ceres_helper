#ifndef DATA_DESCRIPTOR_ROTATION_H
#define DATA_DESCRIPTOR_ROTATION_H

#include <ceres/rotation.h>
#include <ros_ceres_helper/ceres_template_rotations.h>
#include <ros_ceres_helper/data_descriptor.h>



namespace cerise{ 

    template <typename DT>
        struct DataDescriptorRotation : public DataDescriptor<DT, 4, 3> {
            typedef DT DataType;
            typedef TRotation<DT> VarType;
            typedef TRotation<DT> & RefType;
            typedef const TRotation<DT> & ConstRefType;
            typedef TRotation<DT> & WritableType;

            typedef typename DataDescriptor<DT,4,3>::LogVarType LogVarType;
            typedef typename DataDescriptor<DT,4,3>::ConstRefLogType ConstRefLogType;
            typedef typename DataDescriptor<DT,4,3>::LogWritableType LogWritableType;

            virtual VarType create() const {
                return TRotation<DT>();
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
                DT aa[3] = {x(0,0), x(1,0), x(2,0)};
                y.setFromAngleAxis(aa);
            }

            virtual void log(ConstRefType x, LogWritableType y) const {
                DT aa[3];
                x.getAngleAxis(aa);
                y << aa[0], aa[1], aa[2];
            }
            
            virtual void add(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                v3 = v2 * v1;
            }

            virtual void sub(ConstRefType v1, ConstRefType v2, WritableType v3) const {
                v3 = v2.inverse() * v1;
            }
        };

}


#endif // DATA_DESCRIPTOR_ROTATION
