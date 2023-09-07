#ifndef CERES_SPLINES_TYPES_H
#define CERES_SPLINES_TYPES_H

#include <ros_ceres_helper/data_descriptor.h>
#include <ros_ceres_helper/data_descriptor_1d.h>
#include <ros_ceres_helper/data_descriptor_ptr.h>
#include <ros_ceres_helper/data_descriptor_quaternion.h>
#include <ros_ceres_helper/data_descriptor_rotation.h>
#include <ros_ceres_helper/data_descriptor_pose.h>
#include <ros_ceres_helper/data_descriptor_eigen.h>
#include <ros_ceres_helper/ref_uniform_spline.h>
#include <ros_ceres_helper/uniform_spline.h>


namespace cerise{ 
    template <typename DT> 
        struct TRef1DUniformSpline : public TRefUniformSpline<DataDescriptor1D<DT>> {
            typedef typename TRefUniformSpline<DataDescriptor1D<DT>>::ConstRefType ConstRefType;
            TRef1DUniformSpline(ConstRefType k0, ConstRefType k1, ConstRefType k2, ConstRefType k3) :
                TRefUniformSpline<DataDescriptor1D<DT>>(k0,k1,k2,k3) {}
        };

    typedef UniformSpline<TRef1DUniformSpline<double>> GenericSpline1D;

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

    template <typename DT> 
        struct TRefRotationUniformSpline : public TRefUniformSpline<DataDescriptorRotation<DT>> {
            typedef typename TRefUniformSpline<DataDescriptorRotation<DT>>::ConstRefType ConstRefType;
            TRefRotationUniformSpline(ConstRefType k0, ConstRefType k1, ConstRefType k2, ConstRefType k3) :
                TRefUniformSpline<DataDescriptorRotation<DT>>(k0,k1,k2,k3) {}
        };

    template <typename DT> 
        struct TRefPoseUniformSpline : public TRefUniformSpline<DataDescriptorPose<DT>> {
            typedef typename TRefUniformSpline<DataDescriptorPose<DT>>::ConstRefType ConstRefType;
            TRefPoseUniformSpline(ConstRefType k0, ConstRefType k1, ConstRefType k2, ConstRefType k3) :
                TRefUniformSpline<DataDescriptorPose<DT>>(k0,k1,k2,k3) {}
        };

    template <typename DT, int dim> 
        struct TRefEigenUniformSpline : public TRefUniformSpline<DataDescriptorEigen<DT,dim>> {
            typedef typename TRefUniformSpline<DataDescriptorEigen<DT,dim>>::ConstRefType ConstRefType;
            TRefEigenUniformSpline(ConstRefType k0, ConstRefType k1, ConstRefType k2, ConstRefType k3) :
                TRefUniformSpline<DataDescriptorEigen<DT,dim>>(k0,k1,k2,k3) {}
        };
}


#endif // CERES_SPLINES_TYPES_H
