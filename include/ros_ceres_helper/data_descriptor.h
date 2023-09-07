#ifndef DATA_DESCRIPTOR_H
#define DATA_DESCRIPTOR_H

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

}


#endif // DATA_DESCRIPTOR_H
