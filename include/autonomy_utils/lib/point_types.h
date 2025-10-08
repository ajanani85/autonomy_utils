#ifndef AUTONOMY_UTILS_LIB_POINT_TYPE_H_
#define AUTONOMY_UTILS_LIB_POINT_TYPE_H_

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_types.h>
namespace pcl {
struct EIGEN_ALIGN16 PointXYZIR
{
 PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
 float intensity;
 float range;
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} ;                    // enforce SSE padding for correct memory alignment

// FOR XYZ + TIME + INTENSITY + RANGE + REFLECTIVITY + RING + AMBIENT
struct EIGEN_ALIGN16 PointXYZTIRRRA
{
 PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
 float intensity;
 float range;
 uint16_t reflectivity;
 uint16_t ring;
 uint16_t ambient;
 uint32_t time;
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} ;                    // enforce SSE padding for correct memory alignment

struct EIGEN_ALIGN16 PointXYZIRNormal
{
 PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
 PCL_ADD_UNION_NORMAL4D;
 float intensity;
 float range;
 EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} ;                    // enforce SSE padding for correct memory alignment

}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZIR,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, range, range)
 )

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZIRNormal,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, normal_x, normal_x)
                                    (float, normal_y, normal_y)
                                    (float, normal_z, normal_z)
                                    (float, intensity, intensity)
                                    (float, range, range)
)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZTIRRRA,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, range, range)
                                    (uint16_t, reflectivity, reflectivity)
                                    (uint16_t, ring, ring)
                                    (uint16_t, ambient, ambient)
                                    (uint32_t, time, time)
)

#endif /*AUTONOMY_UTILS_LIB_POINT_TYPE_H_*/