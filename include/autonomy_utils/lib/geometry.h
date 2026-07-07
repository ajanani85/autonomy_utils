#ifndef AUTOMODALITY_LIB_GEOMETRY_H_
#define AUTOMODALITY_LIB_GEOMETRY_H_

#include <math.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#define _PI_R 3.14159
#define _TWOPI_R (2 * _PI_R)
const double PI_2 = 1.5707963;

#define C_EARTH (double)6378137.0
#define E2 (double)0.00669437999014 // eccentricity squared

#define deg2rad(d) (((d) / 180.0) * _PI_R)
#define rad2deg(d) (((d) / _PI_R) * 180.0)

#define quat2roll(w, x, y, z) (atan2(2.0 * (z * y + w * x), 1.0 - 2.0 * (x * x + y * y)))
#define quat2pitch(w, x, y, z) (asin(2.0 * (y * w - z * x)))
#define quat2yaw(w, x, y, z) (atan2(2.0 * (z * w + x * y), -1.0 + 2.0 * (w * w + x * x)))

#define Point2Eigen(P, E)   \
        {                   \
                E[0] = P.x; \
                E[1] = P.y; \
                E[2] = P.z; \
        }
#define Eigen2Point(E, P)   \
        {                   \
                P.x = E[0]; \
                P.y = E[1]; \
                P.z = E[2]; \
        }

namespace ros2
{

        struct Point
        {
                double x;
                double y;
                double z;
        };

        struct PointStamp
        {
                Eigen::Vector3d location;
                double stamp;
        };

        // This takes any angle and makes sure that it is between -PI and +PI
        double wrap_pi(double bearing);

        // This takes any angle and makes sure that it is between 0 and 2 * PI
        double wrap_2pi(double bearing);

        //==================================
        // Distances and arc lengths
        //==================================

        // This calculates the distance between two points
        double pointDistance(geometry_msgs::msg::Point &p1, geometry_msgs::msg::Point &p2);
        double pointDistance(ros2::Point &p1, ros2::Point &p2);

        double arcLength(double x1, double y1, double x2, double y2, double ang);

        //==================================
        // Quaternion conversions
        //==================================

        /*
         * If A is an unit vector representing the reference zero axis of rotation and 'angle'
         *   is some angle from that zero axis the the quaternion representing that orientation is
         * qx = Ax * sin(angle/2)
         * qy = Ay * sin(angle/2)
         * qz = Az * sin(angle/2)
         * qw = cos(angle/2)
         *
         * For a simple rotation wrt the x axis the zero axis is represented by 1, 0, 0 and therefore
         * angle = atan2( vector.x, vector.z ) // Note: I expected atan2(z,x) but OP reported success with atan2(x,z) instead! Switch around if you see 90° off.
         * qx = 1 * sin( angle/2 )
         * qy = 0
         * qz = 0
         * qw = cos( angle/2 )
         */

        /**
         * @brief Takes RPY and converts it to a Quaternion
         * @param q reference to quaternion to be generated
         * @param roll roll
         * @param pitch pitch
         * @param yaw yaw
         * @param isDegree if TRUE then the roll, pitch, yaw parameters are in degrees, else they are radians
         * @return void
         */
        void RPY_to_Quaternion(Eigen::Quaterniond &q, double roll, double pitch, double yaw, bool isDegree = false);
        void RPY_to_Quaternion(geometry_msgs::msg::Quaternion &q, double roll, double pitch, double yaw, bool isDegree = false);

        /**
         * @brief Calculates the orientation quaternion from one point to another. Not yet implemented.
         * @param q reference to quaternion to be generated
         * @param p1 first point
         * @param p2 second point
         * @return void
         */
        void points_to_Quaternion(Eigen::Quaterniond &q, Eigen::Vector3d &p1, Eigen::Vector3d &p2);

        /// takes a quaternion and converts it to RPY
        Eigen::Vector3d quaternion_to_RPY(const Eigen::Quaterniond &q);
        Eigen::Vector3d quaternion_to_RPY(const geometry_msgs::msg::Quaternion &q);

        /**
         * @brief Calculates the angle between two orientation vectors.
         * @param v1 starting vector
         * @param v2 ending vector
         * @return minimum angle between the two vectors
         *
         * angle = acos((v1 dot v2) / (||v1|| * ||v2||));
         */
        double angleBetweenVector(Eigen::Vector3d &v1, Eigen::Vector3d &v2);

        /// Takes an ENU based orientation and generates the Roll and Pitch in the body FLU frame wrt the ground plane
        void enuOrientation_to_BodyRP(Eigen::Quaterniond &enu_orientation, double &body_roll, double &body_pitch);

        // This method is used to calculate an XYZ coordinate wrt the ground plane of the ENU frame below the drone.
        // The XY coordinate is specified by the vector point directly from the bottom of the drone assuming the
        //   drone is in an ENU orientation represented by the quaternion enu_orientation and 'distance' is the length of the
        //   vector fromthe bottom of the drone to the XY point. Note that this is different than the distance of the drone to
        //   the ground plane.
        // The XY coordinate is in the bodyFLU frame of the drone. The Z coordinate is the drones distance from the ground plane.
        void enuOrientation_to_BodyXYZ(Eigen::Quaterniond &enu_orientation, double distance, Eigen::Vector3d &body_xyz);

        // Takes a quaternion and rotates the Yaw by the given amount given in radians
        void rotateYaw(Eigen::Quaterniond &org_q, double delta_yaw, Eigen::Quaterniond &new_q);
        void rotateYaw(geometry_msgs::msg::Quaternion &org_q, double delta_yaw, geometry_msgs::msg::Quaternion &new_q);

        // Obsoleting
        // Eigen::Vector3d quaternion_to_RPY2(Eigen::Quaterniond& q);

        //==================================
        // Velocity and point projections
        //==================================

        /// Calculates the velocity vector from the point prev to the point current
        void calculateVelocity(Eigen::Vector3d &prev, Eigen::Vector3d &current,
                               Eigen::Vector3d &velocity, double dt);

        // Project a 3D position from the 'current' position along the velocity vector 'velocity'
        //   assuming time increment 'dt'. The new position is set in 'new_pos'
        void projectPosition(Eigen::Vector3d &current, Eigen::Vector3d &velocity,
                             double dt, Eigen::Vector3d &new_pos);

        // Project a 2D position from the 'from' point a 'distance' along the 'direction' angle.
        // The new position is in new_pos and the z position is set to 0
        void projectPosition(ros2::Point from, double direction, double distance,
                             ros2::Point &new_pos);

        //==================================
        // Finding points on lines
        //==================================

        // Calculate 'vector' from point 'pt' to the perpendicular point on line defined by 'lp1' and 'lp2'
        void vectorToLine(Eigen::Vector3d &vector, Eigen::Vector3d &lp1,
                          Eigen::Vector3d &lp2, Eigen::Vector3d &pt);

        // Calculate vector from point 'pt' to the perpendicular point on line defined by 'lp1' and direction vector 'dir' from 'lp1'.
        void vectorToVector(Eigen::Vector3d &vector, Eigen::Vector3d &lp1,
                            Eigen::Vector3d &dir, Eigen::Vector3d &pt);

        // Calculate vector from point 'pt' along the unit direction vector 'dir' to the point 'lp1'.
        void vectorAlongVector(Eigen::Vector3d &vector, Eigen::Vector3d &lp1,
                               Eigen::Vector3d &dir, Eigen::Vector3d &pt);

        // Given a line defined by points lp1 and lp2 find the "point" on that line such that
        //   the line from pt to "point" is perpendicular to lp1-lp2.
        // In addition it returns true if "point" is on the line segment defined by lp1-lp2
        bool pointOnLine(Eigen::Vector3d &point, Eigen::Vector3d &lp1,
                         Eigen::Vector3d &lp2, Eigen::Vector3d &pt);
        bool pointOnLine(geometry_msgs::msg::Point &point, geometry_msgs::msg::Point &lp1,
                         geometry_msgs::msg::Point &lp2, geometry_msgs::msg::Point &pt);

        // Given a line segment defined by points lp1 and lp2 find the "point" on that line segment such that
        //   the line from pt to "point" is perpendicular to lp1-lp2.
        // In addition it returns true if "point" is on the line segment defined by lp1-lp2.
        // If false then the point returned is one of the line segment endpoints lp1 or lp2.
        bool pointOnLineSegment(Eigen::Vector3d &point, Eigen::Vector3d &lp1,
                                Eigen::Vector3d &lp2, Eigen::Vector3d &pt);
        bool pointOnLineSegment(geometry_msgs::msg::Point &point, geometry_msgs::msg::Point &lp1,
                                geometry_msgs::msg::Point &lp2, geometry_msgs::msg::Point &pt);

        // Given a line defined by 'p1' and 'p2' and a 3rd point 'p3' calculate a point 'p4'
        // that is on line p1-p2 and is perpendicular to 'p3' wrt to p1-p2.
        // i.e. p4 is the point on line p1-p2 that is closest to 'p3'
        // Note that this is ONLY a 2D calculation in the XY plane
        void pointToLine(ros2::Point &p1, ros2::Point &p2, ros2::Point &p3, ros2::Point &p4);

        //==================================
        // Working with GPS coordinates
        //==================================

        // Calculate the x, y coordinates from the GPS coordinates lat_from, lon_from to lat_tp, lon_to
        //   and add the origin_x and origin_y coordinates.
        void latLon_to_XY_origin(double lat_from, double lon_from,
                                 double lat_to, double lon_to,
                                 double origin_x, double origin_y,
                                 double &x, double &y);

        /**
         * @brief Calculates the x and y coordinates in a local Cartesian frame from GPS coordinates.
         *
         * This function computes the x and y coordinates in a local Cartesian frame based on the
         * latitude and longitude of two points. The calculation assumes a flat Earth approximation
         * and uses the haversine formula to determine the distance between the two points.
         *
         * @param lat_from Latitude of the starting point in degrees.
         * @param lon_from Longitude of the starting point in degrees.
         * @param lat_to Latitude of the target point in degrees.
         * @param lon_to Longitude of the target point in degrees.
         * @param x Reference to the variable where the calculated x-coordinate will be stored.
         * @param y Reference to the variable where the calculated y-coordinate will be stored.
         *
         * @note This function assumes that the Earth is a perfect sphere and does not account for
         *       ellipsoidal effects. It is suitable for small distances where the curvature of the
         *       Earth can be neglected.
         */
        void latLon_to_XY(double lat_from, double lon_from, double lat_to, double lon_to, double &x, double &y);

        /**
         * @brief Converts geographical coordinates (latitude and longitude) to Cartesian coordinates (X, Y)
         *        relative to a reference point using the ENU (East-North-Up) coordinate system.
         *
         * @param ref_latitude The latitude of the reference point in degrees.
         * @param ref_longitude The longitude of the reference point in degrees.
         * @param enu_to_asset_frame_yaw The yaw angle (in radians) to rotate the ENU frame to align with the asset frame.
         * @param given_latitude The latitude of the point to be converted in degrees.
         * @param given_longitude The longitude of the point to be converted in degrees.
         * @param X Reference to a double where the calculated X coordinate will be stored.
         * @param Y Reference to a double where the calculated Y coordinate will be stored.
         * @param earth_radius The radius of the Earth in meters (used for calculations).
         *
         * @return bearing or heading between the pose and the reference
         */
        double latLon_to_XY(double ref_latitude, double ref_longitude, double given_latitude, double given_longitude,
                            double enu_to_asset_frame_yaw,
                            double &X, double &Y, double earth_radius);

        /**
         * @brief Converts Cartesian coordinates (X, Y) back to geographical coordinates (latitude and longitude)
         *        relative to a reference point using the ENU (East-North-Up) coordinate system
         * @param ref_latitude The latitude of the reference point in degrees.
         * @param ref_longitude The longitude of the reference point in degrees.
         * @param X The X coordinate to be converted.
         * @param Y The Y coordinate to be converted.
         * @param lat_out Reference to a double where the calculated latitude will be stored.
         * @param lon_out Reference to a double where the calculated longitude will be stored.
         */
        void XY_to_latLon(double ref_latitude, double ref_longitude, double X, double Y,
                          double &lat_out, double &lon_out);

        /**
         * @brief this function transforms the ref_latitude and ref_longitude with the passed cartesian coordinates and
         * returns the new latitude and longitude
         * 
         * @param antenna_latitude the latitude of the antenna in degrees
         * @param antenna_longitude the longitude of the antenna in degrees
         * @param antenna_x_from_base the x coordinate of the antenna relative to the base
         * @param antenna_y_from_base the y coordinate of the antenna relative to the base
         * @param heading_radian_enu the heading of the transform in radians
         * @param base_latitude the final projected latitude of the base in degrees
         * @param base_longitude the final projected longitude of the base in degrees
         */
        void latlong_transform(double antenna_latitude, double antenna_longitude, double antenna_x_from_base, 
                double antenna_y_from_base, double heading_radian_enu, double &base_latitude, double &base_longitude);

        //==================================
        // Rotating coordinates
        //==================================

        // Rotate in_vecd by the rotation represented by the quaternion.
        // This can be useful for transforming vectors from body frame coordinates to some fixed frame such as ENU
        //   if you have the drones orientation in that frame.
        void rotateVector(Eigen::Quaterniond &q, Eigen::Vector3d &in_vec, Eigen::Vector3d &out_vec);
        void rotateVector(Eigen::Quaterniond &q, double x, double y, double z, Eigen::Vector3d &out_vec);
        void rotateVector(geometry_msgs::msg::Quaternion &q, double x, double y, double z, Eigen::Vector3d &out_vec);

        // Given a 2D coordinate defined by 'x_org, y_org', calculate a new coordinate 'x_new, y_new' that is rotated
        //   by theta radians
        // Note that this produces the same results as the transformXYFrame method.
        void rotatePoint(double x_org, double y_org, double &x_new, double &y_new, double theta_R);

        // Given a vector 'org_vec' and a different frame that is rotated by 'rad_diff' radians around the Z axis,
        //   return the vector coordinates 'new_vec' in the different frame.
        // Note that this produces the same results as the rotatePoint method.
        void transformXYFrame(Eigen::Vector3d &org_vec, double rad_diff, Eigen::Vector3d &new_vec);
        void transformXYFrame(geometry_msgs::msg::Vector3 &org_vec, double rad_diff, geometry_msgs::msg::Vector3 &new_vec);

        // Same as transformXYFrame with rad_diff parameter, but it takes the given orientation and extract the yaw before doing the above transform
        // Note that the orientation is assumed to be the org_vec frame of reference with respect to new_vec frame of reference,
        //   i.e. the org_vec frame is a child to the new_vec frame. If this needs to be reversed then set the reverse_rotation
        //   flag to true.
        // For example the drone's orientation is typically given in the ENU frame and assume you have a velocity vector also given in the
        //   ENU frame. In oder to use this method to rotate the velocity vector so it lines up with the heading of the vehicle you would
        //   pass in the orientation and the velocity vector AND set reverse_rotation to true.
        // Returns to rotation amount.
        double transformXYFrame(Eigen::Vector3d &org_vec, geometry_msgs::msg::Quaternion &orientation,
                                Eigen::Vector3d &new_vec, bool reverse_rotation = false);
        double transformXYFrame(Eigen::Vector3d &org_vec, Eigen::Quaterniond &orientation,
                                Eigen::Vector3d &new_vec, bool reverse_rotation = false);

        // This method takes an orientation expressed as a quaternion and levels it out by zeroing the roll and pich
        void levelOrientation(const geometry_msgs::msg::Quaternion &org_orientation, geometry_msgs::msg::Quaternion &level_orientation);
        void levelOrientation(const Eigen::Quaterniond &org_orientation, Eigen::Quaterniond &level_orientation);

        /**
         * @brief Determines the UTM zone for given latitude and longitude coordinates.
         * @param latitude The latitude in decimal degrees.
         * @param longitude The longitude in decimal degrees.
         * @return A string representing the UTM zone (e.g., "33T").
         */
        std::string getUTMZone(double latitude, double longitude);

        void latLon_to_UTM(double latitude, double longitude, double &utm_easting, double &utm_northing, std::string &utm_zone);

        bool utmToLatLon(double utm_easting, double utm_northing, const std::string &utm_zone,
                         double &latitude, double &longitude);

        //==================================
        // Debug stuff
        //==================================

        // Routine to test the above functions
        void test();

        void testQuats();

        // Print the coordiantes of a point
        void printPoint(Eigen::Vector3d &p);
        // Print the RPY coordinates
        void printAttitude(Eigen::Vector3d &a);
        // Print the XYZW coordinates of quaternion
        void printRotation(Eigen::Quaterniond &a);

}

#endif /* AUTOMODALITY_LIB_GEOMETRY_H_ */