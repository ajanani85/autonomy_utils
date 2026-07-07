/*
 * am_geometry.cpp
 *
 *  Created on: Feb 3, 2016
 *      Author: ubuntu
 */
#include <autonomy_utils/lib/geometry.h>

namespace ros2
{

    /*
    ** Pi wrapping utility
    */
    double wrap_pi(double bearing)
    {
        while (bearing >= _PI_R)
        {
            bearing -= _TWOPI_R;
        }

        while (bearing <= -_PI_R)
        {
            bearing += _TWOPI_R;
        }

        return bearing;
    }

    double wrap_2pi(double bearing)
    {
        while (bearing >= 2 * _PI_R)
        {
            bearing -= _TWOPI_R;
        }

        while (bearing <= 0)
        {
            bearing += _TWOPI_R;
        }

        return bearing;
    }

    double pointDistance(geometry_msgs::msg::Point &p1, geometry_msgs::msg::Point &p2)
    {
        double xd = p2.x - p1.x;
        double yd = p2.y - p1.y;
        double zd = p2.z - p1.z;
        return sqrt((xd * xd) + (yd * yd) + (zd * zd));
    }

    double pointDistance(ros2::Point &p1, ros2::Point &p2)
    {
        double xd = p2.x - p1.x;
        double yd = p2.y - p1.y;
        double zd = p2.z - p1.z;
        return sqrt((xd * xd) + (yd * yd) + (zd * zd));
    }

    double arcLength(double x1, double y1, double x2, double y2, double ang)
    {
        double xd = x2 - x1;
        double yd = y2 - y1;

        double c = sqrt((xd * xd) + (yd * yd));

        double a = atan2(yd, xd);
        double ang_diff = ang - a;
        ang_diff = ang_diff >= 0.0 ? ang_diff : -ang_diff;

        if (ang_diff >= PI_2)
        {
            return _PI_R * c / 2;
        }

        if (ang_diff == 0.0)
        {
            return c;
        }

        double s = (ang_diff * c) / (2 * sin(ang_diff / 2));

        return s;
    }

    void RPY_to_Quaternion(Eigen::Quaterniond &q, double roll, double pitch, double yaw, bool isDegree)
    {
        if (isDegree)
        {
            roll = deg2rad(roll);
            pitch = deg2rad(pitch);
            yaw = deg2rad(yaw);
        }

        Eigen::AngleAxisd aaZ(yaw, Eigen::Vector3d::UnitZ());

        Eigen::AngleAxisd aaY(pitch, Eigen::Vector3d::UnitY());

        Eigen::AngleAxisd aaX(roll, Eigen::Vector3d::UnitX());

        q = aaX * aaY * aaZ;

        return;
    }

    void RPY_to_Quaternion(geometry_msgs::msg::Quaternion &q, double roll, double pitch, double yaw, bool isDegree)
    {
        Eigen::Quaterniond qt;
        RPY_to_Quaternion(qt, roll, pitch, yaw, isDegree);

        q.w = qt.w();
        q.x = qt.x();
        q.y = qt.y();
        q.z = qt.z();

        return;
    }

    void points_to_Quaternion(Eigen::Quaterniond &q, Eigen::Vector3d &p1,
                              Eigen::Vector3d &p2)
    {
    }

    double angleBetweenVector(Eigen::Vector3d &v1, Eigen::Vector3d &v2)
    {
        double angle = acos(v1.dot(v2) / (v1.norm() * v2.norm()));

        return angle;
    }

    Eigen::Vector3d quaternion_to_RPY(const geometry_msgs::msg::Quaternion &q)
    {
        Eigen::Quaterniond qe;

        qe.x() = q.x;
        qe.y() = q.y;
        qe.z() = q.z;
        qe.w() = q.w;
        return quaternion_to_RPY(qe);
    }

    Eigen::Vector3d quaternion_to_RPY(const Eigen::Quaterniond &q)
    {
        Eigen::Vector3d ea = q.matrix().eulerAngles(0, 1, 2);

        if (ea[0] > PI_2)
        {
            ea[0] -= _PI_R;
            if (ea[1] > 0.0)
            {
                ea[1] = _PI_R - ea[1];
            }
            else
            {
                ea[1] = -_PI_R - ea[1];
            }
            if (ea[2] > 0.0)
            {
                ea[2] = ea[2] - _PI_R;
            }
            else
            {
                ea[2] = ea[2] + _PI_R;
            }
        }

        return ea;
    }

    void enuOrientation_to_BodyRP(Eigen::Quaterniond &enu_orientation, double &body_roll, double &body_pitch)
    {
        Eigen::Vector3d body_xyz;
        enuOrientation_to_BodyXYZ(enu_orientation, 1.0, body_xyz);
        body_pitch = -atan2(body_xyz[0], body_xyz[2]);
        body_roll = atan2(body_xyz[1], body_xyz[2]);

        //    printf("enuOrientation_to_BodyRP roll[%0.1f]   pitch[%0.1f]\n", rad2deg(body_roll), rad2deg(body_pitch));
    }

    void enuOrientation_to_BodyXYZ(Eigen::Quaterniond &enu_orientation, double distance, Eigen::Vector3d &body_xyz)
    {
        // Get RPY
        Eigen::Vector3d rpy = quaternion_to_RPY(enu_orientation);

        double x_enu, y_enu;
        // First calculate the XY coordinates in the ENU frame
        x_enu = -distance * sin(rpy[1]);
        y_enu = distance * sin(rpy[0]);

        // Rotate the coordinates around the enu Yaw
        rotatePoint(x_enu, y_enu, body_xyz[0], body_xyz[1], -rpy[2]);

        // Calcualte Z
        body_xyz[2] = distance * cos(rpy[0]) * cos(rpy[1]);

        //    printf("enuOrientation_to_BodyXYZ: rpy[%0.1f %0.1f %0.1f ] enuXY[%0.3f  %0.3f] bodyXYZ[%0.3f  %0.3f  %0.3f]\n",
        //           rad2deg(rpy[0]), rad2deg(rpy[1]), rad2deg(rpy[2]),
        //           x_enu, y_enu,
        //           body_xyz[0], body_xyz[1], body_xyz[2]);
    }

    // Obsoleting this method
    // Eigen::Vector3d quaternion_to_RPY2(Eigen::Quaterniond& q) {
    //    Eigen::Vector3d eb = q.matrix().eulerAngles(0,1,2);
    //    Eigen::Vector3d ea;
    //    ea[0] = quat2roll(q.w(), q.x(), q.y(), q.z());
    //    ea[1] = quat2pitch(q.w(), q.x(), q.y(), q.z());
    //    ea[2] = quat2yaw(q.w(), q.x(), q.y(), q.z());
    //
    //    return ea;
    //}
    //

    void rotateYaw(Eigen::Quaterniond &org_q, double delta_yaw, Eigen::Quaterniond &new_q)
    {
        // Get RPY
        Eigen::Vector3d rpy = quaternion_to_RPY(org_q);
        rpy[2] = wrap_pi(rpy[2] + delta_yaw);
        RPY_to_Quaternion(new_q, rpy[0], rpy[1], rpy[2], false);
        return;
    }

    void rotateYaw(geometry_msgs::msg::Quaternion &org_q, double delta_yaw, geometry_msgs::msg::Quaternion &new_q)
    {
        // Get RPY
        Eigen::Vector3d rpy = quaternion_to_RPY(org_q);
        rpy[2] = wrap_pi(rpy[2] + delta_yaw);
        RPY_to_Quaternion(new_q, rpy[0], rpy[1], rpy[2], false);
        return;
    }

    void rotatePoint(double x_org, double y_org, double &x_new, double &y_new, double theta_R)
    {
        double cos_theta = cos(theta_R);
        double sin_theta = sin(theta_R);

        x_new = (x_org * cos_theta) - (y_org * sin_theta);
        y_new = (y_org * cos_theta) + (x_org * sin_theta);

        return;
    }

    void testq(double rd, double pd, double yd)
    {
        Eigen::Quaterniond q;
        Eigen::Vector3d v;

        RPY_to_Quaternion(q, rd, pd, yd, true);
        v = quaternion_to_RPY(q);
        double rn = quat2roll(q.w(), q.x(), q.y(), q.z());
        double pn = quat2pitch(q.w(), q.x(), q.y(), q.z());
        double yn = quat2yaw(q.w(), q.x(), q.y(), q.z());
        printf("\nIN:  r[%0.5f:%0.5f]  p[%0.5f:%0.5f]  y[%0.5f:%0.5f]\n", rd, deg2rad(rd), pd, deg2rad(pd), yd, deg2rad(yd));
        printf("\nTT:  r[%0.5f:%0.5f]  p[%0.5f:%0.5f]  y[%0.5f:%0.5f]\n", rad2deg(rn), rn, rad2deg(pn), pn, rad2deg(yn), yn);
        printf("OUT: r[%0.5f:%0.5f]  p[%0.5f:%0.5f]  y[%0.5f:%0.5f]\n",
               rad2deg(v[0]), v[0], rad2deg(v[1]), v[1], rad2deg(v[2]), v[2]);
        //    v = quaternion_to_RPY2(q);
        //    printf("OUT2: r[%0.5f:%0.5f]  p[%0.5f:%0.5f]  y[%0.5f:%0.5f]\n",
        //            rad2deg(v[0]), v[0], rad2deg(v[1]), v[1], rad2deg(v[2]), v[2]);
        printf("QUAT: w:%0.5f  x:%0.5f  y:%0.5f  z:%0.5f\n", q.w(), q.x(), q.y(), q.z());
    }

    void calculateVelocity(Eigen::Vector3d &prev, Eigen::Vector3d &current,
                           Eigen::Vector3d &velocity, double dt)
    {
        velocity[0] = (current[0] - prev[0]) / dt;
        velocity[1] = (current[1] - prev[1]) / dt;
        velocity[2] = (current[2] - prev[2]) / dt;

        return;
    }

    void projectPosition(Eigen::Vector3d &current, Eigen::Vector3d &velocity,
                         double dt, Eigen::Vector3d &new_pos)
    {
        new_pos[0] = current[0] + (velocity[0] * dt);
        new_pos[1] = current[1] + (velocity[1] * dt);
        new_pos[2] = current[2] + (velocity[2] * dt);

        return;
    }

    void projectPosition(ros2::Point from, double direction, double distance, ros2::Point &new_pos)
    {
        new_pos.x = from.x + (distance * cos(direction));
        new_pos.y = from.y + (distance * sin(direction));
        new_pos.z = 0;

        return;
    }

    double latLon_to_XY(double ref_latitude, double ref_longitude, double given_latitude, double given_longitude,
                        double enu_to_asset_frame_yaw,
                        double &X, double &Y, double earth_radius)
    {
        double REFERENCE_LAT = M_PI * ref_latitude / 180.0;
        double REFERENCE_LONG = M_PI * (ref_longitude) / 180.0;
        const double EARTH = earth_radius;
        const double ENU_TO_ENU_YAW_RAD = M_PI * enu_to_asset_frame_yaw / 180.0;

        const double lat_radians = M_PI * given_latitude / 180.0;
        const double long_radians = M_PI * given_longitude / 180.0;
        const double radians_delta_latitude = lat_radians - REFERENCE_LAT;
        const double radians_delta_longitude = long_radians - REFERENCE_LONG;

        double a = sin(radians_delta_latitude / 2) * sin(radians_delta_latitude / 2) +
                   cos(REFERENCE_LAT) * cos(lat_radians) * sin(radians_delta_longitude / 2) * sin(radians_delta_longitude / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        double d = c * EARTH;
        double bearing = atan2(sin(long_radians - REFERENCE_LONG) * cos(lat_radians),
                               cos(REFERENCE_LAT) * sin(lat_radians) - sin(REFERENCE_LAT) * cos(lat_radians) * cos(long_radians - REFERENCE_LONG));
        X = cos(bearing - ENU_TO_ENU_YAW_RAD) * d;
        Y = -sin(bearing - ENU_TO_ENU_YAW_RAD) * d;

        return bearing;
    }

    void latLon_to_XY(double lat_from, double lon_from, double lat_to, double lon_to, double &x, double &y)
    {
        // double lat_rad = deg2rad(lat_to);
        // double lon_rad = deg2rad(lon_to);
        // double ref_lat_rad = deg2rad(lat_from);
        // double ref_lon_rad = deg2rad(lon_from);

        // double dlat = lat_rad - ref_lat_rad;
        // double dlon = lon_rad - ref_lon_rad;

        // // Approximate x, y in meters
        // x = C_EARTH * dlon * cos(ref_lat_rad);
        // y = C_EARTH * dlat;
        // Compute geodesic between reference point and target point
        double s12 = 0.0;
        double azi1 = 0.0;
        double azi2 = 0.0;

        try
        {
            GeographicLib::Geodesic::WGS84().Inverse(lat_from, lon_from, lat_to, lon_to, s12, azi1, azi2);
        }
        catch (...)
        {
            return;
        }

        // If points coincide
        if (s12 == 0.0)
        {
            x = 0.0;
            y = 0.0;
            return;
        }

        // azi1 is degrees clockwise from north; convert to radians
        const double azi_rad = azi1 * (M_PI / 180.0);

        // ENU: east = s * sin(azi), north = s * cos(azi)
        x = s12 * std::sin(azi_rad);
        y = s12 * std::cos(azi_rad);
    }

    void latLon_to_XY_origin(double lat_from, double lon_from,
                             double lat_to, double lon_to,
                             double origin_x, double origin_y,
                             double &x, double &y)
    {
        latLon_to_XY(lat_from, lon_from, lat_to, lon_to, x, y);
        x += origin_x;
        y += origin_y;
    }

    void XY_to_latLon(double ref_latitude, double ref_longitude, double X, double Y,
                      double &lat_out, double &lon_out)
    {
        // // Convert reference lat/lon to radians
        // double lat0 = ref_latitude * M_PI / 180.0;
        // double lon0 = ref_longitude * M_PI / 180.0;

        // // Prime vertical radius of curvature
        // double N = C_EARTH / std::sqrt(1 - E2 * std::sin(lat0) * std::sin(lat0));

        // // Approximate differential changes
        // double dLat = Y / (N * (1 - E2) / std::pow(1 - E2 * std::sin(lat0) * std::sin(lat0), 1.5));
        // double dLon = X / (N * std::cos(lat0));

        // // Convert to degrees
        // lat_out = ref_latitude + (dLat * 180.0 / M_PI);
        // lon_out = ref_longitude + (dLon * 180.0 / M_PI);

        // distance and azimuth from reference to target
        const double dist = std::hypot(X, Y); // meters
        if (dist == 0.0)
        {
            lat_out = ref_latitude;
            lon_out = ref_longitude;
            return;
        }
        // azimuth: atan2(east, north) radians measured clockwise from north
        const double az_rad = std::atan2(X, Y);
        const double az_deg = az_rad * (180.0 / M_PI);

        double lat2 = 0.0;
        double lon2 = 0.0;
        try
        {
            GeographicLib::Geodesic::WGS84().Direct(ref_latitude, ref_longitude, az_deg, dist, lat2, lon2);
        }
        catch (...)
        {
            return;
        }
        lat_out = lat2;
        lon_out = lon2;
    }

    void latlong_transform( double antenna_latitude, double antenna_longitude, double antenna_x_from_base, double antenna_y_from_base,
        double heading_radian_enu, double &base_latitude, double &base_longitude)
    {
        // Antenna position relative to base_link in base_link frame:
        // +x = forward, +y = left
        //
        // We want base_link relative to antenna, so negate it.
        const double dx_base = antenna_x_from_base;
        const double dy_base = antenna_y_from_base;

        const double dist = std::hypot(dx_base, dy_base);

        if (dist == 0.0)
        {
            base_latitude = antenna_latitude;
            base_longitude = antenna_longitude;
            return;
        }

        // Convert base_link-frame offset into ENU offset.
        //
        // heading_radian_enu is vehicle yaw in ENU:
        // 0 = East, +pi/2 = North.
        const double east =
            dx_base * std::cos(heading_radian_enu) -
            dy_base * std::sin(heading_radian_enu);

        const double north =
            dx_base * std::sin(heading_radian_enu) +
            dy_base * std::cos(heading_radian_enu);

        // GeographicLib bearing convention:
        // 0 deg = North, 90 deg = East.
        const double bearing_rad = std::atan2(east, north);
        const double bearing_deg = bearing_rad * 180.0 / M_PI;

        double lat2 = 0.0;
        double lon2 = 0.0;

        try
        {
            GeographicLib::Geodesic::WGS84().Direct(
                antenna_latitude,
                antenna_longitude,
                bearing_deg,
                dist,
                lat2,
                lon2);
        }
        catch (...)
        {
            return;
        }

        base_latitude = lat2;
        base_longitude = lon2;
    }

    void pointToLine(ros2::Point &p1, ros2::Point &p2, ros2::Point &p3, ros2::Point &p4)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double k = (dy * (p3.x - p1.x) - dx *
                                             (p3.y - p1.y)) /
                   ((dx * dx) + (dy * dy));
        p4.x = p3.x - k * dy;
        p4.y = p3.y + k * dx;

        return;
    }

    void rotateVector(Eigen::Quaterniond &q, Eigen::Vector3d &in_vec, Eigen::Vector3d &out_vec)
    {
        out_vec = q * in_vec;
    }

    void rotateVector(Eigen::Quaterniond &q, double x, double y, double z, Eigen::Vector3d &out_vec)
    {
        Eigen::Vector3d in_vec = Eigen::Vector3d(x, y, z);
        rotateVector(q, in_vec, out_vec);
    }

    void rotateVector(geometry_msgs::msg::Quaternion &q, double x, double y, double z, Eigen::Vector3d &out_vec)
    {
        Eigen::Quaterniond q1;
        q1.x() = q.x;
        q1.y() = q.y;
        q1.z() = q.z;
        q1.w() = q.w;
        rotateVector(q1, x, y, z, out_vec);
    }

    void test()
    {
        ros2::Point p1, p2, p3, p4;

        p1.x = 2.0;
        p1.y = 2.0;
        p2.x = 6.0;
        p2.y = 2.0;
        p3.x = 4.0;
        p3.y = 0.0;
        pointToLine(p1, p2, p3, p4);
        printf("TEST: [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f]\n",
               p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

        p1.x = 2.0;
        p1.y = 2.0;
        p2.x = 2.0;
        p2.y = 6.0;
        p3.x = 4.0;
        p3.y = 4.0;
        pointToLine(p1, p2, p3, p4);
        printf("TEST: [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f]\n",
               p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

        p1.x = 2.0;
        p1.y = 2.0;
        p2.x = 6.0;
        p2.y = 6.0;
        p3.x = 6.0;
        p3.y = 2.0;
        pointToLine(p1, p2, p3, p4);
        printf("TEST: [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f]\n",
               p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

        p1.x = 2.0;
        p1.y = 2.0;
        p2.x = -6.0;
        p2.y = -6.0;
        p3.x = 6.0;
        p3.y = 0.0;
        pointToLine(p1, p2, p3, p4);
        printf("TEST: [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f]\n",
               p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

        p1.x = 2.0;
        p1.y = 2.0;
        p2.x = -6.0;
        p2.y = -6.0;
        p3.x = 6.0;
        p3.y = 4.0;
        pointToLine(p1, p2, p3, p4);
        printf("TEST: [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f] [%0.2f, %0.2f]\n",
               p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

        Eigen::Vector3d v1 = {1.0, 0.0, 0.0};
        Eigen::Vector3d v2 = {3.0, 1.0, 0.0};
        Eigen::Vector3d vc1 = v1.cross(v2);
        Eigen::Vector3d vc2 = v2.cross(v1);
        printf("\nTEST cross product vc1{%0.3f, %0.3f, %0.3f), vc2{%0.3f, %0.3f, %0.3f)\n\n",
               vc1[0], vc1[1], vc1[2], vc2[0], vc2[1], vc2[2]);

        Eigen::Vector3d v1a = {2.0, 0.0, 0.0};
        Eigen::Vector3d v2a = {2.0, 2.0, 0.0};
        double a1 = angleBetweenVector(v1a, v2a);
        v2a = -v2a;
        double a2 = angleBetweenVector(v1a, v2a);
        printf("\nTEST ANGLE BETWEEN VECTORS  anglev1-v2[%0.2f] anglev2-v1[%0.2f]  v1[%0.3f, %0.3f, %0.3f], v2[%0.3f, %0.3f, %0.3f]\n\n",
               a1 * 180.0 / 3.14159, a2 * 180.0 / 3.14159, v1a[0], v1a[1], v1a[2], v2a[0], v2a[1], v2a[2]);

        Eigen::Vector3d A = {0.0, 0.0, 0.0};
        Eigen::Vector3d B = {0.0, 1.0, 0.0};
        Eigen::Vector3d P = {0.5, 0.5, 0.0};
        Eigen::Vector3d V;
        Eigen::Vector3d X;
        Eigen::Vector3d Y;
        bool on_segment, on_segment_2;

        vectorToLine(V, A, B, P);
        on_segment = pointOnLine(X, A, B, P);
        on_segment_2 = pointOnLineSegment(Y, A, B, P);
        printf("TEST POINTS: P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]\n", B[0], B[1], B[2]);
        printf("vectorToLine:        V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);
        printf("pointOnLine:         X[%0.2f %0.2f %0.2f] %s\n", X[0], X[1], X[2], on_segment ? "TRUE" : "FALSE");
        printf("pointOnLineSegment:  Y[%0.2f %0.2f %0.2f] %s\n", Y[0], Y[1], Y[2], on_segment_2 ? "TRUE" : "FALSE");

        A = {0.0, 0.0, 0.0};
        B = {0.0, 1.0, 0.0};
        P = {0.5, 1.5, 0.0};
        vectorToLine(V, A, B, P);
        on_segment = pointOnLine(X, A, B, P);
        on_segment_2 = pointOnLineSegment(Y, A, B, P);
        printf("TEST POINTS: P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]\n", B[0], B[1], B[2]);
        printf("vectorToLine:        V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);
        printf("pointOnLine:         X[%0.2f %0.2f %0.2f] %s\n", X[0], X[1], X[2], on_segment ? "TRUE" : "FALSE");
        printf("pointOnLineSegment:  Y[%0.2f %0.2f %0.2f] %s\n", Y[0], Y[1], Y[2], on_segment_2 ? "TRUE" : "FALSE");

        A = {0.0, 1.0, 0.0};
        B = {1.0, 0.0, 0.0};
        P = {1.0, 1.0, 0.0};
        vectorToLine(V, A, B, P);
        on_segment = pointOnLine(X, A, B, P);
        on_segment_2 = pointOnLineSegment(Y, A, B, P);
        printf("TEST POINTS: P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]\n", B[0], B[1], B[2]);
        printf("vectorToLine:        V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);
        printf("pointOnLine:         X[%0.2f %0.2f %0.2f] %s\n", X[0], X[1], X[2], on_segment ? "TRUE" : "FALSE");
        printf("pointOnLineSegment:  Y[%0.2f %0.2f %0.2f] %s\n", Y[0], Y[1], Y[2], on_segment_2 ? "TRUE" : "FALSE");

        A = {0.0, 0.0, 0.0};
        B = {1.0, 0.0, 0.0};
        P = {2.0, 0.0, 0.0};
        vectorToLine(V, A, B, P);
        on_segment = pointOnLine(X, A, B, P);
        on_segment_2 = pointOnLineSegment(Y, A, B, P);
        printf("TEST POINTS: P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]\n", B[0], B[1], B[2]);
        printf("vectorToLine:        V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);
        printf("pointOnLine:         X[%0.2f %0.2f %0.2f] %s\n", X[0], X[1], X[2], on_segment ? "TRUE" : "FALSE");
        printf("pointOnLineSegment:  Y[%0.2f %0.2f %0.2f] %s\n", Y[0], Y[1], Y[2], on_segment_2 ? "TRUE" : "FALSE");

        A = {0.0, 0.0, 0.0};
        B = {1.0, 0.0, 0.0};
        P = {-2.0, 1.0, 0.0};
        vectorToLine(V, A, B, P);
        on_segment = pointOnLine(X, A, B, P);
        on_segment_2 = pointOnLineSegment(Y, A, B, P);
        printf("TEST POINTS: P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]\n", B[0], B[1], B[2]);
        printf("vectorToLine:        V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);
        printf("pointOnLine:         X[%0.2f %0.2f %0.2f] %s\n", X[0], X[1], X[2], on_segment ? "TRUE" : "FALSE");
        printf("pointOnLineSegment:  Y[%0.2f %0.2f %0.2f] %s\n", Y[0], Y[1], Y[2], on_segment_2 ? "TRUE" : "FALSE");

        A = {2.0, 2.0, 0.0};
        B = {1.0, 0.0, 0.0};
        P = {1.0, 1.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        P = {4.0, 1.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        B = {0.0, -1.0, 0.0};
        P = {1.0, 1.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        P = {4.0, 4.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        B = {0.0, 1.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        P = {4.0, 2.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        B = {0.0, 0.0, -1.0};
        P = {1.0, 1.0, 1.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        P = {4.0, 1.0, 3.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        B = {1.0, 1.0, 0.0};
        P = {1.0, 1.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        P = {4.0, 1.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        B = {1.0, -1.0, 0.0};
        P = {1.0, 1.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        P = {4.0, 1.0, 0.0};
        vectorAlongVector(V, A, B, P);
        printf("vectorAlongVector:  ");
        printf("P[%0.2f %0.2f %0.2f]  ", P[0], P[1], P[2]);
        printf("A[%0.2f %0.2f %0.2f]  ", A[0], A[1], A[2]);
        printf("B[%0.2f %0.2f %0.2f]  ", B[0], B[1], B[2]);
        printf("V[%0.2f %0.2f %0.2f]\n", V[0], V[1], V[2]);

        testq(10.0, 15, 25);
        testq(-10.0, 15, -25);
        testq(-10.0, -15, 175);
        testq(-30.0, 15, -25);
        testq(-30.0, -15, 175);
        testq(-45.0, 15, -25);
        testq(-45.0, 0, -25);
        testq(-45.0, 15, 0);
        testq(-45.0, 0, 25);
        testq(-45.0, -15, 0);
        testq(-45.0, -15, 175);
        testq(-89.0, 15, -25);
        testq(-89.0, -15, 175);
        testq(-90.0, 15, -25);
        testq(-90.0, -15, 175);
        testq(-91.0, 15, -25);
        testq(-91.0, -15, 175);

        testq(0.0, 0.0, 45.0);
        testq(0.0, 0.0, -45.0);
        testq(0.0, 0.0, 90.0);
        testq(0.0, 0.0, -90.0);
        testq(0.0, 0.0, 135.0);
        testq(0.0, 0.0, -135.0);
        testq(0.0, 0.0, 175.0);
        testq(0.0, 0.0, -175.0);
        testq(0.0, 0.0, 180.0);
        testq(0.0, 0.0, -180.0);

        printf("======== ROTATE POINT TEST ========\n");
        Eigen::Vector3d pt_org, pt1_new, pt2_new;
        double rot_rad;
        pt_org[0] = 2.0;
        pt_org[1] = 3.5;
        pt_org[2] = 5.3;

        rot_rad = 3.14159 / 4.0;
        rotatePoint(pt_org[0], pt_org[1], pt1_new[0], pt1_new[1], rot_rad);
        transformXYFrame(pt_org, rot_rad, pt2_new);
        printf("  rotatePoint: rot[%0.1f]  org[%0.3f %0.3f %0.3f ] new[%0.3f %0.3f %0.3f ]\n", rad2deg(rot_rad),
               pt_org[0], pt_org[1], pt_org[2],
               pt1_new[0], pt1_new[1], pt1_new[2]);
        printf("  transformXYFrame: rot[%0.1f]  org[%0.3f %0.3f %0.3f ] new[%0.3f %0.3f %0.3f ]\n\n", rad2deg(rot_rad),
               pt_org[0], pt_org[1], pt_org[2],
               pt2_new[0], pt2_new[1], pt2_new[2]);

        rot_rad = -3.14159 * (3.0 / 4.0);
        rotatePoint(pt_org[0], pt_org[1], pt1_new[0], pt1_new[1], rot_rad);
        transformXYFrame(pt_org, rot_rad, pt2_new);
        printf("  rotatePoint: rot[%0.1f]  org[%0.3f %0.3f %0.3f ] new[%0.3f %0.3f %0.3f ]\n", rad2deg(rot_rad),
               pt_org[0], pt_org[1], pt_org[2],
               pt1_new[0], pt1_new[1], pt1_new[2]);
        printf("  transformXYFrame: rot[%0.1f]  org[%0.3f %0.3f %0.3f ] new[%0.3f %0.3f %0.3f ]\n\n", rad2deg(rot_rad),
               pt_org[0], pt_org[1], pt_org[2],
               pt2_new[0], pt2_new[1], pt2_new[2]);

        pt_org[0] = 0.791148;
        pt_org[1] = -0.551095;
        pt_org[2] = 0.276293;
        rot_rad = 3.14159 * (-165.0 / 180.0);
        rotatePoint(pt_org[0], pt_org[1], pt1_new[0], pt1_new[1], rot_rad);
        transformXYFrame(pt_org, rot_rad, pt2_new);
        printf("  rotatePoint: rot[%0.1f]  org[%0.3f %0.3f %0.3f ] new[%0.3f %0.3f %0.3f ]\n", rad2deg(rot_rad),
               pt_org[0], pt_org[1], pt_org[2],
               pt1_new[0], pt1_new[1], pt1_new[2]);
        printf("  transformXYFrame: rot[%0.1f]  org[%0.3f %0.3f %0.3f ] new[%0.3f %0.3f %0.3f ]\n\n", rad2deg(rot_rad),
               pt_org[0], pt_org[1], pt_org[2],
               pt2_new[0], pt2_new[1], pt2_new[2]);

        rot_rad = 3.14159 * (165.0 / 180.0);
        rotatePoint(pt_org[0], pt_org[1], pt1_new[0], pt1_new[1], rot_rad);
        transformXYFrame(pt_org, rot_rad, pt2_new);
        printf("  rotatePoint: rot[%0.1f]  org[%0.3f %0.3f %0.3f ] new[%0.3f %0.3f %0.3f ]\n", rad2deg(rot_rad),
               pt_org[0], pt_org[1], pt_org[2],
               pt1_new[0], pt1_new[1], pt1_new[2]);
        printf("  transformXYFrame: rot[%0.1f]  org[%0.3f %0.3f %0.3f ] new[%0.3f %0.3f %0.3f ]\n\n", rad2deg(rot_rad),
               pt_org[0], pt_org[1], pt_org[2],
               pt2_new[0], pt2_new[1], pt2_new[2]);
    }

    void printPoint(Eigen::Vector3d &p)
    {
        printf(" x[%0.2f] y[%0.2f] z[%0.2f] ", p[0], p[1], p[2]);
    }

    // P - point
    // D - direction of line (unit length)
    // A - point in line
    //
    // X - base of the perpendicular line
    //
    //     P
    //    /|
    //   / |
    //  /  v
    // A---X----->D
    //
    // (P-A).D == |X-A|
    //
    // X == A + ((P-A).D)D
    // Desired perpendicular: X-P
    //
    void vectorToVector(Eigen::Vector3d &V, Eigen::Vector3d &A, Eigen::Vector3d &dir, Eigen::Vector3d &P)
    {
        Eigen::Vector3d D = dir;
        D.normalize();

        Eigen::Vector3d PA = P - A;

        Eigen::Vector3d X = A + PA.dot(D) * D;

        V = X - P;
    }

    void vectorToLine(Eigen::Vector3d &V, Eigen::Vector3d &A, Eigen::Vector3d &B, Eigen::Vector3d &P)
    {
        Eigen::Vector3d dir = B - A;
        vectorToVector(V, A, dir, P);
    }

    bool pointOnLine(Eigen::Vector3d &X, Eigen::Vector3d &A, Eigen::Vector3d &B, Eigen::Vector3d &P)
    {
        Eigen::Vector3d dir = B - A;
        Eigen::Vector3d D = dir;
        D.normalize();

        Eigen::Vector3d PA = P - A;

        X = A + PA.dot(D) * D;

        // Check to see if X is on the line segment
        if ((X[0] < A[0] && X[0] < B[0]) ||
            (X[0] > A[0] && X[0] > B[0]) ||
            (X[1] < A[1] && X[1] < B[1]) ||
            (X[1] > A[1] && X[1] > B[1]) ||
            (X[2] < A[2] && X[2] < B[2]) ||
            (X[2] > A[2] && X[2] > B[2]))
        {
            return false;
        }

        return true;
    }

    bool pointOnLine(geometry_msgs::msg::Point &point, geometry_msgs::msg::Point &lp1,
                     geometry_msgs::msg::Point &lp2, geometry_msgs::msg::Point &pt)
    {
        Eigen::Vector3d X, A, B, P;
        Point2Eigen(lp1, A);
        Point2Eigen(lp2, B);
        Point2Eigen(pt, P);

        bool ret_val = pointOnLine(X, A, B, P);

        Eigen2Point(X, point);

        return ret_val;
    }

    std::string getUTMZone(double latitude, double longitude)
    {
        std::string zone_str;

        int zone_number = int((longitude + 180) / 6) + 1;
        char hemisphere = (latitude >= 0) ? 'N' : 'S';
        zone_str = std::to_string(zone_number) + hemisphere;

        return zone_str;
    }

    /**
     * @brief Converts latitude and longitude to UTM coordinates.
     * @param latitude The latitude in decimal degrees.
     * @param longitude The longitude in decimal degrees.
     * @param utm_easting The UTM easting coordinate (output).
     * @param utm_northing The UTM northing coordinate (output).
     * @param utm_zone The UTM zone (output).
     */
    void latLon_to_UTM(double latitude, double longitude, double &utm_easting, double &utm_northing, std::string &utm_zone)
    {
        int zone;
        bool northp;
        try
        {
            GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_easting, utm_northing);
        }
        catch (...)
        {
            return;
        }
        utm_zone = std::to_string(zone) + (northp ? 'N' : 'S');
    }

    bool utmToLatLon(double utm_easting, double utm_northing, const std::string &utm_zone,
                     double &latitude, double &longitude)
    {
        if (utm_zone.empty())
        {
            return false;
        }

        //
        // 1. Parse UTM zone ("10N", "33T", "14S", or just "10")
        //
        size_t i = 0;
        while (i < utm_zone.size() && std::isdigit(static_cast<unsigned char>(utm_zone[i])))
        {
            ++i;
        }

        int zone_number = std::stoi(utm_zone.substr(0, i));

        bool northern = true; // default
        if (i < utm_zone.size())
        {
            char letter = std::toupper(utm_zone[i]);
            northern = (letter >= 'N'); // C–M = south, N–X = north
        }

        //
        // 2. WGS84 parameters
        //
        const double a = 6378137.0;
        const double eccSquared = 0.00669437999014;
        const double k0 = 0.9996;

        //
        // 3. Remove false Easting/Northing
        //
        double x = utm_easting - 500000.0;
        double y = utm_northing;

        if (!northern)
            y -= 10000000.0; // 10,000,000 m offset for southern hemisphere

        //
        // 4. Compute footpoint latitude
        //
        double eccPrimeSquared = eccSquared / (1.0 - eccSquared);

        double M = y / k0;
        double mu = M / (a * (1.0 - eccSquared / 4.0 -
                              3.0 * std::pow(eccSquared, 2) / 64.0 -
                              5.0 * std::pow(eccSquared, 3) / 256.0));

        double e1 = (1.0 - std::sqrt(1.0 - eccSquared)) /
                    (1.0 + std::sqrt(1.0 - eccSquared));

        double J1 = (3.0 * e1 / 2.0 - 27.0 * std::pow(e1, 3) / 32.0);
        double J2 = (21.0 * std::pow(e1, 2) / 16.0 - 55.0 * std::pow(e1, 4) / 32.0);
        double J3 = (151.0 * std::pow(e1, 3) / 96.0);
        double J4 = (1097.0 * std::pow(e1, 4) / 512.0);

        double phi1 = mu + J1 * std::sin(2.0 * mu) + J2 * std::sin(4.0 * mu) + J3 * std::sin(6.0 * mu) + J4 * std::sin(8.0 * mu);

        //
        // 5. Compute latitude and longitude
        //
        double sinPhi1 = std::sin(phi1);
        double cosPhi1 = std::cos(phi1);

        double N1 = a / std::sqrt(1.0 - eccSquared * sinPhi1 * sinPhi1);
        double T1 = std::tan(phi1) * std::tan(phi1);
        double C1 = eccPrimeSquared * cosPhi1 * cosPhi1;
        double R1 = a * (1.0 - eccSquared) /
                    std::pow(1.0 - eccSquared * sinPhi1 * sinPhi1, 1.5);

        double D = x / (N1 * k0);

        // Latitude (radians)
        double lat_rad =
            phi1 - (N1 * std::tan(phi1) / R1) *
                       (D * D / 2.0 - (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * eccPrimeSquared) * std::pow(D, 4) / 24.0 + (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 - 252.0 * eccPrimeSquared - 3.0 * C1 * C1) * std::pow(D, 6) / 720.0);

        // Longitude relative to central meridian (radians)
        double lon_rad =
            (D - (1.0 + 2.0 * T1 + C1) * std::pow(D, 3) / 6.0 + (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * eccPrimeSquared + 24.0 * T1 * T1) * std::pow(D, 5) / 120.0) / cosPhi1;

        //
        // 6. Add central meridian in radians
        //
        double lon0_deg = (zone_number - 1) * 6.0 - 180.0 + 3.0; // degrees
        double lon0_rad = lon0_deg * M_PI / 180.0;

        double lon_final_rad = lon0_rad + lon_rad;

        //
        // 7. Convert to degrees
        //
        latitude = lat_rad * 180.0 / M_PI;
        longitude = lon_final_rad * 180.0 / M_PI;

        return true;
    }

    bool pointOnLineSegment(Eigen::Vector3d &X, Eigen::Vector3d &A, Eigen::Vector3d &B, Eigen::Vector3d &P)
    {
        Eigen::Vector3d dir = B - A;
        Eigen::Vector3d D = dir;
        D.normalize();

        Eigen::Vector3d PA = P - A;

        X = A + PA.dot(D) * D;

        Eigen::Vector3d min, max;
        if (A[0] > B[0])
        {
            min[0] = B[0];
            max[0] = A[0];
        }
        else
        {
            min[0] = A[0];
            max[0] = B[0];
        }

        if (A[1] > B[1])
        {
            min[1] = B[1];
            max[1] = A[1];
        }
        else
        {
            min[1] = A[1];
            max[1] = B[1];
        }

        if (A[2] > B[2])
        {
            min[2] = B[2];
            max[2] = A[2];
        }
        else
        {
            min[2] = A[2];
            max[2] = B[2];
        }

        bool on_segment = true;
        if (X[0] < min[0])
        {
            on_segment = false;
            X[0] = min[0];
        }
        else if (X[0] > max[0])
        {
            on_segment = false;
            X[0] = max[0];
        }

        if (X[1] < min[1])
        {
            on_segment = false;
            X[1] = min[1];
        }
        else if (X[1] > max[1])
        {
            on_segment = false;
            X[1] = max[1];
        }

        if (X[2] < min[2])
        {
            on_segment = false;
            X[2] = min[2];
        }
        else if (X[2] > max[2])
        {
            on_segment = false;
            X[2] = max[2];
        }

        return on_segment;
    }

    bool pointOnLineSegment(geometry_msgs::msg::Point &point, geometry_msgs::msg::Point &lp1,
                            geometry_msgs::msg::Point &lp2, geometry_msgs::msg::Point &pt)
    {
        Eigen::Vector3d X, A, B, P;
        Point2Eigen(lp1, A);
        Point2Eigen(lp2, B);
        Point2Eigen(pt, P);

        bool ret_val = pointOnLine(X, A, B, P);

        Eigen2Point(X, point);

        return ret_val;
    }

    void vectorAlongVector(Eigen::Vector3d &vector, Eigen::Vector3d &lp1, Eigen::Vector3d &dir, Eigen::Vector3d &pt)
    {
        // First find the vector between lp1 and pt
        Eigen::Vector3d dest = lp1 - pt;
        Eigen::Vector3d D = dir;
        D.normalize();

        // Take the dot product to get the vector length along the direction vector
        double dist = dest.dot(D);
        //    printf("   XXX dist[%0.2f], dest[%0.1f, %0.1f, %0.1f] D[%0.1f, %0.1f, %0.1f]\n",
        //           dist, dest[0], dest[1], dest[2], D[0], D[1], D[2]);

        // Now apply the distance along the direction vector.
        vector = D * dist;
    }

    void transformXYFrame(Eigen::Vector3d &org_vec, double diff_R, Eigen::Vector3d &new_vec)
    {
        // get the angle of the existing vector in the existing frame
        double length = sqrt((org_vec[0] * org_vec[0]) + (org_vec[1] * org_vec[1]));
        double org_ang = atan2(org_vec[1], org_vec[0]);
        double new_ang = org_ang + diff_R;
        new_vec[2] = org_vec[2];
        new_vec[0] = length * cos(new_ang);
        new_vec[1] = length * sin(new_ang);
    }

    void transformXYFrame(geometry_msgs::msg::Vector3 &org_vec, double diff_R, geometry_msgs::msg::Vector3 &new_vec)
    {
        // get the angle of the existing vector in the existing frame
        double length = sqrt((org_vec.x * org_vec.x) + (org_vec.y * org_vec.y));
        double org_ang = atan2(org_vec.y, org_vec.x);
        double new_ang = org_ang + diff_R;
        new_vec.z = org_vec.z;
        new_vec.x = length * cos(new_ang);
        new_vec.y = length * sin(new_ang);
    }

    double transformXYFrame(Eigen::Vector3d &org_vec, geometry_msgs::msg::Quaternion &orientation,
                            Eigen::Vector3d &new_vec, bool reverse_rotation)
    {
        Eigen::Quaterniond q;
        q.x() = orientation.x;
        q.y() = orientation.y;
        q.z() = orientation.z;
        q.w() = orientation.w;

        return transformXYFrame(org_vec, q, new_vec, reverse_rotation);
    }

    double transformXYFrame(Eigen::Vector3d &org_vec, Eigen::Quaterniond &orientation,
                            Eigen::Vector3d &new_vec, bool reverse_rotation)
    {

        Eigen::Vector3d rpy = quaternion_to_RPY(orientation);

        double yaw;

        if (reverse_rotation)
        {
            yaw = -rpy[2];
        }
        else
        {
            yaw = rpy[2];
        }

        // Do the translation
        ros2::transformXYFrame(org_vec, yaw, new_vec);

        return yaw;
    }

    void levelOrientation(const geometry_msgs::msg::Quaternion &org_orientation, geometry_msgs::msg::Quaternion &level_orientation)
    {
        Eigen::Quaterniond q;
        q.x() = org_orientation.x;
        q.y() = org_orientation.y;
        q.z() = org_orientation.z;
        q.w() = org_orientation.w;
        Eigen::Vector3d rpy = quaternion_to_RPY(q);

        RPY_to_Quaternion(level_orientation, 0.0, 0.0, rpy[2]);

        return;
    }

    void levelOrientation(const Eigen::Quaterniond &org_orientation, Eigen::Quaterniond &level_orientation)
    {
        Eigen::Vector3d rpy = quaternion_to_RPY(org_orientation);
        RPY_to_Quaternion(level_orientation, 0.0, 0.0, rpy[2]);

        return;
    }

    void printAttitude(Eigen::Vector3d &a)
    {
        printf(" roll[%0.2f] pitch[%0.2f] yaw[%0.2f] ", a[0], a[1], a[2]);
    }

    void printRotation(Eigen::Quaterniond &a)
    {
        printf(" x[%0.2f] y[%0.2f] z[%0.2f] w[%0.2f] ", a.x(), a.y(), a.z(), a.w());
    }

    // void RPY_to_Quaternion(Eigen::Quaterniond& q, double roll, double pitch, double yaw, bool isDegree = false);
    // void RPY_to_Quaternion(geometry_msgs::Quaternion& q, double roll, double pitch, double yaw, bool isDegree = false);
    //
    //// takes a quaternion and converts it to RPY
    // Eigen::Vector3d quaternion_to_RPY(const Eigen::Quaterniond& q);
    // Eigen::Vector3d quaternion_to_RPY(const geometry_msgs::Quaternion& q);

#define SAME(x, y) (fabs(x - y) < 0.001)
    void testQuats()
    {
        Eigen::Quaterniond q1, q2;
        double R, P, Y;
        Eigen::Vector3d rpy;
        for (R = -180.0; R <= 180.0; R += 30.0)
            for (P = -180.0; P <= 180.0; P += 30.0)
                for (Y = -180.0; Y <= 180.0; Y += 30.0)
                {
                    RPY_to_Quaternion(q1, R, P, Y, true);
                    rpy = quaternion_to_RPY(q1);
                    rpy[0] = rad2deg(rpy[0]);
                    rpy[1] = rad2deg(rpy[1]);
                    rpy[2] = rad2deg(rpy[2]);
                    printf("%0.1f  %0.1f  %0.1f  :  %0.3f  %0.3f  %0.3f  %0.3f", R, P, Y, q1.x(), q1.y(), q1.z(), q1.w());
                    if (!SAME(rpy[0], R) || !SAME(rpy[1], P) || !SAME(rpy[2], Y))
                    {
                        printf("   ERROR: %0.3f, %0.3f, %0.3f", rpy[0], rpy[1], rpy[2]);
                    }

                    printf("\n");
                }
    }

}