 #include <autonomy_utils/lib/rotate.h>

 namespace ros2
 {
   void Rotate::setRotationMatrixComponents()
   {
     /*
     //Old R
     R[0] = YAW.c * PITCH.c;
     R[1] = YAW.c * PITCH.s * ROLL.s - YAW.s * ROLL.c;
     R[2] = YAW.s * ROLL.s + YAW.c * PITCH.s * ROLL.c;
 
     R[3] = YAW.s * PITCH.c;
     R[4] = YAW.s * PITCH.s * ROLL.s + YAW.c * ROLL.c;
     R[5] = YAW.s * PITCH.s * ROLL.c - YAW.c * ROLL.s;
 
     R[6] = PITCH.s;
     R[7] = PITCH.c * ROLL.s;
     R[8] = PITCH.c * ROLL.c;*/
 
     //New R
     R[0] = YAW.c * PITCH.c;
     R[1] = -1 * PITCH.c * YAW.s;
     R[2] = PITCH.s;
 
     R[3] = ROLL.s * PITCH.s * YAW.c + ROLL.c * YAW.s;
     R[4] = ROLL.c * YAW.c - ROLL.s * PITCH.s * YAW.s;
     R[5] = -1 * ROLL.s * PITCH.c;
 
     R[6] = ROLL.s * YAW.s - ROLL.c * PITCH.s * YAW.c;
     R[7] = ROLL.c * PITCH.s * YAW.s + ROLL.s * YAW.c;
     R[8] = ROLL.c * PITCH.c;
 
     processR();
 
   }
   void Rotate::processR()
   {
     //Finding determinant of the R matrix
     R_det = R[0] * (R[4]*R[8] - R[5]*R[7]) - R[1] * (R[3]*R[8] - R[5]*R[6]) + R[2] * (R[3]*R[7] - R[4]*R[6]);
 
     //Finding Transpose of R
     R_t[0] = R[0];
     R_t[1] = R[3];
     R_t[2] = R[6];
     R_t[3] = R[1];
     R_t[4] = R[4];
     R_t[5] = R[7];
     R_t[6] = R[2];
     R_t[7] = R[5];
     R_t[8] = R[8];
 
     //Finding Adjugate Matrix of R
     R_adj[0] = R[4]*R[8] - R[7]*R[5];
     R_adj[1] = -1*(R[1]*R[8] - R[2]*R[7]);
     R_adj[2] = R[1]*R[5] - R[2]*R[4];
     R_adj[3] = -1*(R[3]*R[8] - R[5]*R[6]);
     R_adj[4] = R[0]*R[8] - R[2]*R[6];
     R_adj[5] = -1*(R[0]*R[5] - R[2]*R[3]);
     R_adj[6] = R[3]*R[7] - R[4]*R[6];
     R_adj[7] = -1*(R[0]*R[7] - R[1]*R[6]);
     R_adj[8] = R[0]*R[4] - R[1]*R[3];
 
     //Finding Members of the inverse matrix
     for(int i = 0; i < 9; i++)
     {
       R_inv[i] = R_adj[i]/R_det;
     }
   }
   Rotate::Rotate(double roll, double pitch, double yaw)
   {
     ROLL = getEuler(roll);
     PITCH = getEuler(pitch);
     YAW = getEuler(yaw);
     //ROS_INFO(GREEN "%f, %f, %f" COLOR_RESET, ROLL.angle, PITCH.angle, YAW.angle);
     setRotationMatrixComponents();
   }
   Rotate::Rotate()
   {
   }
   Rotate::Rotate(double x, double y, double z, double w)
   {
     //getting the imu converted:
     double roll, pitch, yaw;
     tf2::Quaternion q(x, y, z, w);
     tf2::Matrix3x3 m(q);
     m.getRPY(roll, pitch, yaw);
     ROLL = getEuler(roll);
     PITCH = getEuler(pitch);
     YAW = getEuler(yaw);
     //ROS_INFO(GREEN "%f, %f, %f" COLOR_RESET, ROLL.angle, PITCH.angle, YAW.angle);
     setRotationMatrixComponents();
   }
   Rotate::Rotate(geometry_msgs::msg::Quaternion &q)
   {
     //getting the imu converted:
     double roll, pitch, yaw;
     tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
     tf2::Matrix3x3 m(tfq);
     m.getRPY(roll, pitch, yaw);
     ROLL = getEuler(roll);
     PITCH = getEuler(pitch);
     YAW = getEuler(yaw);
     //ROS_INFO(GREEN "%f, %f, %f" COLOR_RESET, ROLL.angle, PITCH.angle, YAW.angle);
     setRotationMatrixComponents();
   }
   Rotate::Rotate(tf2::Quaternion &q)
   {
     //getting the imu converted:
     double roll, pitch, yaw;
     tf2::Matrix3x3 m(q);
     m.getRPY(roll, pitch, yaw);
     ROLL = getEuler(roll);
     PITCH = getEuler(pitch);
     YAW = getEuler(yaw);
     //ROS_INFO(GREEN "%f, %f, %f" COLOR_RESET, ROLL.angle, PITCH.angle, YAW.angle);
     setRotationMatrixComponents();
   }
   EULER Rotate::getEuler(double ang)
   {
     EULER e;
     e.angle = ang;
     e.s = sin(ang);
     e.c = cos(ang);
     return e;
   }
   void Rotate::setRPY(double roll, double pitch, double yaw)
   {
     ROLL = getEuler(roll);
     PITCH = getEuler(pitch);
     YAW = getEuler(yaw);
     setRotationMatrixComponents();
   }
 
   void Rotate::rotateInv(float &x, float &y, float &z)
   {
     float tx = x;
     float ty = y;
     float tz = z;
 
     x = tx * R_inv[0] + ty * R_inv[1] + tz * R_inv[2];
     y = tx * R_inv[3] + ty * R_inv[4] + tz * R_inv[5];
     z = tx * R_inv[6] + ty * R_inv[7] + tz * R_inv[8];
   }
   void Rotate::rotateTranspose(float &x, float &y, float &z)
   {
     float tx = x;
     float ty = y;
     float tz = z;
 
     x = tx * R_t[0] + ty * R_t[1] + tz * R_t[2];
     y = tx * R_t[3] + ty * R_t[4] + tz * R_t[5];
     z = tx * R_t[6] + ty * R_t[7] + tz * R_t[8];
   }
   /*
                             ROTATIOTION AROUND ALL THREE AXISES WITH PREDEFINED ANGLES.
                             BEAR IN MIND THAT THE ROTATION MATRIX IS IN ORDER OF ZYX.
                             THIS IS THE EULER ORDER AND IF YOU MULTIPLY THE MATRICES IN DIFFERENT ORDERS
                             YOU WILL GET DIFFERENT RESULTS. THIS IS THE CORRECT ONE.
    */
   void Rotate::rotate(float &x, float &y, float &z)
   {
 
     float tx = x;
     float ty = y;
     float tz = z;
 
     x = tx * R[0] + ty * R[1] + tz * R[2];
     y = tx * R[3] + ty * R[4] + tz * R[5];
     z = tx * R[6] + ty * R[7] + tz * R[8];
 
     //The original 3D Rotation matrix, the above method is faster since it requires less multiplication.
     //                        x = tx * YAW.c * PITCH.c + ty * (YAW.c * PITCH.s * ROLL.s - YAW.s * ROLL.c) + tz * (YAW.s * ROLL.s + YAW.c * PITCH.s * ROLL.c);
     //                        y = tx * YAW.s * PITCH.c + ty * (YAW.s * PITCH.s * ROLL.s + YAW.c * ROLL.c) + tz * (YAW.s * PITCH.s * ROLL.c - YAW.c * ROLL.s);
     //                        z = -tx * PITCH.s + ty * (PITCH.c * ROLL.s) + tz * (PITCH.c * ROLL.c);
   }
 
   //ROTATE AROUND ONLY ONE AXIS WITH NEW ANGLE.
   void Rotate::rotate(float &x, float &y, float &z, double angle, char axis)
   {
     float t_x = x;
     float t_y = y;
     float t_z = z;
 
     double s = sin(angle);
     double c = cos(angle);
     //Switch-case is faster than if-else
     switch(axis)
     {
       case 'x':
         x = t_x;
         y = t_y * c - t_z * s;
         z = t_y * s + t_z * c;
         break;
       case 'X':
         x = t_x;
         y = t_y * c - t_z * s;
         z = t_y * s + t_z * c;
         break;
       case 'y':
         x = t_x * c + t_z * s;
         y = t_y;
         z = -t_x * s + t_z * c;
         break;
       case 'Y':
         x = t_x * c + t_z * s;
         y = t_y;
         z = -t_x * s + t_z * c;
         break;
       case 'z':
         x = t_x * c - t_y * s;
         y = t_x * s + t_y * c;
         z = t_z;
         break;
       case 'Z':
         x = t_x * c - t_y * s;
         y = t_x * s + t_y * c;
         z = t_z;
         break;
 
     }
   }
   //ROTATE AROUND ONLY ONE AXIS WITH PREDEFINED ANGLE -> IT IS SLIGHTLY FASTER THAN CONSIDERING ALL THREE.
   void Rotate::rotate(float &x, float &y, float &z, char axis)
   {
     float t_x = x;
     float t_y = y;
     float t_z = z;
 
     //Switch-case is faster than if-else
     switch(axis)
     {
       case 'x':
         x = t_x;
         y = t_y * ROLL.c - t_z * ROLL.s;
         z = t_y * ROLL.s + t_z * ROLL.c;
         break;
       case 'X':
         x = t_x;
         y = t_y * ROLL.c - t_z * ROLL.s;
         z = t_y * ROLL.s + t_z * ROLL.c;
         break;
       case 'y':
         x = t_x * PITCH.c + t_z * PITCH.s;
         y = t_y;
         z = -t_x * PITCH.s + t_z * PITCH.c;
         break;
       case 'Y':
         x = t_x * PITCH.c + t_z * PITCH.s;
         y = t_y;
         z = -t_x * PITCH.s + t_z * PITCH.c;
         break;
       case 'z':
         x = t_x * YAW.c - t_y * YAW.s;
         y = t_x * YAW.s + t_y * YAW.c;
         z = t_z;
         break;
       case 'Z':
         x = t_x * YAW.c - t_y * YAW.s;
         y = t_x * YAW.s + t_y * YAW.c;
         z = t_z;
         break;
 
     }
 
   }
   void Rotate::getRPY(double &roll, double &pitch, double &yaw)
   {
     roll = ROLL.angle;
     pitch = PITCH.angle;
     yaw = YAW.angle;
   }
   void Rotate::rotate(double &x, double &y, double &z, float roll, float pitch, float yaw)
   {
       EULER ROLL = Rotate::getEuler(roll);
       EULER PITCH = Rotate::getEuler(pitch);
       EULER YAW = Rotate::getEuler(yaw);
 
       float t_x = x;
       float t_y = y;
       float t_z = z;
 
       //ROTATE Around Roll
       x = t_x;
       y = t_y * ROLL.c - t_z * ROLL.s;
       z = t_y * ROLL.s + t_z * ROLL.c;
 
       t_x = x;
       t_y = y;
       t_z = z;
 
 
       //ROTATE Around Pitch
       x = t_x * PITCH.c + t_z * PITCH.s;
       y = t_y;
       z = -t_x * PITCH.s + t_z * PITCH.c;
 
       t_x = x;
       t_y = y;
       t_z = z;
 
 
       //ROTATE Around Yaw
       x = t_x * YAW.c - t_y * YAW.s;
       y = t_x * YAW.s + t_y * YAW.c;
       z = t_z;
 
 
   }
   //Static method to convert quaternion by component to rpy by component
   void Rotate::getRPY(double x, double y, double z, double w, double &roll, double &pitch, double &yaw)
   {
     //getting the imu converted:
     tf2::Quaternion q(x, y, z, w);
     tf2::Matrix3x3 m(q);
     m.getRPY(roll, pitch, yaw);
   }
 
   //Static method to convert quaternion to rpy
   void Rotate::getRPY(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw)
   {
     //getting the imu converted:
     tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
     tf2::Matrix3x3 m(tfq);
     m.getRPY(roll, pitch, yaw);
   }
   //Static method to convert quaternion to rpy
   void Rotate::getRPY(tf2::Quaternion &q, double &roll, double &pitch, double &yaw)
   {
     tf2::Matrix3x3 m(q);
     m.getRPY(roll, pitch, yaw);
   }
 
   geometry_msgs::msg::Vector3 Rotate::getRPY(const geometry_msgs::msg::Pose &pose)
   {
       tf2::Quaternion tf2_q;
       tf2::fromMsg(pose.orientation, tf2_q);
       geometry_msgs::msg::Vector3 rpy;
 
       tf2::Matrix3x3(tf2_q).getRPY(rpy.x, rpy.y, rpy.z);
       return rpy;
   }
 
   void Rotate::rotate(double &x, double &y, double &z, geometry_msgs::msg::Quaternion &q_rot)
   {
       double old_x = x;
       double old_y = y;
       double old_z = z;
 
       x = q_rot.w*q_rot.w*old_x + 2*q_rot.y*q_rot.w*old_z - 2*q_rot.z*q_rot.w*old_y + q_rot.x*q_rot.x*old_x + 2*q_rot.y*q_rot.x*old_y + 2*q_rot.z*q_rot.x*old_z - q_rot.z*q_rot.z*old_x - q_rot.y*q_rot.y*old_x;
       y = 2*q_rot.x*q_rot.y*old_x + q_rot.y*q_rot.y*old_y + 2*q_rot.z*q_rot.y*old_z + 2*q_rot.w*q_rot.z*old_x - q_rot.z*q_rot.z*old_y + q_rot.w*q_rot.w*old_y - 2*q_rot.x*q_rot.w*old_z - q_rot.x*q_rot.x*old_y;
       z = 2*q_rot.x*q_rot.z*old_x + 2*q_rot.y*q_rot.z*old_y + q_rot.z*q_rot.z*old_z - 2*q_rot.w*q_rot.y*old_x - q_rot.y*q_rot.y*old_z + 2*q_rot.w*q_rot.x*old_y - q_rot.x*q_rot.x*old_z + q_rot.w*q_rot.w*old_z;
 
   }
 
   void Rotate::rotate(double &x, double &y, double &z, const geometry_msgs::msg::Quaternion &q_rot)
   {
       double old_x = x;
       double old_y = y;
       double old_z = z;
 
       x = q_rot.w*q_rot.w*old_x + 2*q_rot.y*q_rot.w*old_z - 2*q_rot.z*q_rot.w*old_y + q_rot.x*q_rot.x*old_x + 2*q_rot.y*q_rot.x*old_y + 2*q_rot.z*q_rot.x*old_z - q_rot.z*q_rot.z*old_x - q_rot.y*q_rot.y*old_x;
       y = 2*q_rot.x*q_rot.y*old_x + q_rot.y*q_rot.y*old_y + 2*q_rot.z*q_rot.y*old_z + 2*q_rot.w*q_rot.z*old_x - q_rot.z*q_rot.z*old_y + q_rot.w*q_rot.w*old_y - 2*q_rot.x*q_rot.w*old_z - q_rot.x*q_rot.x*old_y;
       z = 2*q_rot.x*q_rot.z*old_x + 2*q_rot.y*q_rot.z*old_y + q_rot.z*q_rot.z*old_z - 2*q_rot.w*q_rot.y*old_x - q_rot.y*q_rot.y*old_z + 2*q_rot.w*q_rot.x*old_y - q_rot.x*q_rot.x*old_z + q_rot.w*q_rot.w*old_z;
 
   }
 
   void Rotate::rotate(geometry_msgs::msg::Point32 &p, const geometry_msgs::msg::Quaternion &q_rot)
   {
       geometry_msgs::msg::Point32 old_p = p;
       p.x = q_rot.w*q_rot.w*old_p.x + 2*q_rot.y*q_rot.w*old_p.z - 2*q_rot.z*q_rot.w*old_p.y + q_rot.x*q_rot.x*old_p.x + 2*q_rot.y*q_rot.x*old_p.y + 2*q_rot.z*q_rot.x*old_p.z - q_rot.z*q_rot.z*old_p.x - q_rot.y*q_rot.y*old_p.x;
       p.y = 2*q_rot.x*q_rot.y*old_p.x + q_rot.y*q_rot.y*old_p.y + 2*q_rot.z*q_rot.y*old_p.z + 2*q_rot.w*q_rot.z*old_p.x - q_rot.z*q_rot.z*old_p.y + q_rot.w*q_rot.w*old_p.y - 2*q_rot.x*q_rot.w*old_p.z - q_rot.x*q_rot.x*old_p.y;
       p.z = 2*q_rot.x*q_rot.z*old_p.x + 2*q_rot.y*q_rot.z*old_p.y + q_rot.z*q_rot.z*old_p.z - 2*q_rot.w*q_rot.y*old_p.x - q_rot.y*q_rot.y*old_p.z + 2*q_rot.w*q_rot.x*old_p.y - q_rot.x*q_rot.x*old_p.z + q_rot.w*q_rot.w*old_p.z;
 
   }
 
   void Rotate::rotate(pcl::PointXYZIR &p, const geometry_msgs::msg::Quaternion &q_rot)
   {
       pcl::PointXYZIR old_p = p;
       p.x = q_rot.w*q_rot.w*old_p.x + 2*q_rot.y*q_rot.w*old_p.z - 2*q_rot.z*q_rot.w*old_p.y + q_rot.x*q_rot.x*old_p.x + 2*q_rot.y*q_rot.x*old_p.y + 2*q_rot.z*q_rot.x*old_p.z - q_rot.z*q_rot.z*old_p.x - q_rot.y*q_rot.y*old_p.x;
       p.y = 2*q_rot.x*q_rot.y*old_p.x + q_rot.y*q_rot.y*old_p.y + 2*q_rot.z*q_rot.y*old_p.z + 2*q_rot.w*q_rot.z*old_p.x - q_rot.z*q_rot.z*old_p.y + q_rot.w*q_rot.w*old_p.y - 2*q_rot.x*q_rot.w*old_p.z - q_rot.x*q_rot.x*old_p.y;
       p.z = 2*q_rot.x*q_rot.z*old_p.x + 2*q_rot.y*q_rot.z*old_p.y + q_rot.z*q_rot.z*old_p.z - 2*q_rot.w*q_rot.y*old_p.x - q_rot.y*q_rot.y*old_p.z + 2*q_rot.w*q_rot.x*old_p.y - q_rot.x*q_rot.x*old_p.z + q_rot.w*q_rot.w*old_p.z;
 
   }
 
   void Rotate::rotate(pcl::PointXYZ &p, const geometry_msgs::msg::Quaternion &q_rot)
   {
       pcl::PointXYZ old_p = p;
       p.x = q_rot.w*q_rot.w*old_p.x + 2*q_rot.y*q_rot.w*old_p.z - 2*q_rot.z*q_rot.w*old_p.y + q_rot.x*q_rot.x*old_p.x + 2*q_rot.y*q_rot.x*old_p.y + 2*q_rot.z*q_rot.x*old_p.z - q_rot.z*q_rot.z*old_p.x - q_rot.y*q_rot.y*old_p.x;
       p.y = 2*q_rot.x*q_rot.y*old_p.x + q_rot.y*q_rot.y*old_p.y + 2*q_rot.z*q_rot.y*old_p.z + 2*q_rot.w*q_rot.z*old_p.x - q_rot.z*q_rot.z*old_p.y + q_rot.w*q_rot.w*old_p.y - 2*q_rot.x*q_rot.w*old_p.z - q_rot.x*q_rot.x*old_p.y;
       p.z = 2*q_rot.x*q_rot.z*old_p.x + 2*q_rot.y*q_rot.z*old_p.y + q_rot.z*q_rot.z*old_p.z - 2*q_rot.w*q_rot.y*old_p.x - q_rot.y*q_rot.y*old_p.z + 2*q_rot.w*q_rot.x*old_p.y - q_rot.x*q_rot.x*old_p.z + q_rot.w*q_rot.w*old_p.z;
 
   }
 
   void Rotate::rotate(pcl::PointXYZRGB &p, const geometry_msgs::msg::Quaternion &q_rot)
   {
       pcl::PointXYZRGB old_p = p;
       p.x = q_rot.w*q_rot.w*old_p.x + 2*q_rot.y*q_rot.w*old_p.z - 2*q_rot.z*q_rot.w*old_p.y + q_rot.x*q_rot.x*old_p.x + 2*q_rot.y*q_rot.x*old_p.y + 2*q_rot.z*q_rot.x*old_p.z - q_rot.z*q_rot.z*old_p.x - q_rot.y*q_rot.y*old_p.x;
       p.y = 2*q_rot.x*q_rot.y*old_p.x + q_rot.y*q_rot.y*old_p.y + 2*q_rot.z*q_rot.y*old_p.z + 2*q_rot.w*q_rot.z*old_p.x - q_rot.z*q_rot.z*old_p.y + q_rot.w*q_rot.w*old_p.y - 2*q_rot.x*q_rot.w*old_p.z - q_rot.x*q_rot.x*old_p.y;
       p.z = 2*q_rot.x*q_rot.z*old_p.x + 2*q_rot.y*q_rot.z*old_p.y + q_rot.z*q_rot.z*old_p.z - 2*q_rot.w*q_rot.y*old_p.x - q_rot.y*q_rot.y*old_p.z + 2*q_rot.w*q_rot.x*old_p.y - q_rot.x*q_rot.x*old_p.z + q_rot.w*q_rot.w*old_p.z;
 
   }
 
   double Rotate::getYaw(const geometry_msgs::msg::Quaternion &q)
   {
       double roll = 0.0, pitch = 0.0, yaw =0.0;
 
       Rotate::getRPY(q, roll, pitch, yaw);
 
       return yaw;
 
   }
 
   double Rotate::getYaw(tf2::Quaternion &q)
   {
       tf2::Matrix3x3 m(q);
       double roll = 0.0, pitch = 0.0, yaw = 0.0;
       m.getRPY(roll, pitch, yaw);
       return yaw;
   }
 
   tf2::Quaternion Rotate::toQuaternion(double r, double p, double y)
   {
       tf2::Quaternion q;
       q.setRPY(r,p,y);
       return q;
   }
 
   //Static method to convert from rpy to quaternion message
   geometry_msgs::msg::Quaternion Rotate::toQuaternionMsg(double r, double p, double y)
   {
       return tf2::toMsg(Rotate::toQuaternion(r,p,y));
   }
   tf2::Quaternion Rotate::toQuaternion(const std::array<double,3> rpy)
   {
       return ros2::Rotate::toQuaternion((double)rpy[0], (double)rpy[1], (double)rpy[2]);
   }
 
   geometry_msgs::msg::Quaternion Rotate::toQuaternionMsg(const std::array<double,3> rpy)
   {
       return ros2::Rotate::toQuaternionMsg((double)rpy[0], (double)rpy[1], (double)rpy[2]);
 
   }
 
   double Rotate::toRadian(double ang)
   {
       return ang * M_PI / 180.0;
   }
   double Rotate::toDegree(double ang)
   {
       return ang * 180.0 / M_PI;
   }
   void Rotate::toRadian(geometry_msgs::msg::Point &rpy)
   {
       rpy.x = Rotate::toRadian(rpy.x);
       rpy.y = Rotate::toRadian(rpy.y);
       rpy.z = Rotate::toRadian(rpy.z);
   }
   void Rotate::toDegree(geometry_msgs::msg::Point &rpy)
   {
       rpy.x = Rotate::toDegree(rpy.x);
       rpy.y = Rotate::toDegree(rpy.y);
       rpy.z = Rotate::toDegree(rpy.z);
   }
 
   /*
    * @brief transforms a point with the passed transformed stamped
    */
   void Rotate::transform(geometry_msgs::msg::Point32 &p, const geometry_msgs::msg::TransformStamped &tranform)
   {
       rotate(p, tranform.transform.rotation);
       p.x += tranform.transform.translation.x;
       p.y += tranform.transform.translation.y;
       p.z += tranform.transform.translation.z;
   }
 
   /*
    * @brief inverse transform
    */
   void Rotate::reverse(geometry_msgs::msg::Quaternion &q)
   {
       double roll, pitch, yaw;
       getRPY(q, roll, pitch, yaw);
 
       yaw -= M_PI;
       pitch -= M_PI;
       roll -=M_PI;
 
       q = toQuaternionMsg(wrap_pi(roll), wrap_pi(pitch), wrap_pi(yaw));
   }
 
   /*
    * @brief wrap_pi
    */
   double Rotate::wrap_pi(double angle)
   {
     while (angle >= M_PI)
       {
         angle -= 2 * M_PI;
       }
 
     while (angle <= -M_PI)
       {
         angle += 2 * M_PI;
       }
 
     return angle;
   }
 
   void Rotate::transform(const geometry_msgs::msg::Pose &src, geometry_msgs::msg::Pose &des)
   {
       des.position.x = des.position.x - src.position.x;
       des.position.y = des.position.y - src.position.y;
       des.position.z = des.position.z - src.position.z;
 
       geometry_msgs::msg::Vector3 rpy = Rotate::getRPY(src);
       rpy.x *= -1;
       rpy.y *= -1;
       rpy.z *= -1;
       //ROS_INFO("Q:[%f,%f,%f,%f]", src.orientation.x, src.orientation.y, src.orientation.z, src.orientation.w);
       Rotate::rotate(des.position.x, des.position.y, des.position.z, toQuaternionMsg(rpy.x, rpy.y, rpy.z));
 
   }
 
 }