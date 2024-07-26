

#ifndef _SO3_H
#define _SO3_H

#include <math.h>
#include <iostream>
#include <sstream>


#ifndef M_PI
# define M_PI		3.14159265358979323846	/* pi */
#endif
#define NEAR_ZERO 1e-10

struct Point2d
{
    Point2d(): x(0), y(0){}
    Point2d(double x_, double y_):x(x_),y(y_){}


    friend Point2d operator + (const Point2d& a, const Point2d& b)
    {
        return Point2d(a.x+b.x, a.y+b.y);
    }

    friend Point2d operator - (const Point2d& a, const Point2d& b)
    {
        return Point2d(a.x-b.x, a.y-b.y);
    }

    friend Point2d operator -(const Point2d& a)
    {
        return Point2d(-a.x, -a.y);
    }

    friend Point2d operator * (const Point2d& a, const Point2d& b)
    {
        return Point2d(a.x*b.x, a.y*b.y);
    }

    friend Point2d operator * (const double& a, const Point2d& b)
    {
        return Point2d(a*b.x, a*b.y);
    }

    friend Point2d operator * (const Point2d& b, const double& a)
    {
        return Point2d(a*b.x, a*b.y);
    }

    friend Point2d operator / (const Point2d& a, const double& b)
    {
        return (1./b) * a;
    }

    inline double norm()const
    {
        return sqrt(x*x+y*y);
    }

    inline Point2d normalize()const
    {
        if(x*x+y*y!=0)
            return (*this)*(1./norm());
        else
            return Point2d(0, 0);
    }

    inline double dot(const Point2d& a)const
    {
        return x*a.x+y*a.y;
    }


    std::string toString()const{std::stringstream sst; sst << *this; return sst.str();}

    friend inline std::ostream& operator <<(std::ostream& os, const Point2d& p)
    {
        os << std::to_string(p.x) << " " << std::to_string(p.y);
        return os;
    }

    friend inline std::istream& operator >>(std::istream& is,Point2d& p)
    {
        is >> p.x >> p.y;
        return is;
    }
    double x,y;
};


struct Point3d
{
    Point3d():x(0),y(0),z(0){}

    Point3d(double x_, double y_, double z_):x(x_),y(y_),z(z_){}

    inline double norm()const
    {
        return sqrt(x*x + y*y + z*z);
    }

    inline double dot(const Point3d& a)const
    {
        return x*a.x+y*a.y+z*a.z;
    }

    inline Point3d cross(const Point3d& a)const
    {
        return Point3d(y*a.z - z*a.y, z*a.x - x*a.z, x*a.y - y*a.x);
    }

    inline Point3d normalize()const
    {
        if(x*x+y*y+z*z!=0)
            return (*this)*(1./norm());
        else
            return Point3d(0,0,0);
    }

    friend inline std::ostream& operator <<(std::ostream& os,const Point3d& p)
    {
        os<<std::to_string(p.x)<<" "<<std::to_string(p.y)<<" "<<std::to_string(p.z);
        return os;
    }

    friend inline std::istream& operator >>(std::istream& is,Point3d& p)
    {
        is>>p.x>>p.y>>p.z;
        return is;
    }

    friend Point3d operator + (const Point3d& a,const Point3d& b)
    {
        return Point3d(a.x+b.x,a.y+b.y,a.z+b.z);
    }

    friend Point3d operator - (const Point3d& a,const Point3d& b)
    {
        return Point3d(a.x-b.x,a.y-b.y,a.z-b.z);
    }

    friend Point3d operator -(const Point3d& a)
    {
        return Point3d(-a.x,-a.y,-a.z);
    }

    friend double operator * (const Point3d& a, const Point3d& b)
    {
        return (a.x*b.x+a.y*b.y+a.z*b.z);
    }

    friend Point3d operator ^ (const Point3d& a,const Point3d& b)
    {
        return Point3d(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
    }

    friend Point3d operator * (const double& a,const Point3d& b)
    {
        return Point3d(a*b.x, a*b.y, a*b.z);
    }

    friend Point3d operator * (const Point3d& b, const double& a)
    {
        return Point3d(a*b.x, a*b.y, a*b.z);
    }

    friend Point3d operator / (const Point3d& a,const double& b)
    {
        return (1./b)*a;
    }

    friend inline bool operator < (const Point3d& a,const Point3d b)
    {
        return a.x<b.x;
    }

    std::string toString()const{std::stringstream sst;sst<<*this;return sst.str();}

    double x, y, z;
};

class SO3
{
public:
    /// Default constructor, Idendity Matrix
    SO3():x(0),y(0),z(0),w(1){}

    /// Construct from Quaternion
    SO3(const double& X, const double& Y, const double& Z, const double& W)
        :x(X),y(Y),z(Z),w(W) { }

    /// Construct from a direction and angle in radius.
    SO3(const Point3d& direction, double angle)
    {
        FromAxis(direction, angle);
    }

    /// Construct from a rotation matrix data (ColMajor).
    SO3(const double* M)
    {
        fromMatrix(M);
    }

    SO3(const SO3& r) : SO3(r.x,r.y,r.z,r.w){
    }

    /// @return The lie algebra representation in \f$[a,b,c]\f$
    Point3d log()const
    {
        const double squared_w = w*w;
        const double n = sqrt(x*x+y*y+z*z);

        double A_inv;
        // Atan-based log thanks to
        //
        // C. Hertzberg et al.:
        // "Integrating Generic Sensor Fusion Algorithms with Sound State
        // Representation through Encapsulation of Manifolds"
        // Information Fusion, 2011

        if (n < NEAR_ZERO)
        {
            //If n is too small
            A_inv = 2./w - 2.*(1.0-squared_w)/(w*squared_w);
        }
        else
        {
            if (fabs(w)<NEAR_ZERO)
            {
                //If w is too small
                if (w>0)
                {
                    A_inv = M_PI/n;
                }
                else
                {
                    A_inv = -M_PI/n;
                }
            }
            else
                A_inv = 2*atan(n/w)/n;
        }
        return Point3d(x*A_inv, y*A_inv, z*A_inv);
    }

    /// Construct from lie algebra representation
    static SO3 exp(const Point3d& r)
    {
        const double theta_sq = r.x*r.x+r.y*r.y+r.z*r.z;
        const double theta = sqrt(theta_sq);
        const double half_theta = 0.5*theta;

        const double W = cos(half_theta);
        double sin_half_theta;
        if( theta < NEAR_ZERO)
        {
          double theta_po4 = theta_sq*theta_sq;
          sin_half_theta = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
        }
        else
        {
          sin_half_theta = sin(half_theta);
          sin_half_theta = sin_half_theta/theta;
        }

        return SO3(sin_half_theta*r.x,
                              sin_half_theta*r.y,
                              sin_half_theta*r.z, W);
    }

    static inline double sine(double x) {
        double sin = 0;
        //always wrap input angle to -PI..PI
        while (x < -3.14159265)
            x += 6.28318531;
        while (x > 3.14159265)
            x -= 6.28318531;
        //compute sine
        if (x < 0) {
            sin = 1.27323954 * x + .405284735 * x * x;
            if (sin < 0)
                sin = .225 * (sin * -sin - sin) + sin;
            else
                sin = .225 * (sin * sin - sin) + sin;
        } else {
            sin = 1.27323954 * x - 0.405284735 * x * x;
            if (sin < 0)
                sin = .225 * (sin * -sin - sin) + sin;
            else
                sin = .225 * (sin * sin - sin) + sin;
        }
        return sin;
    }

    static inline double cosine(double x) {
        //compute cosine: sin(x + PI/2) = cos(x)
        return sine(x+1.57079632);
    }

    static SO3 expFast(const Point3d& l)
    {
        double theta_sq = l.dot(l);
        double theta    = sqrt(theta_sq);
        double half_theta = static_cast<double>(0.5) * theta;

        double imag_factor;
        double real_factor;

        if (theta < static_cast<double>(1e-10)) {
          double theta_po4 = theta_sq * theta_sq;
          imag_factor = static_cast<double>(0.5) -
                        static_cast<double>(1.0 / 48.0) * theta_sq +
                        static_cast<double>(1.0 / 3840.0) * theta_po4;
          real_factor = static_cast<double>(1) -
                        static_cast<double>(0.5) * theta_sq +
                        static_cast<double>(1.0 / 384.0) * theta_po4;
        } else {
          double sin_half_theta = sine(half_theta);
          imag_factor = sin_half_theta / theta;
          real_factor = cosine(half_theta);
        }

        return SO3( imag_factor * l.x, imag_factor * l.y,
            imag_factor * l.z, real_factor);
    }

    /// This is an unsafe operation.
    /// Please make sure that your pointer is both valid and has an appropriate size
    /// Wrong usage:
    /// Precision* p;
    /// getMatrix(p);
    ///
    /// Correct:
    /// Precision p[9];
    /// getMatrix(p);
    ///
    /// M is curved as follow
    /// |m0 m1 m2|
    /// |m3 m4 m5|
    /// |m6 m7 m8|
    inline void fromMatrix(const double* m)
    {
        auto SIGN = [](const double& v){return v > 0 ? 1.:-1.;};
        double& q0 = w;
        double& q1 = x;
        double& q2 = y;
        double& q3 = z;
        const double &r11 = m[0], &r12 = m[1], &r13 = m[2],
                &r21 = m[3], &r22 = m[4], &r23 = m[5],
                &r31 = m[6], &r32 = m[7], &r33 = m[8];
        q0 = ( r11 + r22 + r33 + 1.0f) / 4.0f;
        q1 = ( r11 - r22 - r33 + 1.0f) / 4.0f;
        q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
        q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
        if(q0 < 0.0f) q0 = 0.0f;
        if(q1 < 0.0f) q1 = 0.0f;
        if(q2 < 0.0f) q2 = 0.0f;
        if(q3 < 0.0f) q3 = 0.0f;
        q0 = sqrt(q0);
        q1 = sqrt(q1);
        q2 = sqrt(q2);
        q3 = sqrt(q3);
        if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
            q0 *= +1.0f;
            q1 *= SIGN(r32 - r23);
            q2 *= SIGN(r13 - r31);
            q3 *= SIGN(r21 - r12);
        } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
            q0 *= SIGN(r32 - r23);
            q1 *= +1.0f;
            q2 *= SIGN(r21 + r12);
            q3 *= SIGN(r13 + r31);
        } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
            q0 *= SIGN(r13 - r31);
            q1 *= SIGN(r21 + r12);
            q2 *= +1.0f;
            q3 *= SIGN(r32 + r23);
        } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
            q0 *= SIGN(r21 - r12);
            q1 *= SIGN(r31 + r13);
            q2 *= SIGN(r32 + r23);
            q3 *= +1.0f;
        } else {
            std::cerr << "Unable to construct SO3 from this Matrix.";
        }
        double r = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
        q0 /= r;
        q1 /= r;
        q2 /= r;
        q3 /= r;
    }

    /// return the matrix M
    inline void getMatrix(double* m)const
    {
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        double xy = x * y;
        double xz = x * z;
        double yz = y * z;
        double wx = w * x;
        double wy = w * y;
        double wz = w * z;
        m[0]=1.0-2.0*(y2+z2);    m[1]=2.0 * ( xy- wz);  m[2]= 2.0 * (xz + wy);
        m[3]=2.0 * (xy + wz);    m[4]=1.0-2.0*(x2+z2);  m[5]= 2.0 * (yz - wx);
        m[6]=2.0 * (xz - wy);    m[7]=2.0 * ( yz+ wx);  m[8]= 1.0-2.0*(x2+y2);
    }

    static SO3 fromPitchYawRollAngle(const double& pitch, const double& yaw, const double& roll)
    {
        double a2r=(3.1415926/180.0);
        return fromPitchYawRoll(pitch*a2r,yaw*a2r,roll*a2r);
    }

    static SO3 fromPitchYawRoll(const Point3d& pyr){
        return fromPitchYawRoll(pyr.x,pyr.y,pyr.z);
    }

    /// Convert from Euler Angles,
    /// Please use "Radian" rather than degree to present angle
    static SO3 fromPitchYawRoll(const double& pitch, const double& yaw, const double& roll)
    {
        // Basically we create 3 Quaternions, one for pitch, one for yaw, one for roll
        // and multiply those together.
        // the calculation below does the same, just shorter
        double piover360=0.5;//3.1415926/360.0;
        double p = pitch * piover360;
        double y = yaw * piover360;
        double r = roll * piover360;


        double sinp = sin(p);
        double siny = sin(y);
        double sinr = sin(r);
        double cosp = cos(p);
        double cosy = cos(y);
        double cosr = cos(r);


        double rx = sinr * cosp * cosy - cosr * sinp * siny;
        double ry = cosr * sinp * cosy + sinr * cosp * siny;
        double rz = cosr * cosp * siny - sinr * sinp * cosy;
        double rw = cosr * cosp * cosy + sinr * sinp * siny;

        SO3 ret(rx,ry,rz,rw);
        ret.normalise();
        return ret;
    }

    double getRoll()const//Radian about axis X
    {
        return atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y));
    }

    double getPitch()const//Radian about axis Y
    {
        return asin(2.0*(w*y-z*x));
    }

    double getYaw()const//Radian about axis Z
    {
        return atan2(2.0*(w*z+x*y), 1.0-2.0*(z*z+y*y));
    }

    SO3 operator* (const SO3& rq) const
    {
        // the constructor takes its arguments as (x, y, z, w)
        return SO3( w * rq.x + x * rq.w + y * rq.z - z * rq.y,
                    w * rq.y + y * rq.w + z * rq.x - x * rq.z,
                    w * rq.z + z * rq.w + x * rq.y - y * rq.x,
                    w * rq.w - x * rq.x - y * rq.y - z * rq.z);
    }

    // Multiplying a quaternion q with a vector v applies the q-rotation to v
    Point3d operator* (const Point3d& p) const
    {
        // Note that this algorithm comes from the optimization by hand
        // of the conversion to a Matrix followed by a Matrix/Vector product.
        // It appears to be much faster than the common algorithm found
        // in the literature (30 versus 39 flops). It also requires two
        // Vector3 as temporaries.
        Point3d uv = Point3d(x,y,z).cross(p);
        uv = uv + uv;
        return p + w * uv + Point3d(x,y,z).cross(uv);
    }

    bool operator ==(const SO3& rq)const{
        return ((*this).log()-rq.log()).norm()<1e-6;
    }

    // Convert from Axis Angle
    static SO3 FromAxis(const Point3d& p, double angle)
    {
        double det = sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
        if(det<0.00001)
        {
            return SO3();
        }
        angle *= 0.5;
        double p2v = sin(angle)/det;
        return SO3(p.x*p2v, p.y*p2v, p.z*p2v, cos(angle));
    }

    void normalise()
    {
        // Don't normalize if we don't have to
        double mag2 = w * w + x * x + y * y + z * z;
        if (  mag2!=0.f && (fabs(mag2 - 1.0f) > 0.001))
        {
            double mag = 1./sqrt(mag2);
            w *= mag;
            x *= mag;
            y *= mag;
            z *= mag;
        }
    }

    // We need to get the inverse of a quaternion to properly apply a quaternion-rotation to a vector
    // The conjugate of a quaternion is the same as the inverse, as long as the quaternion is unit-length
    SO3 inverse() const
    {
        return SO3(-x, -y, -z, w);
    }

    friend std::ostream& operator << (std::ostream& os, const SO3& so3)
    {
        os << so3.x << " " << so3.y << " " << so3.z << " " << so3.w;
        return os;
    }

    friend std::istream& operator >> (std::istream& is, SO3& so3)
    {
        is >> so3.x >> so3.y >> so3.z >> so3.w;
        return is;
    }

    operator std::string()const
    {
        std::stringstream sst; sst << *this; return sst.str();
    }

    std::string toString()const{std::stringstream sst; sst << *this; return sst.str();}

public:
    double x,y,z,w;
};

#endif 
