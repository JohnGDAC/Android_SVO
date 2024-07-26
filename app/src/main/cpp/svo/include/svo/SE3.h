

#ifndef _SE3_H
#define _SE3_H

#include "SO3.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

class SE3
{
public:
    typedef Point3d Vec3;

public:
    SE3(): my_rotation(0, 0, 0, 1), my_translation(0, 0, 0){}

    SE3(const double& x, const double& y,const double& z,
        const double& wx, const double& wy, const double& wz, const double& w)
        :my_rotation(wx,wy,wz,w),my_translation(x,y,z){}

    SE3(const SO3& r, const Vec3& t)
        :my_rotation(r), my_translation(t){}

    inline SO3& get_rotation(){return my_rotation;}
    /// @overload
    inline const SO3& get_rotation() const {return my_rotation;}

    /// Returns the translation part of the transformation as a Vector
    inline Vec3& get_translation() {return my_translation;}
    /// @overload
    inline const Vec3& get_translation() const {return my_translation;}

    inline SE3 inverse() const {
        const SO3 rinv(my_rotation.inverse());
        return SE3(rinv, -(rinv * my_translation));
    }

    inline SE3& operator *=(const SE3& rhs) {
        my_translation = my_translation + my_rotation * rhs.get_translation();
        my_rotation = my_rotation*rhs.get_rotation();
        return *this;
    }

    inline SE3 operator *(const SE3& rhs) const
    {
        return SE3(my_rotation*rhs.get_rotation(),
                    my_translation + my_rotation*rhs.get_translation());
    }


    friend Vec3 operator*(const SE3& lhs, const Vec3& rhs){
        return lhs.get_translation() + lhs.get_rotation() * rhs;
    }

    friend Eigen::Vector3d operator*(const SE3& lhs, const Eigen::Vector3d& rhs){
        Point3d tmp(rhs.x(), rhs.y(), rhs.z());
        tmp = lhs.get_translation() + lhs.get_rotation() * tmp;
        return Eigen::Vector3d(tmp.x, tmp.y, tmp.z);
    }

    friend inline std::ostream& operator <<(std::ostream& os, const SE3& rhs){
        os << rhs.get_translation();
        os << " " << rhs.get_rotation();
        return os;
    }

    friend inline std::istream &operator>>(std::istream &is, SE3 &rhs)
    {
        double x, y, z, rx, ry, rz, w;
        is >> x >> y >> z >> rx >> ry >> rz >> w;
        rhs = SE3(x, y, z, rx, ry, rz, w);
        return is;
    }

    /// R is described as follow, M is a 4*4 Homography matrix
    /// |r0 r1 r2|
    /// |r3 r4 r5|
    /// |r6 r7 r8|
    void fromMatrix(const double* m)
    {
        double r[9];
        r[0]=m[0];  r[1]=m[1];  r[2] =m[2];  my_translation.x =m[3];
        r[3]=m[4];  r[4]=m[5];  r[5] =m[6];  my_translation.y =m[7];
        r[6]=m[8];  r[7]=m[9];  r[8]=m[10];  my_translation.z =m[11];
        my_rotation.fromMatrix(r);
    }

    //return the matrix
    void getMatrix(double* m)const
    {
        double r[9];
        my_rotation.getMatrix(r);

        m[0]=r[0];  m[1]=r[1];  m[2] =r[2];  m[3] =my_translation.x;
        m[4]=r[3];  m[5]=r[4];  m[6] =r[5];  m[7] =my_translation.y;
        m[8]=r[6];  m[9]=r[7];  m[10]=r[8];  m[11]=my_translation.z;
    }

    // inline Vector<Precision,6> log()const
    // {
    //     Vector<Precision,6> result;
    //     const auto& l=my_rotation;
    //     const auto& t=my_translation;
    //     const Precision squared_w = l.w*l.w;
    //     const Precision n = sqrt(l.x*l.x+l.y*l.y+l.z*l.z);

    //     Precision A_inv;
    //     // Atan-based log thanks to
    //     //
    //     // C. Hertzberg et al.:
    //     // "Integrating Generic Sensor Fusion Algorithms with Sound State
    //     // Representation through Encapsulation of Manifolds"
    //     // Information Fusion, 2011

    //     if (n < NEAR_ZERO)
    //     {
    //         //If n is too small
    //         A_inv = 2./l.w - 2.*(1.0-squared_w)/(l.w*squared_w);
    //         Point3d<Precision> r(l.x*A_inv,l.y*A_inv,l.z*A_inv);
    //         Point3d<Precision> p=t-0.5*r.cross(t)+static_cast<Precision>(1. / 12.)*r.cross(r.cross(t));
    //         result.set(Vector3<Precision>(p.x,p.y,p.z),0,0);
    //         result.set(Vector3<Precision>(r.x,r.y,r.z),3,0);
    //     }
    //     else
    //     {
    //         if (fabs(l.w)<NEAR_ZERO)
    //         {
    //             //If w is too small
    //             if (l.w>0)
    //             {
    //                 A_inv = M_PI/n;
    //             }
    //             else
    //             {
    //                 A_inv = -M_PI/n;
    //             }
    //         }
    //         else
    //             A_inv = 2*atan(n/l.w)/n;

    //         auto theta=A_inv*n;
    //         Point3d<Precision> r(l.x*A_inv,l.y*A_inv,l.z*A_inv);
    //         Point3d<Precision> a=r/theta;
    //         Point3d<Precision> p=t-0.5*r.cross(t)+(1-theta/(2*tan(0.5*theta)))*a.cross(a.cross(t));
    //         result.set(Vector3<Precision>(p.x,p.y,p.z),0,0);
    //         result.set(Vector3<Precision>(r.x,r.y,r.z),3,0);
    //     }
    //     return result;
    // }

    static inline SE3 exp(const double* l)
    {
        Point3d p(l[0],l[1],l[2]);
        Point3d r(l[3],l[4],l[5]);
        double theta_sq = r.dot(r);
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
          double sin_half_theta = sin(half_theta);
          imag_factor = sin_half_theta / theta;
          real_factor = cos(half_theta);
        }

        SO3 R( imag_factor * r.x, imag_factor * r.y,imag_factor * r.z,real_factor);
        auto t= p+(1-cos(theta))/theta_sq*r.cross(p)+
                (theta-sin(theta))/(theta_sq*theta)*r.cross(r.cross(p));
        return SE3(R,t);
    }

    static inline SE3 expFast(const double* l)
    {
        Point3d p(l[0],l[1],l[2]);
        Point3d r(l[3],l[4],l[5]);
        double theta_sq = r.dot(r);
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
          double sin_half_theta = SO3::sine(half_theta);
          imag_factor = sin_half_theta / theta;
          real_factor = SO3::cosine(half_theta);
        }

        SO3 R( imag_factor * r.x, imag_factor * r.y,imag_factor * r.z,real_factor);
        auto t = p + (1-SO3::cosine(theta))/theta_sq*r.cross(p)+
                (theta-SO3::sine(theta))/(theta_sq*theta)*r.cross(r.cross(p));
        return SE3(R, t);
    }


    Eigen::Matrix3d rotation_matrix() const
    {
        double rr[9];
        Eigen::Matrix3d matrix;
        my_rotation.getMatrix(rr);
        matrix << rr[0], rr[1], rr[2],
                  rr[3], rr[4], rr[5],
                  rr[6], rr[7], rr[8];
        return matrix;
    }

    Eigen::Vector3d translation_vec() const
    {
        return Eigen::Vector3d(my_translation.x, my_translation.y, my_translation.z);
    }

    std::string toString()const{std::stringstream sst;sst<<*this;return sst.str();}

protected:
    SO3 my_rotation;
    Vec3 my_translation;
};

#endif
