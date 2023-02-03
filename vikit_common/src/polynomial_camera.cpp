/*
 * polynomial_camera.cpp
 *
 *  Created on: February 3, 2023
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vikit/polynomial_camera.h>
#include <vikit/math_utils.h>

namespace vk {

PolynomialCamera::
PolynomialCamera(double width, double height, double scale,
              double fx, double fy, double skew,
              double cx, double cy,
              double k2, double k3, double k4, double k5, double k6, double k7) :
              AbstractCamera(width * scale , height * scale, scale),
              fx_(fx * scale), fy_(fy * scale), cx_(cx * scale), cy_(cy * scale), skew_(skew * scale),
              distortion_(fabs(k2) > 0.0000001)
{
  k2_ = k2; k3_ = k3; k4_ = k4; k5_ = k5; k6_ = k6; k7_ = k7;

  K_ << fx_, skew_, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
  K_inv_ = K_.inverse();

  m_inv_K11 = K_inv_(0, 0);
  m_inv_K12 = K_inv_(0, 1);
  m_inv_K13 = K_inv_(0, 2);
  m_inv_K22 = K_inv_(1, 1);
  m_inv_K23 = K_inv_(1, 2);
}

PolynomialCamera::
~PolynomialCamera()
{}

Vector3d PolynomialCamera::
cam2world (const Vector2d& uv) const
{
  return cam2world(uv[0], uv[1]);
}

Vector3d PolynomialCamera::
cam2world(const double& u, const double& v) const
{
  Vector3d xyz;
  // Lift points to normalised plane
  double cos_theta, sin_theta;
  // Obtain a projective ray
  double cos_phi, sin_phi;

  cos_theta = sin_theta = cos_phi = sin_phi = 0.0;

  if(!distortion_)
  {
    Eigen::Vector2d p_u(m_inv_K11 * u + m_inv_K13, m_inv_K22 * v + m_inv_K23);
    double r  = p_u.norm();
    sin_phi   = p_u[1] / r;
    cos_phi   = p_u[0] / r;
    cos_theta = cos(r);
    sin_theta = sqrt(1 - cos_theta * cos_theta); // sin(r);
  }
  else
  {
    // backprojectSymmetric( Eigen::Vector2d(m_inv_K11 * u + m_inv_K12 * v + m_inv_K13, m_inv_K22 * v + m_inv_K23),
    //                       cos_theta,
    //                       sin_theta,
    //                       cos_phi,
    //                       sin_phi );
    // std::cout << " cos_theta " << cos_theta << std::endl;
  }
  xyz =  Vector3d(cos_phi * sin_theta, sin_phi * sin_theta, cos_theta);
  return xyz.normalized();
}

Vector2d PolynomialCamera::
world2cam(const Vector3d& xyz) const
{
  return world2cam(project2d(xyz));
}

Vector2d PolynomialCamera::
world2cam(const Vector2d& uv) const
{
  Vector3d xyz(uv[0], uv[1], 1);
  double theta = acos(xyz[2] / xyz.norm());
  //    double phi   = atan2(P(1), P(0));
  double inverse_r_P2 = 1.0 / sqrt(xyz[1] * xyz[1] + xyz[0] * xyz[0]);
  double sin_phi      = xyz[1] * inverse_r_P2;
  double cos_phi      = xyz[0] * inverse_r_P2;

  Vector2d p_u, p_x;

  if(!distortion_)
  {
    // p_u = theta * Eigen::Vector2d( cos_phi, sin_phi );

    // Apply generalised projection matrix
    p_x[0] = fx_ * theta * cos_phi /*p_u( 0 )*/ + cx_;
    p_x[1] = fy_ * theta * cos_phi /*p_u( 1 )*/ + cy_;
  }
  else
  {   
    double u = xyz[0] / xyz[2];
    double v = xyz[1] / xyz[2];

    double r_point = r(k2_, k3_, k4_, k5_, k6_, k7_, theta);

    // Eigen::Vector2d du
    // = Eigen::Vector2d( 2.0 * mParameters.p1( ) * u * v +
    // mParameters.p2( ) * ( r_point + 2.0 * u * u ),
    //                   mParameters.p1( ) * ( r_point + 2.0 * v * v ) +
    //                   2.0 * mParameters.p2( ) * u * v );

    p_u = r_point * Vector2d(cos_phi, sin_phi) /*+ du*/;      

    // Apply generalised projection matrix
    p_x[0] = fx_ * p_u( 0 ) + skew_ * p_u( 1 ) + cx_;
    p_x[1] = fy_ * p_u( 1 ) + cy_;
  }
  return p_x;
}

} // end namespace vk
