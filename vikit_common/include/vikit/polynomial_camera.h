/*
 * polynomial_camera.h
 *
 *  Created on: January 26, 2023
 *      Author: xuankuzcr
 */

#ifndef POLYNOMIAL_CAMERA_H_
#define POLYNOMIAL_CAMERA_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <vikit/abstract_camera.h>
#include <opencv2/opencv.hpp>

namespace vk {

using namespace std;
using namespace Eigen;

class PolynomialCamera : public AbstractCamera {

private:
  const double fx_, fy_;
  const double cx_, cy_;
  const double skew_;
  bool distortion_;             //!< is it pure pinhole model or has it equidistant distortion
  double k2_, k3_, k4_, k5_, k6_, k7_;
  Matrix3d K_, K_inv_;
  double m_inv_K11, m_inv_K12, m_inv_K13, m_inv_K22, m_inv_K23;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PolynomialCamera(double width, double height, double scale,
                double fx, double fy, double skew, double cx, double cy,
                double k2=0.0, double k3=0.0, double k4=0.0, double k5=0.0, double k6=0.0, double k7=0.0);

  ~PolynomialCamera();

  inline double r(double k2, double k3, double k4, double k5, double k6, double k7, double theta) const
  {
    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;
    const double theta4 = theta3 * theta;
    const double theta5 = theta4 * theta;
    const double theta6 = theta5 * theta;
    const double theta7 = theta6 * theta;
    // clang-format off
    return theta + k2 * theta2 + k3 * theta3 + k4 * theta4 + k5 * theta5 + k6 * theta6 + k7 * theta7; // clang-format on
  }

  virtual Vector3d
  cam2world(const double& x, const double& y) const;

  virtual Vector3d
  cam2world(const Vector2d& px) const;

  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const;

  virtual Vector2d
  world2cam(const Vector2d& uv) const;

  const Vector2d focal_length() const
  {
    return Vector2d(fx_, fy_);
  }

  virtual double errorMultiplier2() const
  {
    return fabs(fx_);
  }

  virtual double errorMultiplier() const
  {
    return fabs(4.0*fx_*fy_);
  }

  virtual double fx() const { return fx_; };
  virtual double fy() const { return fy_; };
  virtual double cx() const { return cx_; };
  virtual double cy() const { return cy_; };
};

} // end namespace vk


#endif /* #define POLYNOMIAL_CAMERA_H_ */
