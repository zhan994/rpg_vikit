/*
 * equidistant_camera.h
 *
 *  Created on: January 26, 2023
 *      Author: xuankuzcr
 */

#ifndef EQUIDISTANT_CAMERA_H_
#define EQUIDISTANT_CAMERA_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <vikit/abstract_camera.h>
#include <opencv2/opencv.hpp>

namespace vk {

using namespace std;
using namespace Eigen;

class EquidistantCamera : public AbstractCamera {

private:
  const double fx_, fy_;
  const double cx_, cy_;
  bool distortion_;             //!< is it pure pinhole model or has it equidistant distortion
  double k1_, k2_, k3_, k4_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EquidistantCamera(double width, double height, double scale,
                double fx, double fy, double cx, double cy,
                double k1=0.0, double k2=0.0, double k3=0.0, double k4=0.0);

  ~EquidistantCamera();

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
  
  inline double thetad_from_theta(const double theta) const
  {
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    const double thetad = theta * (1.0 + k1_ * theta2 + k2_ * theta4 +
                                   k3_ * theta6 + k4_ * theta8);
    return thetad;
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


#endif /* #define EQUIDISTANT_CAMERA_H_ */
