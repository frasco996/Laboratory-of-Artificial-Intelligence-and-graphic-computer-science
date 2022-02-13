#include "tf/tf.h"
#include <Eigen/Geometry>

inline Eigen::Isometry2f convertPose2D(const tf::StampedTransform& f) {
  double y,p,r;
  tf::Matrix3x3 matrix =  f.getBasis();
  matrix.getRPY(r, p, y);
  Eigen::Isometry2f F;
  F.setIdentity();
  Eigen::Matrix2f H;
  H << std::cos(y), -std::sin(y),
    std::sin(y), std::cos(y);
  F.linear() = H;
  F.translation() = Eigen::Vector2f(f.getOrigin().x(), f.getOrigin().y());
  return F;
}
