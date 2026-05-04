#ifndef KINDR_MINIMAL_QUAT_TRANSFORMATION_H_
#define KINDR_MINIMAL_QUAT_TRANSFORMATION_H_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kindr {
namespace minimal {

template <typename Scalar>
class RotationQuaternionTemplate {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Implementation = Eigen::Quaternion<Scalar>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  RotationQuaternionTemplate() : q_(Implementation::Identity()) {}

  explicit RotationQuaternionTemplate(const Matrix3& rotation_matrix)
      : q_(rotation_matrix) {
    q_.normalize();
  }

  explicit RotationQuaternionTemplate(const Implementation& quaternion)
      : q_(quaternion) {
    q_.normalize();
  }

  const Implementation& toImplementation() const {
    return q_;
  }

  Matrix3 getRotationMatrix() const {
    return q_.toRotationMatrix();
  }

  Vector3 rotate(const Vector3& vector) const {
    return q_ * vector;
  }

  static bool isValidRotationMatrix(
      const Matrix3& rotation_matrix,
      const Scalar tolerance = static_cast<Scalar>(1e-3)) {
    return (rotation_matrix.transpose() * rotation_matrix -
            Matrix3::Identity())
               .norm() < tolerance &&
           std::abs(rotation_matrix.determinant() - Scalar(1)) < tolerance;
  }

 private:
  Implementation q_;
};

template <typename Scalar>
class QuatTransformationTemplate {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Rotation = RotationQuaternionTemplate<Scalar>;
  using Quaternion = typename Rotation::Implementation;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;

  QuatTransformationTemplate()
      : rotation_(Rotation()), position_(Vector3::Zero()) {}

  QuatTransformationTemplate(const Rotation& rotation, const Vector3& position)
      : rotation_(rotation), position_(position) {}

  QuatTransformationTemplate(const Matrix4& matrix)
      : rotation_(Rotation(matrix.template block<3, 3>(0, 0))),
        position_(matrix.template block<3, 1>(0, 3)) {}

  void setIdentity() {
    rotation_ = Rotation();
    position_.setZero();
  }

  const Vector3& getPosition() const {
    return position_;
  }

  const Rotation& getRotation() const {
    return rotation_;
  }

  Matrix3 getRotationMatrix() const {
    return rotation_.getRotationMatrix();
  }

  Quaternion getEigenQuaternion() const {
    return rotation_.toImplementation();
  }

  Matrix4 getTransformationMatrix() const {
    Matrix4 matrix = Matrix4::Identity();
    matrix.template block<3, 3>(0, 0) = getRotationMatrix();
    matrix.template block<3, 1>(0, 3) = position_;
    return matrix;
  }

  QuatTransformationTemplate inverse() const {
    Quaternion q_inv = rotation_.toImplementation().conjugate();
    return QuatTransformationTemplate(
        Rotation(q_inv), -(q_inv * position_));
  }

  Vector3 operator*(const Vector3& point) const {
    return rotation_.rotate(point) + position_;
  }

  QuatTransformationTemplate operator*(
      const QuatTransformationTemplate& other) const {
    Quaternion q = rotation_.toImplementation() *
                   other.rotation_.toImplementation();
    Vector3 p = rotation_.rotate(other.position_) + position_;
    return QuatTransformationTemplate(Rotation(q), p);
  }

  template <typename OtherScalar>
  QuatTransformationTemplate<OtherScalar> cast() const {
    using OtherRotation = RotationQuaternionTemplate<OtherScalar>;
    return QuatTransformationTemplate<OtherScalar>(
        OtherRotation(rotation_.toImplementation().template cast<OtherScalar>()),
        position_.template cast<OtherScalar>());
  }

  Vector6 log() const {
    Vector6 result;
    const Vector3 omega = logQuaternion(rotation_.toImplementation());
    const Matrix3 V_inv = leftJacobianInverseSO3(omega);
    result.template head<3>() = V_inv * position_;
    result.template tail<3>() = omega;
    return result;
  }

  static QuatTransformationTemplate exp(const Vector6& tangent) {
    const Vector3 upsilon = tangent.template head<3>();
    const Vector3 omega = tangent.template tail<3>();
    const Matrix3 V = leftJacobianSO3(omega);
    return QuatTransformationTemplate(
        Rotation(expQuaternion(omega)), V * upsilon);
  }

 private:
  static Matrix3 skew(const Vector3& vector) {
    Matrix3 matrix;
    matrix << Scalar(0), -vector.z(), vector.y(),
        vector.z(), Scalar(0), -vector.x(),
        -vector.y(), vector.x(), Scalar(0);
    return matrix;
  }

  static Quaternion expQuaternion(const Vector3& omega) {
    const Scalar theta = omega.norm();
    if (theta < Scalar(1e-8)) {
      Quaternion q(Scalar(1), omega.x() / Scalar(2), omega.y() / Scalar(2),
                   omega.z() / Scalar(2));
      q.normalize();
      return q;
    }
    const Vector3 axis = omega / theta;
    return Quaternion(Eigen::AngleAxis<Scalar>(theta, axis));
  }

  static Vector3 logQuaternion(const Quaternion& quaternion_in) {
    Quaternion quaternion = quaternion_in;
    quaternion.normalize();
    if (quaternion.w() < Scalar(0)) {
      quaternion.coeffs() *= Scalar(-1);
    }
    const Scalar sin_half_theta = quaternion.vec().norm();
    if (sin_half_theta < Scalar(1e-8)) {
      return Scalar(2) * quaternion.vec();
    }
    const Scalar half_theta = std::atan2(sin_half_theta, quaternion.w());
    return (Scalar(2) * half_theta / sin_half_theta) * quaternion.vec();
  }

  static Matrix3 leftJacobianSO3(const Vector3& omega) {
    const Scalar theta = omega.norm();
    const Matrix3 omega_hat = skew(omega);
    if (theta < Scalar(1e-8)) {
      return Matrix3::Identity() + Scalar(0.5) * omega_hat;
    }
    return Matrix3::Identity() +
           ((Scalar(1) - std::cos(theta)) / (theta * theta)) * omega_hat +
           ((theta - std::sin(theta)) / (theta * theta * theta)) *
               omega_hat * omega_hat;
  }

  static Matrix3 leftJacobianInverseSO3(const Vector3& omega) {
    const Scalar theta = omega.norm();
    const Matrix3 omega_hat = skew(omega);
    if (theta < Scalar(1e-8)) {
      return Matrix3::Identity() - Scalar(0.5) * omega_hat;
    }
    const Scalar half_theta = Scalar(0.5) * theta;
    return Matrix3::Identity() - Scalar(0.5) * omega_hat +
           (Scalar(1) -
            theta * std::cos(half_theta) / (Scalar(2) * std::sin(half_theta))) /
               (theta * theta) *
               omega_hat * omega_hat;
  }

  Rotation rotation_;
  Vector3 position_;
};

}  // namespace minimal
}  // namespace kindr

#endif  // KINDR_MINIMAL_QUAT_TRANSFORMATION_H_
