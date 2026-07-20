#ifndef TRAC_IK__ARM_KINEMATICS_HPP_
#define TRAC_IK__ARM_KINEMATICS_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "trac_ik/kdl_tl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "trac_ik/visibility_control.hpp"

namespace TRAC_IK
{

/// @brief End-effector pose in Cartesian space.
struct TRAC_IK_PUBLIC Pose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;   // degrees
  double pitch = 0.0;  // degrees
  double yaw = 0.0;    // degrees
};

/// @brief Unified result for kinematics operations.
///
/// On success:      success=true,  approximate=false, joints populated
/// On failure:      success=false, message set
/// On approximate:  success=true,  approximate=true,  position_error set
struct TRAC_IK_PUBLIC KinResult
{
  bool success = false;
  std::vector<double> joints;       // joint angles in degrees (IK result)
  Pose pose;                        // end-effector pose (FK result)
  std::vector<double> matrix;       // flattened matrix data (Jacobian / pinv result)
  std::string message;
  bool approximate = false;
  double position_error = 0.0;      // meters, meaningful when approximate=true
};

/// @brief Kinematics solver wrapper using TRAC-IK.
class TRAC_IK_PUBLIC ArmKinematics
{
public:
  /// @brief Construct from URDF XML string (provided by caller).
  /// @param urdf_xml     URDF robot description as an XML string.
  /// @param base_link    Name of the base link.
  /// @param tip_link     Name of the end-effector (tip) link.
  /// @param timeout      Default IK timeout in seconds.
  /// @param eps          Default Cartesian error tolerance.
  ArmKinematics(
    const std::string & urdf_xml,
    const std::string & base_link = "base_link",
    const std::string & tip_link = "gripper_tcp",
    double timeout = 0.005,
    double eps = 1e-5);

  ~ArmKinematics() = default;

  /// @brief Number of active joints in the kinematic chain.
  size_t numJoints() const { return num_joints_; }

  /// @brief Joint names in chain order (base → tip).
  const std::vector<std::string> & jointNames() const { return joint_names_; }

  /// @brief Joint limits in degrees, as (lower, upper) pairs.
  const std::vector<std::pair<double, double>> & jointLimits() const { return joint_limits_deg_; }

  // ---- Cartesian tolerance bounds ----

  /// @brief Get current Cartesian tolerance bounds.
  const KDL::Twist & bounds() const { return bounds_; }

  /// @brief Set Cartesian tolerance bounds for IK solving.
  /// @param x, y, z       Position tolerances (meters).
  /// @param roll, pitch, yaw  Orientation tolerances (radians).
  void setBounds(double x, double y, double z,
                 double roll, double pitch, double yaw);

  // ---- Forward Kinematics ----

  /// @brief Compute forward kinematics.
  /// @param joints_deg  Joint angles in degrees [num_joints].
  /// @return KinResult with pose on success.
  KinResult forwardKinematics(const std::vector<double> & joints_deg) const;

  // ---- Inverse Kinematics ----

  /// @brief Compute inverse kinematics.
  /// @param x, y, z            Target position (meters).
  /// @param roll, pitch, yaw   Target orientation (degrees).
  /// @param seed               Initial joint seed (degrees, size num_joints).
  ///                           Empty = let TRAC-IK start from zero.
  /// @param allow_approximate  If true, return best-effort on total failure.
  /// @return KinResult with joints on success.
  KinResult inverseKinematics(
    double x, double y, double z,
    double roll = 0.0, double pitch = 0.0, double yaw = 0.0,
    const std::vector<double> & seed = {},
    bool allow_approximate = false);

  // ---- Jacobian ----

  /// @brief Compute the 6×N geometric Jacobian at the given configuration.
  /// @param joints_deg  Joint angles in degrees.
  /// @return KinResult with a flat vector of 6*N elements (row-major).
  KinResult computeJacobian(const std::vector<double> & joints_deg) const;

  /// @brief Compute damped pseudo-inverse J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹.
  /// @param joints_deg  Joint angles in degrees.
  /// @param damping     Damping factor λ (default 0.01).
  /// @return KinResult with flat vector of N*6 elements (row-major).
  KinResult getJacobianPinv(
    const std::vector<double> & joints_deg, double damping = 0.01) const;

  /// @brief Compute Yoshikawa manipulability μ = √det(JJᵀ).
  /// @param joints_deg  Joint angles in degrees.
  /// @return μ > 0 = well-conditioned, ≈0 = near singularity, <0 = error.
  double computeManipulability(const std::vector<double> & joints_deg) const;

private:
  // ---- Internal helpers ----

  /// Build KDL chain and extract joint names/limits from URDF XML.
  void buildChain(const std::string & urdf_xml);

  /// Single IK attempt with given seed. Returns true on success.
  bool solveIkOnce(
    const KDL::Frame & target,
    const KDL::JntArray & seed,
    KDL::JntArray & result) const;

  /// Try approximate IK using KDL TL solver (writes best effort to q_out).
  bool solveIkApproximate(
    const KDL::Frame & target,
    const KDL::JntArray & seed,
    KDL::JntArray & result) const;

  KDL::JntArray toJntArray(const std::vector<double> & joints_deg) const;
  std::vector<double> toDegVector(const KDL::JntArray & q) const;

  static double deg2rad(double deg) { return deg * M_PI / 180.0; }
  static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

  // ---- Members ----

  size_t num_joints_ = 0;

  std::string base_link_;
  std::string tip_link_;

  std::vector<std::string> joint_names_;
  std::vector<std::pair<double, double>> joint_limits_deg_;  // degrees

  KDL::Chain chain_;
  KDL::JntArray lb_;   // lower bounds (radians)
  KDL::JntArray ub_;   // upper bounds (radians)

  // TRAC-IK solver for exact IK
  std::unique_ptr<TRAC_IK> ik_solver_;

  // KDL solvers
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_TL> tl_solver_;  // for approximate fallback
  KDL::Twist bounds_;

  double eps_ = 1e-5;
};

}  // namespace TRAC_IK

#endif  // TRAC_IK__ARM_KINEMATICS_HPP_
