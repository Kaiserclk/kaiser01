#include "trac_ik/arm_kinematics.hpp"
#include <algorithm>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include "Eigen/Dense"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

namespace TRAC_IK
{

  void ArmKinematics::buildChain(const std::string &urdf_xml)
  {
    urdf::Model robot_model;
    if (!robot_model.initString(urdf_xml))
    {
      throw std::runtime_error("Unable to initialize urdf::Model from robot description");
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
    {
      throw std::runtime_error("Failed to extract KDL tree from URDF");
    }

    if (!tree.getChain(base_link_, tip_link_, chain_))
    {
      throw std::runtime_error("Could not find chain from '" + base_link_ + "' to '" + tip_link_ + "'");
    }

    num_joints_ = chain_.getNrOfJoints();
    if (num_joints_ == 0)
    {
      throw std::runtime_error("Chain has 0 joints");
    }

    // Allocate limit arrays
    lb_.resize(num_joints_);
    ub_.resize(num_joints_);
    joint_names_.resize(num_joints_);
    joint_limits_deg_.resize(num_joints_);

    // Extract joint names and limits
    const auto &segments = chain_.segments;
    size_t joint_idx = 0;
    for (size_t i = 0; i < segments.size(); ++i)
    {
      const auto &joint_name = segments[i].getJoint().getName();
      auto joint = robot_model.getJoint(joint_name);
      if (!joint || joint->type == urdf::Joint::UNKNOWN ||
          joint->type == urdf::Joint::FIXED)
      {
        continue;
      }

      joint_names_[joint_idx] = joint_name;
      double lower = -M_PI;
      double upper = M_PI;

      if (joint->type != urdf::Joint::CONTINUOUS && joint->limits)
      {
        lower = joint->limits->lower;
        upper = joint->limits->upper;
        if (joint->safety)
        {
          lower = std::max(lower, joint->safety->soft_lower_limit);
          upper = std::min(upper, joint->safety->soft_upper_limit);
        }
      }

      lb_(joint_idx) = lower;
      ub_(joint_idx) = upper;
      joint_limits_deg_[joint_idx] = {rad2deg(lower), rad2deg(upper)};
      joint_idx++;
    }

    if (joint_idx != num_joints_)
    {
      throw std::runtime_error("Joint count mismatch during limit extraction");
    }
  }

  ArmKinematics::ArmKinematics(const std::string &urdf_xml, const std::string &base_link,
                               const std::string &tip_link, double timeout, double eps)
      : base_link_(base_link),
        tip_link_(tip_link),
        eps_(eps)
  {
    buildChain(urdf_xml);

    // TRAC-IK solver for exact IK
    ik_solver_ = std::make_unique<TRAC_IK>(chain_, lb_, ub_, timeout, eps, Speed);

    // KDL forward kinematics and Jacobian
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);

    // KDL TL solver for approximate IK fallback (longer timeout, random restart, joint wrap)
    tl_solver_ = std::make_unique<KDL::ChainIkSolverPos_TL>(
        chain_, lb_, ub_, timeout * 10.0, eps, true, true);

    bounds_ = KDL::Twist(
        KDL::Vector(eps_, eps_, eps_),  // vel: x, y, z
        KDL::Vector(eps_, eps_, eps_)); // rot: roll, pitch, yaw

  }

  void ArmKinematics::setBounds(
      double x, double y, double z,
      double roll, double pitch, double yaw)
  {
    bounds_ = KDL::Twist(
        KDL::Vector(x, y, z),
        KDL::Vector(roll, pitch, yaw));
  }

  KinResult ArmKinematics::forwardKinematics(const std::vector<double> &joints_deg) const
  {
    KinResult result;
    if (joints_deg.size() != num_joints_)
    {
      result.message = "Expected " + std::to_string(num_joints_) +
                       " joint angles, got " + std::to_string(joints_deg.size());
      return result;
    }

    KDL::JntArray q = toJntArray(joints_deg);
    KDL::Frame frame;
    int rc = fk_solver_->JntToCart(q, frame);
    if (rc < 0)
    {
      result.message = "FK solver failed (code " + std::to_string(rc) + ")";
      return result;
    }

    double roll, pitch, yaw;
    frame.M.GetRPY(roll, pitch, yaw);

    result.success = true;
    result.pose.x = frame.p.x();
    result.pose.y = frame.p.y();
    result.pose.z = frame.p.z();
    result.pose.roll = rad2deg(roll);
    result.pose.pitch = rad2deg(pitch);
    result.pose.yaw = rad2deg(yaw);
    return result;
  }


  KinResult ArmKinematics::inverseKinematics(
      double x, double y, double z,
      double roll, double pitch, double yaw,
      const std::vector<double> &seed,
      bool allow_approximate)
  {
    KinResult result;
    if (!seed.empty() && seed.size() != num_joints_)
    {
      result.message = "Expected " + std::to_string(num_joints_) +
                       " seed angles, got " + std::to_string(seed.size());
      return result;
    }

    KDL::Frame target_frame(
        KDL::Rotation::RPY(deg2rad(roll), deg2rad(pitch), deg2rad(yaw)),
        KDL::Vector(x, y, z));

    KDL::JntArray q_result(num_joints_);

    // ---- Stage 1: Exact IK (TRAC-IK handles random seed retry internally) ----
    KDL::JntArray q_seed;
    if (!seed.empty())
    {
      q_seed = toJntArray(seed);
    }
    else
    {
      q_seed.resize(num_joints_);  // zero seed — TRAC-IK will retry with random seeds
    }

    if (solveIkOnce(target_frame, q_seed, q_result))
    {
      result.success = true;
      result.joints = toDegVector(q_result);
      return result;
    }

    // ---- Stage 2: Approximate fallback ----
    if (allow_approximate)
    {
      // Try approximate with user seed (or zero if none given)
      if (solveIkApproximate(target_frame, q_seed, q_result))
      {
        // Clamp to joint limits
        for (size_t j = 0; j < num_joints_; ++j)
        {
          q_result(j) = std::max(lb_(j), std::min(q_result(j), ub_(j)));
        }

        // Evaluate via FK
        auto fk = forwardKinematics(toDegVector(q_result));
        if (fk.success)
        {
          double dx = fk.pose.x - x;
          double dy = fk.pose.y - y;
          double dz = fk.pose.z - z;
          double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);

          result.success = true;
          result.joints = toDegVector(q_result);
          result.approximate = true;
          result.position_error = pos_err;
          result.message = "Approximate solution (position error: " +
                           std::to_string(pos_err) + " m)";
          return result;
        }
      }
    }

    result.message = "IK failed (target unreachable)";
    return result;
  }

  // =========================================================================
  //  Single IK attempt (exact)
  // =========================================================================

  bool ArmKinematics::solveIkOnce(
      const KDL::Frame &target,
      const KDL::JntArray &seed,
      KDL::JntArray &result) const
  {
    KDL::JntArray q_out(num_joints_);
    int rc = ik_solver_->CartToJnt(seed, target, q_out, bounds_);
    if (rc < 0)
    {
      return false;
    }

    // Check joint limits (±1° tolerance)
    for (size_t i = 0; i < num_joints_; ++i)
    {
      double angle = rad2deg(q_out(i));
      double lo = joint_limits_deg_[i].first;
      double hi = joint_limits_deg_[i].second;
      if (angle < lo - 1.0 || angle > hi + 1.0)
      {
        return false;
      }
    }

    result = q_out;
    return true;
  }

  // =========================================================================
  //  Approximate IK using KDL TL solver
  // =========================================================================

  bool ArmKinematics::solveIkApproximate(
      const KDL::Frame &target,
      const KDL::JntArray &seed,
      KDL::JntArray &result) const
  {
    // TL solver writes best-effort result to result even on timeout
    KDL::JntArray q_out(num_joints_);
    tl_solver_->CartToJnt(seed, target, q_out, bounds_);
    result = q_out;
    return true; // always succeeds (best-effort), caller evaluates quality
  }

  // =========================================================================
  //  Jacobian
  // =========================================================================

  KinResult ArmKinematics::computeJacobian(const std::vector<double> &joints_deg) const
  {
    KinResult result;
    if (joints_deg.size() != num_joints_)
    {
      result.message = "Expected " + std::to_string(num_joints_) +
                       " joint angles, got " + std::to_string(joints_deg.size());
      return result;
    }

    KDL::JntArray q = toJntArray(joints_deg);
    KDL::Jacobian jac(num_joints_);
    int rc = jac_solver_->JntToJac(q, jac);
    if (rc < 0)
    {
      result.message = "Jacobian solver failed (code " + std::to_string(rc) + ")";
      return result;
    }

    // Flatten 6×N matrix into row-major vector
    std::vector<double> flat(6 * num_joints_);
    for (int row = 0; row < 6; ++row)
    {
      for (size_t col = 0; col < num_joints_; ++col)
      {
        flat[row * num_joints_ + col] = jac(row, col);
      }
    }

    result.success = true;
    result.matrix = flat; // 6×N row-major
    return result;
  }

  // =========================================================================
  //  Damped pseudo-inverse
  // =========================================================================

  KinResult ArmKinematics::getJacobianPinv(
      const std::vector<double> &joints_deg, double damping) const
  {
    KinResult result;
    auto jac_result = computeJacobian(joints_deg);
    if (!jac_result.success)
    {
      result.message = "Jacobian computation failed: " + jac_result.message;
      return result;
    }

    // Reconstruct 6×N matrix from flattened data
    const auto &flat = jac_result.matrix;
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        J(flat.data(), 6, num_joints_);

    // Damped pseudo-inverse: Jᵀ(JJᵀ + λ²I)⁻¹
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    Eigen::Matrix<double, 6, 6> damp = (damping * damping) * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> to_invert = JJt + damp;

    Eigen::Matrix<double, 6, 6> inv;
    bool invertible = false;
    // Try LDLT for symmetric positive-(semi)definite
    {
      Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(to_invert);
      if (ldlt.info() == Eigen::Success)
      {
        inv = ldlt.solve(Eigen::Matrix<double, 6, 6>::Identity());
        invertible = true;
      }
    }
    if (!invertible)
    {
      // Fallback to fullPivLu
      Eigen::FullPivLU<Eigen::Matrix<double, 6, 6>> lu(to_invert);
      if (!lu.isInvertible())
      {
        result.message = "Pseudo-inverse singular: (JJᵀ + λ²I) not invertible";
        return result;
      }
      inv = lu.inverse();
    }

    // N×6 result
    Eigen::MatrixXd J_pinv = J.transpose() * inv;

    // Flatten row-major
    std::vector<double> flat_pinv(num_joints_ * 6);
    for (size_t i = 0; i < num_joints_; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        flat_pinv[i * 6 + j] = J_pinv(i, j);
      }
    }

    result.success = true;
    result.matrix = flat_pinv;
    return result;
  }

  // =========================================================================
  //  Manipulability
  // =========================================================================

  double ArmKinematics::computeManipulability(const std::vector<double> &joints_deg) const
  {
    auto jac_result = computeJacobian(joints_deg);
    if (!jac_result.success)
    {
      return -1.0;
    }

    const auto &flat = jac_result.matrix;
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        J(flat.data(), 6, num_joints_);

    double det = (J * J.transpose()).determinant();
    return std::sqrt(std::max(det, 0.0));
  }


  KDL::JntArray ArmKinematics::toJntArray(const std::vector<double> &joints_deg) const
  {
    KDL::JntArray q(num_joints_);
    for (size_t i = 0; i < num_joints_ && i < joints_deg.size(); ++i)
    {
      q(i) = deg2rad(joints_deg[i]);
    }
    return q;
  }

  std::vector<double> ArmKinematics::toDegVector(const KDL::JntArray &q) const
  {
    std::vector<double> v(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i)
    {
      v[i] = rad2deg(q(i));
    }
    return v;
  }

} // namespace TRAC_IK
