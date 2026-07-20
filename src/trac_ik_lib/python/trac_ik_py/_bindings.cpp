
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "trac_ik/arm_kinematics.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_trac_ik, m)
{
  m.doc() = "TRAC-IK ArmKinematics Python bindings";

  // ---- Pose ----
  py::class_<TRAC_IK::Pose>(m, "Pose",
    "End-effector pose in Cartesian space.\n\n"
    "All angles are in **degrees**.")
    .def(py::init<>())
    .def_readwrite("x", &TRAC_IK::Pose::x, "X position (meters)")
    .def_readwrite("y", &TRAC_IK::Pose::y, "Y position (meters)")
    .def_readwrite("z", &TRAC_IK::Pose::z, "Z position (meters)")
    .def_readwrite("roll", &TRAC_IK::Pose::roll, "Roll angle (degrees)")
    .def_readwrite("pitch", &TRAC_IK::Pose::pitch, "Pitch angle (degrees)")
    .def_readwrite("yaw", &TRAC_IK::Pose::yaw, "Yaw angle (degrees)")
    .def("__repr__", [](const TRAC_IK::Pose & p) {
      char buf[256];
      snprintf(buf, sizeof(buf),
        "Pose(x=%.4f, y=%.4f, z=%.4f, roll=%.2f, pitch=%.2f, yaw=%.2f)",
        p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
      return std::string(buf);
    });

  // ---- KinResult ----
  py::class_<TRAC_IK::KinResult>(m, "KinResult",
    "Unified result for kinematics operations.\n\n"
    "Attributes:\n"
    "  success (bool):          True if operation succeeded.\n"
    "  approximate (bool):      True if result is approximate (not exact).\n"
    "  joints (list[float]):    Joint angles in degrees (IK result).\n"
    "  pose (Pose):             End-effector pose (FK result).\n"
    "  matrix (list[float]):    Flattened matrix data (Jacobian / pinv).\n"
    "  message (str):           Error or status message.\n"
    "  position_error (float):  Position error in meters (approximate only).")
    .def(py::init<>())
    .def_readwrite("success", &TRAC_IK::KinResult::success)
    .def_readwrite("approximate", &TRAC_IK::KinResult::approximate)
    .def_readwrite("joints", &TRAC_IK::KinResult::joints)
    .def_readwrite("pose", &TRAC_IK::KinResult::pose)
    .def_readwrite("matrix", &TRAC_IK::KinResult::matrix)
    .def_readwrite("message", &TRAC_IK::KinResult::message)
    .def_readwrite("position_error", &TRAC_IK::KinResult::position_error)
    .def("__repr__", [](const TRAC_IK::KinResult & r) {
      if (r.success && r.approximate) {
        return "KinResult(success=True, approximate=True, "
               "position_error=" + std::to_string(r.position_error) + ")";
      }
      if (r.success) {
        return "KinResult(success=True, joints=" +
               std::to_string(r.joints.size()) + " values)";
      }
      return "KinResult(success=False, message='" + r.message + "')";
    });

  // ---- ArmKinematics ----
  py::class_<TRAC_IK::ArmKinematics>(m, "ArmKinematics",
    "Kinematics solver wrapper using TRAC-IK.\n\n"
    "Provides forward kinematics, inverse kinematics, Jacobian computation,\n"
    "damped pseudoinverse, and manipulability measure.\n\n"
    "Args:\n"
    "  urdf_xml:  URDF robot description as an XML string (not a file path).\n"
    "  base_link: Name of the base link (default 'base_link').\n"
    "  tip_link:  Name of the end-effector link (default 'gripper_tcp').\n"
    "  timeout:   Default IK timeout in seconds (default 0.005).\n"
    "  eps:       Cartesian error tolerance (default 1e-5).")
    .def(py::init<const std::string &, const std::string &,
                  const std::string &, double, double>(),
         py::arg("urdf_xml"),
         py::arg("base_link") = "base_link",
         py::arg("tip_link") = "gripper_tcp",
         py::arg("timeout") = 0.005,
         py::arg("eps") = 1e-5)

    // ---- Properties ----
    .def_property_readonly("num_joints", &TRAC_IK::ArmKinematics::numJoints,
      "Number of active joints in the chain.")
    .def_property_readonly("joint_names", &TRAC_IK::ArmKinematics::jointNames,
      "Joint names in chain order (base → tip).")
    .def_property_readonly("joint_limits", &TRAC_IK::ArmKinematics::jointLimits,
      "Joint limits in degrees, list of (lower, upper) pairs.")

    // ---- Bounds ----
    .def("set_bounds", &TRAC_IK::ArmKinematics::setBounds,
         py::arg("x"), py::arg("y"), py::arg("z"),
         py::arg("roll"), py::arg("pitch"), py::arg("yaw"),
         "Set Cartesian tolerance bounds for IK.\n\n"
         "Args:\n"
         "  x, y, z:  Position tolerances (meters).\n"
         "  roll, pitch, yaw:  Orientation tolerances (radians).\n\n"
         "Smaller = stricter. Use M_PI to relax a dimension entirely.")

    // ---- Forward Kinematics ----
    .def("forward_kinematics", &TRAC_IK::ArmKinematics::forwardKinematics,
         py::arg("joints_deg"),
         "Compute forward kinematics.\n\n"
         "Args:\n"
         "  joints_deg (list[float]): Joint angles in degrees.\n\n"
         "Returns:\n"
         "  KinResult: On success, result.pose contains the end-effector pose.")

    // ---- Inverse Kinematics ----
    .def("inverse_kinematics", &TRAC_IK::ArmKinematics::inverseKinematics,
         py::arg("x"), py::arg("y"), py::arg("z"),
         py::arg("roll") = 0.0,
         py::arg("pitch") = 0.0,
         py::arg("yaw") = 0.0,
         py::arg("seed") = std::vector<double>{},
         py::arg("allow_approximate") = false,
         "Compute inverse kinematics.\n\n"
         "TRAC-IK's internal solver handles random seed retries.\n\n"
         "Args:\n"
         "  x, y, z:             Target position (meters).\n"
         "  roll, pitch, yaw:    Target orientation (degrees).\n"
         "  seed (list[float]):  Initial joint seed (degrees). Empty = auto.\n"
         "  allow_approximate:   If True, return best-effort on total failure.\n\n"
         "Returns:\n"
         "  KinResult: On success, result.joints contains the joint angles.")

    // ---- Jacobian ----
    .def("compute_jacobian", &TRAC_IK::ArmKinematics::computeJacobian,
         py::arg("joints_deg"),
         "Compute 6×N geometric Jacobian.\n\n"
         "Args:\n"
         "  joints_deg (list[float]): Joint angles in degrees.\n\n"
         "Returns:\n"
         "  KinResult: result.matrix = 6*N elements (row-major).")
    .def("get_jacobian_pinv", &TRAC_IK::ArmKinematics::getJacobianPinv,
         py::arg("joints_deg"),
         py::arg("damping") = 0.01,
         "Compute damped pseudo-inverse J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹.\n\n"
         "Args:\n"
         "  joints_deg (list[float]): Joint angles in degrees.\n"
         "  damping (float):  Damping factor (default 0.01).\n\n"
         "Returns:\n"
         "  KinResult: result.matrix = N*6 elements (row-major).")

    // ---- Manipulability ----
    .def("compute_manipulability", &TRAC_IK::ArmKinematics::computeManipulability,
         py::arg("joints_deg"),
         "Compute Yoshikawa manipulability μ = √det(JJᵀ).\n\n"
         "Args:\n"
         "  joints_deg (list[float]): Joint angles in degrees.\n\n"
         "Returns:\n"
         "  float: μ > 0 = well-conditioned, ≈ 0 = near singularity, < 0 = error.");
}
