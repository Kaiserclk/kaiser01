"""TRAC-IK Python bindings.

Usage::

    from trac_ik_py import ArmKinematics, Pose, KinResult

    # Read URDF XML (caller is responsible for file I/O)
    with open("robot.urdf") as f:
        urdf_xml = f.read()

    kin = ArmKinematics(urdf_xml, base_link="base_link", tip_link="tool0")

    # Forward kinematics
    result = kin.forward_kinematics([0.0, -45.0, 30.0, 0.0, 90.0])
    if result.success:
        print(result.pose)

    # Inverse kinematics
    result = kin.inverse_kinematics(0.5, 0.0, 0.3, roll=0.0, pitch=45.0, yaw=0.0)
    if result.success:
        print(result.joints)

    # For 5DOF arms, relax uncontrollable dimensions via set_bounds()
    import math
    kin.set_bounds(1e-5, 1e-5, 1e-5, math.pi, 1e-5, math.pi)
    result = kin.inverse_kinematics(0.5, 0.0, 0.3, pitch=45.0)
"""

from trac_ik_py._trac_ik import Pose, KinResult, ArmKinematics

__all__ = ["Pose", "KinResult", "ArmKinematics"]
