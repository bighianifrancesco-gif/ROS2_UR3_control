#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf2_ros import Buffer, TransformListener

import pinocchio as pin

from ros_gz_interfaces.msg import Contacts


class SpiralGenerator:
    """Archimedean spiral: r=a*theta -> (x,y) offsets in meters."""
    def __init__(self, spacing_m: float, max_radius_m: float, dtheta_rad: float):
        self.spacing = float(spacing_m)
        self.max_radius = float(max_radius_m)
        self.dtheta = float(dtheta_rad)
        self.a = self.spacing / (2.0 * math.pi)
        self.theta = 0.0

    def reset(self):
        self.theta = 0.0

    def next(self):
        self.theta += self.dtheta
        r = self.a * self.theta
        if r > self.max_radius:
            return None
        return (r * math.cos(self.theta), r * math.sin(self.theta))


class PegInsertionFSM(Node):
    """
    FSM (simple):
      IDLE -> (MOVE_ABOVE optional) -> GUARDED_INSERT
      contact persistent -> RETRACT -> (ORIENT_UPRIGHT once) -> CLEAR_WAIT -> SEARCH_XY -> CLEAR_WAIT -> GUARDED_INSERT
      success when tip.z <= z_target AND no contact for settle_s
      failure when spiral exhausted

    Notes:
      - Contact topic must be bridged to ROS (ros_gz_interfaces/msg/Contacts).
      - TF must provide hole_frame and tip_frame.
      - IK is translation-only DLS using Pinocchio (LOCAL_WORLD_ALIGNED).
    """

    def __init__(self):
        super().__init__("peg_insertion_fsm")

        # ---------------- Parameters: joints/topics ----------------
        self.joint_names = self.declare_parameter(
            "joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        ).value

        self.joint_states_topic = self.declare_parameter("joint_states_topic", "/joint_states").value
        self.contact_topic = self.declare_parameter("contact_topic", "/peg_tip_contact").value
        self.fjt_action = self.declare_parameter(
            "fjt_action",
            "/joint_trajectory_controller/follow_joint_trajectory",
        ).value

        # ---------------- Parameters: speed/IK defaults (faster) ----------------
        self.ee_frame = self.declare_parameter("ee_frame", "tool0").value
        self.dt_cmd = float(self.declare_parameter("dt_cmd", 0.05).value)          # seconds per micro goal
        self.timer_hz = float(self.declare_parameter("timer_hz", 60.0).value)

        self.lambda_dls = float(self.declare_parameter("lambda_dls", 0.02).value)
        self.dq_max = float(self.declare_parameter("dq_max", 0.08).value)          # rad per step clamp

        # Insertion/retract
        self.dz_step = float(self.declare_parameter("dz_step", -0.005).value)     # m per insert step
        self.retract_z = float(self.declare_parameter("retract_z", 0.01).value)    # 1 cm total retract

        # Contact timing
        self.debounce_s = float(self.declare_parameter("debounce_s", 0.04).value)
        self.clear_s = float(self.declare_parameter("clear_s", 0.12).value)
        self.settle_s = float(self.declare_parameter("settle_s", 0.15).value)

        # ---------------- Parameters: spiral search (bigger) ----------------
        self.search_spacing = float(self.declare_parameter("search_spacing", 0.008).value)  # 5 mm pitch
        self.search_max_r = float(self.declare_parameter("search_max_r", 0.04).value)       # 20 mm radius
        self.search_dtheta = float(self.declare_parameter("search_dtheta", 1.2).value)
        self.xy_step_limit = float(self.declare_parameter("xy_step_limit", 0.008).value)   # 5 mm per command

        # ---------------- Parameters: TF success ----------------
        self.hole_frame = self.declare_parameter("hole_frame", "hole_frame").value
        self.tip_frame = self.declare_parameter("tip_frame", "tool0").value
        self.z_target = float(self.declare_parameter("z_target", -0.02).value)
        self.go_up_remaining = 0.0
        self.go_up_distance = float(self.declare_parameter("go_up_distance", 0.15).value)  # 3 cm up after success


        # ---------------- Parameters: pre-insert pose (optional) ----------------
        self.pre_insert_q = self.declare_parameter("pre_insert_q", []).value
        self.pre_insert_tol = float(self.declare_parameter("pre_insert_tol", 0.02).value)

        # ---------------- Parameters: contact filtering ----------------
        # Your contact echo showed: ur3::peg_sensor_link::peg_tip_col
        self.tip_collision_token = self.declare_parameter("tip_collision_token", "peg_tip_col").value
        self.tip_link_token = self.declare_parameter("tip_link_token", "peg_sensor_link").value

        # Ignore tiny numerical contacts (tune if needed)
        self.min_depth_m = float(self.declare_parameter("min_depth_m", 1e-5).value)
        self.min_force_n = float(self.declare_parameter("min_force_n", 2.0).value)

        # ---------------- Parameters: "upright" on first retract ----------------
        # Simple joint-space wrist alignment; tune these 3 values.
        self.enable_upright_on_first_retract = bool(
            self.declare_parameter("enable_upright_on_first_retract", False).value
        )
        self.upright_wrist = self.declare_parameter("upright_wrist", [0.0, 0.0, 0.0]).value
        if len(self.upright_wrist) != 3:
            raise RuntimeError("Parameter upright_wrist must be a list of 3 floats [wrist_1, wrist_2, wrist_3].")

        # ---------------- Internal state ----------------
        self.q = None
        self.goal_in_flight = False

        # Contact state
        self.contact_active = False
        self.contact_since = None
        self.no_contact_since = self.get_clock().now()

        # FSM
        self.state = "IDLE"
        self.retract_remaining = 0.0
        self.spiral = SpiralGenerator(self.search_spacing, self.search_max_r, self.search_dtheta)

        # TF-based XY search
        self.search_origin_xy = None  # (x0,y0) at start of a SEARCH_XY segment
        self.target_xy = (0.0, 0.0)
        self.just_finished_xy = False

        # "upright once"
        self.did_upright_once = False

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pinocchio model
        robot_description = self.declare_parameter("robot_description", "").value
        robot_description_file = self.declare_parameter("robot_description_file", "").value
        if (not robot_description) and robot_description_file:
            with open(robot_description_file, "r") as f:
                robot_description = f.read()
        if not robot_description:
            raise RuntimeError("robot_description is empty; set robot_description_file to a URDF path.")

        self.pin_model = pin.buildModelFromXML(robot_description)
        self.pin_data = self.pin_model.createData()

        self.qidx = {}
        for jn in self.joint_names:
            jid = self.pin_model.getJointId(jn)
            if jid == 0:
                raise RuntimeError(f"Joint '{jn}' not found in URDF.")
            self.qidx[jn] = self.pin_model.joints[jid].idx_q

        self.ee_frame_id = self.pin_model.getFrameId(self.ee_frame)
        if self.ee_frame_id >= len(self.pin_model.frames):
            raise RuntimeError(f"Frame '{self.ee_frame}' not found in URDF frames.")

        # ROS I/O
        self.create_subscription(JointState, self.joint_states_topic, self._on_joint_state, 20)
        self.create_subscription(Contacts, self.contact_topic, self._on_contacts, 50)
        self.ajt = ActionClient(self, FollowJointTrajectory, self.fjt_action)

        # main loop
        self.timer = self.create_timer(1.0 / self.timer_hz, self._step)

        self.get_logger().info("peg_insertion_fsm started.")

    # ---------------- TF helpers (robust) ----------------
    def _can_tf(self) -> bool:
        return self.tf_buffer.can_transform(
            self.hole_frame, self.tip_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)
        )

    def _tip_xyz_in_hole(self):
        if not self._can_tf():
            return None
        tf = self.tf_buffer.lookup_transform(self.hole_frame, self.tip_frame, rclpy.time.Time())
        t = tf.transform.translation
        return (float(t.x), float(t.y), float(t.z))

    def _tip_z_in_hole(self):
        xyz = self._tip_xyz_in_hole()
        if xyz is None:
            return None
        return xyz[2]

    def _tip_xy_in_hole(self):
        xyz = self._tip_xyz_in_hole()
        if xyz is None:
            return None
        return (xyz[0], xyz[1])

    # ---------------- Joint state callback ----------------
    def _on_joint_state(self, msg: JointState):
        name_to_i = {n: i for i, n in enumerate(msg.name)}
        q = []
        for jn in self.joint_names:
            if jn not in name_to_i:
                return
            q.append(msg.position[name_to_i[jn]])
        self.q = np.array(q, dtype=float)

    # ---------------- Contact parsing/filtering ----------------
    @staticmethod
    def _entity_name(x) -> str:
        if x is None:
            return ""
        name = getattr(x, "name", None)
        if isinstance(name, str) and name:
            return name
        if isinstance(x, str):
            return x
        return str(x)

    def _contact_is_tip_contact(self, contact) -> bool:
        c1 = self._entity_name(getattr(contact, "collision1", None))
        c2 = self._entity_name(getattr(contact, "collision2", None))
        tok_col = str(self.tip_collision_token)
        tok_link = str(self.tip_link_token)
        return (tok_col in c1) or (tok_link in c1) or (tok_col in c2) or (tok_link in c2)

    def _contact_is_significant(self, contact) -> bool:
        depths = getattr(contact, "depths", [])
        max_depth = max([float(d) for d in depths], default=0.0)

        max_force = 0.0
        wrenches = getattr(contact, "wrenches", [])
        for w in wrenches:
            f = getattr(getattr(w, "body_1_wrench", None), "force", None)
            if f is None:
                continue
            fx = float(getattr(f, "x", 0.0))
            fy = float(getattr(f, "y", 0.0))
            fz = float(getattr(f, "z", 0.0))
            mag = math.sqrt(fx * fx + fy * fy + fz * fz)
            if mag > max_force:
                max_force = mag

        return (max_depth >= self.min_depth_m) or (max_force >= self.min_force_n)

    def _on_contacts(self, msg: Contacts):
        active = False
        for c in msg.contacts:
            if self._contact_is_tip_contact(c) and self._contact_is_significant(c):
                active = True
                break

        now = self.get_clock().now()
        if active and not self.contact_active:
            self.contact_since = now
        if (not active) and self.contact_active:
            self.no_contact_since = now
        self.contact_active = active

    # ---------------- Contact timing helpers ----------------
    def _contact_persistent(self) -> bool:
        if (not self.contact_active) or (self.contact_since is None):
            return False
        dt = (self.get_clock().now() - self.contact_since).nanoseconds * 1e-9
        return dt >= self.debounce_s

    def _no_contact_for(self, seconds: float) -> bool:
        dt = (self.get_clock().now() - self.no_contact_since).nanoseconds * 1e-9
        return dt >= seconds

    # ---------------- IK: DLS translation-only ----------------
    def _compute_dq_dls(self, dx, dy, dz):
        if self.q is None:
            return None

        qpin = np.zeros(self.pin_model.nq, dtype=float)
        for i, jn in enumerate(self.joint_names):
            qpin[self.qidx[jn]] = float(self.q[i])

        pin.forwardKinematics(self.pin_model, self.pin_data, qpin)
        pin.updateFramePlacements(self.pin_model, self.pin_data)

        J6 = pin.computeFrameJacobian(
            self.pin_model,
            self.pin_data,
            qpin,
            self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        xi = np.array([dx / self.dt_cmd, dy / self.dt_cmd, dz / self.dt_cmd, 0.0, 0.0, 0.0], dtype=float)
        lam = self.lambda_dls
        A = J6 @ J6.T + (lam * lam) * np.eye(6)
        dq_full = J6.T @ np.linalg.solve(A, xi) * self.dt_cmd

        dq = np.zeros(6, dtype=float)
        for i, jn in enumerate(self.joint_names):
            dq[i] = dq_full[self.qidx[jn]]

        return np.clip(dq, -self.dq_max, self.dq_max)

    # ---------------- FollowJointTrajectory micro goal ----------------
    def _send_micro_goal(self, q_target: np.ndarray) -> bool:
        if not self.ajt.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("FollowJointTrajectory server not available yet.")
            return False

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q_target.tolist()]
        sec = int(self.dt_cmd)
        nsec = int((self.dt_cmd - sec) * 1e9)
        pt.time_from_start.sec = sec
        pt.time_from_start.nanosec = nsec

        traj.points = [pt]
        goal.trajectory = traj

        self.goal_in_flight = True
        fut = self.ajt.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or (not goal_handle.accepted):
            self.goal_in_flight = False
            self.get_logger().error("Trajectory goal rejected.")
            return
        rfut = goal_handle.get_result_async()
        rfut.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self.goal_in_flight = False

    # ---------------- "upright wrist" goal ----------------
    def _send_upright_wrist_goal(self) -> bool:
        """
        Smooth wrist alignment: interpolate current -> target wrist over upright_duration_s
        using multiple points in one FollowJointTrajectory goal.
        """
        if self.q is None:
            return False
        if not self.ajt.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("FollowJointTrajectory server not available yet.")
            return False

        upright_duration_s = float(self.declare_parameter("upright_duration_s", 1.0).value)
        upright_points = int(self.declare_parameter("upright_points", 10).value)

        q_start = np.array(self.q, dtype=float)
        q_end = np.array(self.q, dtype=float)
        q_end[3] = float(self.upright_wrist[0])
        q_end[4] = float(self.upright_wrist[1])
        q_end[5] = float(self.upright_wrist[2])

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)

        pts = []
        for k in range(1, upright_points + 1):
            a = k / float(upright_points)  # 0..1
            qk = (1.0 - a) * q_start + a * q_end

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in qk.tolist()]

            t = a * upright_duration_s
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            pts.append(pt)

        traj.points = pts
        goal.trajectory = traj

        self.goal_in_flight = True
        fut = self.ajt.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)
        return True


    # ---------------- Pre-insert check ----------------
    def _close_to_pre_insert(self) -> bool:
        if not self.pre_insert_q or len(self.pre_insert_q) != 6 or self.q is None:
            return True
        err = np.abs(self.q - np.array(self.pre_insert_q, dtype=float))
        return bool(np.all(err <= self.pre_insert_tol))

    # ---------------- FSM step ----------------
    def _step(self):
        if self.q is None:
            return
        if self.goal_in_flight:
            return

        # IDLE -> MOVE_ABOVE or GUARDED_INSERT
        if self.state == "IDLE":
            self.state = "MOVE_ABOVE" if (self.pre_insert_q and len(self.pre_insert_q) == 6) else "GUARDED_INSERT"
            self.get_logger().info(f"FSM -> {self.state}")
            return

        # MOVE_ABOVE
        if self.state == "MOVE_ABOVE":
            self._send_micro_goal(np.array(self.pre_insert_q, dtype=float))
            if self._close_to_pre_insert():
                self.state = "CLEAR_WAIT"
                self.just_finished_xy = True  # so next is insert
                self.get_logger().info("FSM -> CLEAR_WAIT (after MOVE_ABOVE)")
            return

        # CLEAR_WAIT: require contact-free for clear_s
        if self.state == "CLEAR_WAIT":
            if self.contact_active:
                return
            if not self._no_contact_for(self.clear_s):
                return

            # If we just finished XY segment, next is insert
            if self.just_finished_xy:
                self.just_finished_xy = False
                self.state = "GUARDED_INSERT"
                self.get_logger().info("FSM -> GUARDED_INSERT (contact cleared)")
                return

            # Otherwise after retract/orient, go search
            off = self.spiral.next()
            if off is None:
                self.state = "FAILURE"
                self.get_logger().info("FSM -> FAILURE (spiral exhausted)")
                return
            self.target_xy = off
            self.search_origin_xy = None
            self.state = "SEARCH_XY"
            self.get_logger().info(f"FSM -> SEARCH_XY target={self.target_xy}")
            return

        # GUARDED_INSERT
        if self.state == "GUARDED_INSERT":
            # Important ordering: retract must not be blocked by "clear" gates
            if self._contact_persistent():
                self.retract_remaining = self.retract_z
                self.state = "RETRACT"
                self.get_logger().info("FSM -> RETRACT (contact persistent)")
                return

            if self.contact_active:
                return
            if not self._no_contact_for(self.clear_s):
                return

            z = self._tip_z_in_hole()
            if z is None:
                self.get_logger().warn(f"TF not available for ({self.hole_frame} <- {self.tip_frame}). Waiting.")
                return

            if (z <= self.z_target) and self._no_contact_for(self.settle_s):
                self.go_up_remaining = self.go_up_distance
                self.state = "GO_UP"
                self.get_logger().info("FSM -> GO_UP (success, retracting)")
                return


            dq = self._compute_dq_dls(0.0, 0.0, self.dz_step)
            if dq is None:
                self.state = "FAILURE"
                self.get_logger().error("FSM -> FAILURE (IK failed)")
                return

            self._send_micro_goal(self.q + dq)
            return

        # RETRACT: move up by retract_z total (in small chunks)
        if self.state == "RETRACT":
            if self.retract_remaining <= 1e-9:
                # After retract complete, optionally do "upright" once, then clear-wait
                if self.enable_upright_on_first_retract and (not self.did_upright_once):
                    self.did_upright_once = True
                    self.state = "ORIENT_UPRIGHT"
                    self.get_logger().info("FSM -> ORIENT_UPRIGHT (first retract)")
                    return

                self.state = "CLEAR_WAIT"
                self.get_logger().info("FSM -> CLEAR_WAIT (after RETRACT)")
                return

            dz_chunk = min(self.retract_remaining, abs(self.dz_step))  # retract in same scale as insert
            dq = self._compute_dq_dls(0.0, 0.0, +dz_chunk)
            if dq is None:
                self.state = "FAILURE"
                self.get_logger().error("FSM -> FAILURE (IK failed on retract)")
                return

            self.retract_remaining -= dz_chunk
            self._send_micro_goal(self.q + dq)
            return

        # ORIENT_UPRIGHT: one joint-space wrist goal, then clear-wait
        if self.state == "ORIENT_UPRIGHT":
            ok = self._send_upright_wrist_goal()
            if ok:
                # after this goal completes, _step() will run again (goal_in_flight false)
                # then we immediately pick the next spiral target
                self.state = "POST_UPRIGHT"
                self.get_logger().info("FSM -> POST_UPRIGHT")
            return

        if self.state == "POST_UPRIGHT":
            # no extra waiting; start searching
            off = self.spiral.next()
            if off is None:
                self.state = "FAILURE"
                self.get_logger().info("FSM -> FAILURE (spiral exhausted)")
                return
            self.target_xy = off
            self.search_origin_xy = None
            self.state = "SEARCH_XY"
            self.get_logger().info(f"FSM -> SEARCH_XY target={self.target_xy}")
            return


        # SEARCH_XY: TF-based progress in hole_frame
        if self.state == "SEARCH_XY":
            if self.contact_active:
                self.retract_remaining = self.retract_z
                self.state = "RETRACT"
                self.get_logger().info("FSM -> RETRACT (contact during SEARCH_XY)")
                return

            if self.search_origin_xy is None:
                xy = self._tip_xy_in_hole()
                if xy is None:
                    self.get_logger().warn(f"TF not available for ({self.hole_frame} <- {self.tip_frame}) in SEARCH_XY.")
                    return
                self.search_origin_xy = xy

            xy = self._tip_xy_in_hole()
            if xy is None:
                self.get_logger().warn(f"TF not available for ({self.hole_frame} <- {self.tip_frame}) in SEARCH_XY.")
                return

            x0, y0 = self.search_origin_xy
            x, y = xy
            cx = x - x0
            cy = y - y0

            tx, ty = self.target_xy
            dx = tx - cx
            dy = ty - cy

            dx_step = float(np.clip(dx, -self.xy_step_limit, self.xy_step_limit))
            dy_step = float(np.clip(dy, -self.xy_step_limit, self.xy_step_limit))

            dq = self._compute_dq_dls(dx_step, dy_step, 0.0)
            if dq is None:
                self.state = "FAILURE"
                self.get_logger().error("FSM -> FAILURE (IK failed on search)")
                return

            if float(np.linalg.norm(dq)) < 1e-7:
                self.get_logger().warn("SEARCH_XY: dq ~ 0 (no motion). Try reducing lambda_dls or increasing dq_max.")
                return

            self._send_micro_goal(self.q + dq)

            # arrival tolerance: 0.5 mm
            if abs(dx) < 5e-4 and abs(dy) < 5e-4:
                self.just_finished_xy = True
                self.state = "CLEAR_WAIT"
                self.get_logger().info("FSM -> CLEAR_WAIT (after XY move)")
            return

            if self.state == "GO_UP":
                if self.go_up_remaining <= 1e-9:
                    self.state = "SUCCESS"
                    self.get_logger().info("FSM -> SUCCESS (done)")
                    return

                dz = min(self.go_up_remaining, abs(self.dz_step))  # step up in same increments
                dq = self._compute_dq_dls(0.0, 0.0, +dz)
                if dq is None:
                    self.state = "SUCCESS"
                    self.get_logger().warn("GO_UP: IK failed, stopping.")
                    return

                self.go_up_remaining -= dz
                self._send_micro_goal(self.q + dq)
                return


        # SUCCESS/FAILURE do nothing
        return


def main():
    rclpy.init()
    node = PegInsertionFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
