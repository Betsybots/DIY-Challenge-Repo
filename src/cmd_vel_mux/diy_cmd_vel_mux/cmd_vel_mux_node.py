"""
cmd_vel_mux_node — DIY Challenge 2026
═══════════════════════════════════════════════════════════════════════════════
PURPOSE
  Arbitrates command-velocity (Twist) from three competing sources — joystick,
  Nav2 planner, and an emergency-stop signal — and publishes a single safe
  /cmd_vel_safe that the motor driver always reads.

WHY THIS NODE EXISTS
  Without a mux the motor driver would have to choose between /cmd_vel_joy and
  /cmd_vel_nav itself, or one would silently overwrite the other. The mux makes
  the arbitration logic explicit, auditable, and reconfigurable at runtime.

DATA FLOW
    /cmd_vel_joy       ──┐
    /cmd_vel_nav       ──┤──► [CmdVelMuxNode] ──► /cmd_vel_safe ──► motor driver
    /cmd_vel_zone_nav  ──┤           │
    /estop_active      ──┘           └──► /mux_mode (monitoring)

TOPIC CONTRACT
  Subscribers:
    /cmd_vel_joy       (geometry_msgs/Twist) — teleop_twist_joy output
    /cmd_vel_nav       (geometry_msgs/Twist) — Nav2 FollowPath controller output
    /cmd_vel_zone_nav  (geometry_msgs/Twist) — zone_nav_manager BLIND_DRIVE output
    /estop_active      (std_msgs/Bool)       — latched advisory from diy_estop_controller

  Publishers:
    /cmd_vel_safe  (geometry_msgs/Twist) — goes to differential-drive node
    /mux_mode      (std_msgs/String)     — human-readable current mode; used by
                                           health_check.sh and RViz dashboards

OPERATING MODES
  JOYSTICK    — /cmd_vel_joy passes through; Nav2 output is ignored.
                Used for manual pilot override and competition safety walks.
  AUTONOMOUS  — /cmd_vel_nav passes through; joystick input is ignored.
                Normal competition run mode.
  BLIND_DRIVE — /cmd_vel_zone_nav passes through; Nav2 + joystick ignored.
                Active during tunnel dead-reckoning (zone_nav_manager sets this
                via SetParameters when entering/exiting BLIND_DRIVE state).
  ESTOP_LOCK  — Hard zero velocity published on every tick regardless of inputs.
                Cannot be exited by anything except /estop_active going False.

SAFETY RULES (enforced in code, not configurable)
  Rule 1: /estop_active = True → ESTOP_LOCK immediately. No exceptions.
  Rule 2: While in ESTOP_LOCK, publish zero. Never pass any input through.
  Rule 3: /cmd_vel_joy older than joy_staleness_timeout_s → publish zero.
  Rule 4: /cmd_vel_nav older than nav_staleness_timeout_s → publish zero.
  Rule 5: E-stop can only be cleared by /estop_active = False (from STM32).
          A ros2 param set targeting mode cannot override an active E-stop.

RUNTIME MODE SWITCH
  ros2 param set /cmd_vel_mux_node mode AUTONOMOUS
  ros2 param set /cmd_vel_mux_node mode JOYSTICK
  (ESTOP_LOCK cannot be selected manually — only triggered by the E-stop source)
═══════════════════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import SetParametersResult


# ─────────────────────────────────────────────────────────────────────────────
# Mode constants
# ─────────────────────────────────────────────────────────────────────────────
class Mode:
    """
    Namespace for the four valid operating modes of the mux.

    Using a class with class-level string attributes (rather than an Enum)
    keeps the mode values as plain strings that are directly compatible with
    ROS 2 parameter strings and the /mux_mode String topic.
    """
    JOYSTICK    = 'JOYSTICK'     # Manual pilot control via gamepad
    AUTONOMOUS  = 'AUTONOMOUS'   # Nav2 fully in charge
    BLIND_DRIVE = 'BLIND_DRIVE'  # Tunnel dead-reckoning — zone_nav_manager owns cmd_vel
    ESTOP_LOCK  = 'ESTOP_LOCK'   # Hard stop — only STM32 can release

    # Set used for O(1) membership checks in validation paths
    VALID = {JOYSTICK, AUTONOMOUS, BLIND_DRIVE, ESTOP_LOCK}

    # Modes that can be set manually (via ros2 param set or SetParameters RPC).
    # ESTOP_LOCK is intentionally excluded: it must only be entered via
    # /estop_active (hardware signal from the STM32).  Allowing it as a manual
    # param would permanently lock the robot until hardware releases the e-stop.
    MANUAL_SETTABLE = {JOYSTICK, AUTONOMOUS, BLIND_DRIVE}


# ─────────────────────────────────────────────────────────────────────────────
# Main node class
# ─────────────────────────────────────────────────────────────────────────────
class CmdVelMuxNode(Node):
    """
    Command velocity multiplexer with hard E-stop priority.

    Inherits from rclpy.node.Node — all ROS 2 communication (pubs, subs, timers,
    params) is managed through the parent class's lifecycle helpers.
    """

    def __init__(self):
        super().__init__('cmd_vel_mux_node')  # ROS 2 node name visible in ros2 node list

        # ── Declare ROS 2 parameters ──────────────────────────────────────────
        # Parameters declared here can be overridden via:
        #   1. A params YAML file passed to the launch file
        #   2. `ros2 param set` at runtime (if add_on_set_parameters_callback allows it)
        self.declare_parameter('mode', Mode.JOYSTICK)
        self.declare_parameter('joy_staleness_timeout_s', 2.0)    # joystick watchdog
        self.declare_parameter('nav_staleness_timeout_s', 1.0)    # nav watchdog
        self.declare_parameter('zone_staleness_timeout_s', 0.5)   # blind-drive watchdog
        self.declare_parameter('publish_rate_hz', 20.0)           # output rate

        # ── Read initial parameter values ─────────────────────────────────────
        self._current_mode  = self.get_parameter('mode').get_parameter_value().string_value
        self._joy_timeout   = self.get_parameter('joy_staleness_timeout_s').get_parameter_value().double_value
        self._nav_timeout   = self.get_parameter('nav_staleness_timeout_s').get_parameter_value().double_value
        self._zone_timeout  = self.get_parameter('zone_staleness_timeout_s').get_parameter_value().double_value
        pub_rate            = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # Guard against an invalid initial mode being passed via launch args or YAML.
        # Use MANUAL_SETTABLE (not VALID) so ESTOP_LOCK cannot be set as the startup
        # mode — if it were, the robot would be permanently locked because _estop_active
        # starts False and the _estop_cb release path checks `self._estop_active` first.
        if self._current_mode not in Mode.MANUAL_SETTABLE:
            self.get_logger().warn(
                f"Invalid initial mode '{self._current_mode}', defaulting to JOYSTICK")
            self._current_mode = Mode.JOYSTICK

        # Guard against a non-positive publish rate (division by zero below).
        if pub_rate <= 0.0:
            self.get_logger().warn(
                f"Invalid publish_rate_hz {pub_rate}, defaulting to 20.0")
            pub_rate = 20.0

        # Non-positive staleness timeouts would make every source permanently
        # stale, so the mux would only ever emit zeros.
        for attr, name, default in (
            ('_joy_timeout',  'joy_staleness_timeout_s',  2.0),
            ('_nav_timeout',  'nav_staleness_timeout_s',  1.0),
            ('_zone_timeout', 'zone_staleness_timeout_s', 0.5),
        ):
            if getattr(self, attr) <= 0.0:
                self.get_logger().warn(
                    f"Invalid {name} {getattr(self, attr)}, defaulting to {default}")
                setattr(self, attr, default)

        # ── Internal state ────────────────────────────────────────────────────
        # Hold the most-recently received Twist from each source.
        # Initialised to all-zeros (safe default: no motion).
        self._joy_twist   = Twist()   # latest joystick command
        self._nav_twist   = Twist()   # latest Nav2 command
        self._zone_twist  = Twist()   # latest zone_nav_manager BLIND_DRIVE command
        # Timestamps (rclpy.time.Time) for the staleness watchdog.
        # None means no message has been received yet on that topic.
        self._joy_stamp   = None
        self._nav_stamp   = None
        self._zone_stamp  = None
        # Mirror of /estop_active — set True by the E-stop callback.
        self._estop_active = False

        # ── Subscriptions ─────────────────────────────────────────────────────
        # Queue depth 10 is sufficient — we only care about the latest message,
        # and the publish timer fires at 20 Hz so the queue drains quickly.
        self.create_subscription(Twist, '/cmd_vel_joy',       self._joy_cb,   10)
        self.create_subscription(Twist, '/cmd_vel_nav',       self._nav_cb,   10)
        self.create_subscription(Twist, '/cmd_vel_zone_nav',  self._zone_cb,  10)
        self.create_subscription(Bool,  '/estop_active',      self._estop_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        # /cmd_vel_safe — the motor driver subscribes here; queue=10 is fine
        self._pub      = self.create_publisher(Twist,  '/cmd_vel_safe', 10)
        # /mux_mode — String status topic for monitoring; picked up by health_check.sh
        self._mode_pub = self.create_publisher(String, '/mux_mode',     10)

        # ── Output timer ──────────────────────────────────────────────────────
        # Fires at pub_rate Hz (default 20 Hz). Even if no new input arrives,
        # the motor driver receives a zero-velocity heartbeat so it does not
        # timeout and stall on its own.
        period = 1.0 / pub_rate
        self.create_timer(period, self._publish_cb)

        # ── Runtime parameter change hook ─────────────────────────────────────
        # Allows `ros2 param set /cmd_vel_mux_node mode AUTONOMOUS`
        # The callback validates the request BEFORE storing the new value,
        # so an E-stop cannot be overridden via a param change.
        self.add_on_set_parameters_callback(self._param_cb)

        self.get_logger().info(
            f"cmd_vel_mux_node started. Initial mode: {self._current_mode}")

    # ─────────────────────────────────────────────────────────────────────────
    # Subscriber callbacks — simply store the latest value + arrival timestamp
    # ─────────────────────────────────────────────────────────────────────────

    # Runtime-adjustable staleness timeouts → the attribute each one drives.
    _TIMEOUT_PARAMS = {
        'joy_staleness_timeout_s':  '_joy_timeout',
        'nav_staleness_timeout_s':  '_nav_timeout',
        'zone_staleness_timeout_s': '_zone_timeout',
    }

    def _joy_cb(self, msg: Twist):
        """Cache the latest joystick Twist and record arrival time for staleness check."""
        self._joy_twist = msg
        self._joy_stamp = self.get_clock().now()

    def _nav_cb(self, msg: Twist):
        """Cache the latest Nav2 Twist and record arrival time for staleness check."""
        self._nav_twist = msg
        self._nav_stamp = self.get_clock().now()

    def _zone_cb(self, msg: Twist):
        """Cache the latest zone_nav_manager Twist (BLIND_DRIVE commands)."""
        self._zone_twist = msg
        self._zone_stamp = self.get_clock().now()

    def _estop_cb(self, msg: Bool):
        """
        React to E-stop state changes from diy_estop_controller.

        Transition logic:
          False → True  : Immediately enter ESTOP_LOCK. Log a warning.
          True  → False : Exit ESTOP_LOCK; revert to JOYSTICK (safest default).
                          The operator must consciously switch to AUTONOMOUS
                          after confirming the course is clear.
        """
        # Transition into E-stop: log once to avoid spam
        if msg.data and self._current_mode != Mode.ESTOP_LOCK:
            self.get_logger().warn('E-stop activated — switching to ESTOP_LOCK')

        # Transition out of E-stop: revert to safest mode (JOYSTICK)
        if not msg.data and self._estop_active:
            self.get_logger().info('E-stop released — reverting to JOYSTICK')
            self._current_mode = Mode.JOYSTICK

        # Store the new flag value, then enforce ESTOP_LOCK if needed
        self._estop_active = msg.data
        if self._estop_active:
            self._current_mode = Mode.ESTOP_LOCK

    # ─────────────────────────────────────────────────────────────────────────
    # Publish loop — runs at pub_rate Hz via the timer
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_cb(self):
        """
        Decide which velocity command to forward and publish it.

        Decision tree (evaluated top-to-bottom; first match wins):
          1. ESTOP_LOCK or _estop_active → zero velocity (hard stop)
          2. JOYSTICK mode:
               a. No message received yet        → zero velocity
               b. Message older than joy_timeout → zero velocity + warning
               c. Otherwise                      → forward joystick Twist
          3. AUTONOMOUS mode:
               a. No message received yet        → zero velocity
               b. Message older than nav_timeout → zero velocity + warning
               c. Otherwise                      → forward Nav2 Twist

        Note: `out = Twist()` initialises to all-zeros, so any early return
        automatically publishes a stop command.
        """
        out = Twist()         # default: all-zero (safe stop)
        now = self.get_clock().now()

        # ── Branch 1: E-stop takes absolute priority ──────────────────────────
        if self._current_mode == Mode.ESTOP_LOCK or self._estop_active:
            # `out` remains all-zeros — do nothing more; fall through to publish
            pass

        # ── Branch 2: Joystick mode ───────────────────────────────────────────
        elif self._current_mode == Mode.JOYSTICK:
            if self._joy_stamp is None:
                pass
            elif (now - self._joy_stamp) > Duration(seconds=self._joy_timeout):
                self.get_logger().warning(
                    'Joystick topic stale — publishing zero velocity',
                    throttle_duration_sec=2.0, clock=self.get_clock())
            else:
                out = self._joy_twist

        # ── Branch 3: Autonomous mode ─────────────────────────────────────────
        elif self._current_mode == Mode.AUTONOMOUS:
            if self._nav_stamp is None:
                pass
            elif (now - self._nav_stamp) > Duration(seconds=self._nav_timeout):
                self.get_logger().warning(
                    'Nav2 cmd_vel stale — publishing zero velocity',
                    throttle_duration_sec=1.0, clock=self.get_clock())
            else:
                out = self._nav_twist

        # ── Branch 4: Blind drive mode ────────────────────────────────────────
        # zone_nav_manager publishes fixed-heading commands on /cmd_vel_zone_nav.
        # Nav2 and joystick are both ignored — only the zone_nav_manager drives.
        # Short staleness timeout (0.5 s): if zone_nav_manager dies during the
        # tunnel, robot stops rather than holding last command.
        elif self._current_mode == Mode.BLIND_DRIVE:
            if self._zone_stamp is None:
                self.get_logger().warning(
                    'BLIND_DRIVE active but /cmd_vel_zone_nav not publishing yet',
                    throttle_duration_sec=2.0, clock=self.get_clock())
            elif (now - self._zone_stamp) > Duration(seconds=self._zone_timeout):
                self.get_logger().warning(
                    'BLIND_DRIVE: /cmd_vel_zone_nav stale — publishing zero velocity',
                    throttle_duration_sec=0.5, clock=self.get_clock())
            else:
                out = self._zone_twist

        # ── Publish the resolved velocity ─────────────────────────────────────
        self._pub.publish(out)

        # Also publish the current mode name so monitoring tools can read it
        mode_msg      = String()
        mode_msg.data = self._current_mode
        self._mode_pub.publish(mode_msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Parameter change callback — validates and applies runtime param updates
    # ─────────────────────────────────────────────────────────────────────────

    def _param_cb(self, params) -> SetParametersResult:
        """
        Validate parameter updates before they are applied by ROS 2 internals.

        This is called by rclpy BEFORE storing the new value, so we can reject
        unsafe changes here without a TOCTOU race condition.

        Rejection cases:
          - mode is not in Mode.MANUAL_SETTABLE (ESTOP_LOCK cannot be set manually)
          - mode change requested while E-stop is active (safety lock)
        """
        # Phase 1 — validate everything. Nothing is applied yet, so a request
        # that is rejected halfway through cannot leave us half-updated.
        pending = []
        for param in params:
            if param.name == 'mode':
                new_mode = param.value

                # Reject unknown mode strings early
                if new_mode not in Mode.MANUAL_SETTABLE:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"Invalid mode '{new_mode}'. "
                            f"Manually settable modes: {sorted(Mode.MANUAL_SETTABLE)}. "
                            "ESTOP_LOCK can only be set by the hardware e-stop signal."
                        ))

                # Prevent bypassing ESTOP_LOCK via a parameter change
                if self._estop_active and new_mode != Mode.ESTOP_LOCK:
                    return SetParametersResult(
                        successful=False,
                        reason='E-stop is active — cannot change mode out of ESTOP_LOCK')

                pending.append(('mode', new_mode))

            elif param.name in self._TIMEOUT_PARAMS:
                # These were previously accepted but never applied, so a runtime
                # change silently did nothing.
                if param.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be > 0 (got {param.value})")
                pending.append((param.name, param.value))

            elif param.name == 'publish_rate_hz':
                # The timer period is fixed at construction; changing this at
                # runtime would be silently ignored, so reject it outright.
                return SetParametersResult(
                    successful=False,
                    reason='publish_rate_hz cannot be changed at runtime')

        # Phase 2 — apply.
        for name, value in pending:
            if name == 'mode':
                self.get_logger().info(
                    f"Mode changed: {self._current_mode} → {value}")
                self._current_mode = value
            else:
                setattr(self, self._TIMEOUT_PARAMS[name], value)
                self.get_logger().info(f"{name} changed to {value}")

        return SetParametersResult(successful=True)


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 entry point
# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    """
    Standard rclpy spin entry point.
    Handles clean shutdown on Ctrl-C (KeyboardInterrupt) and always calls
    node.destroy_node() + rclpy.shutdown() to release DDS resources.
    """
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)       # blocks until shutdown is requested
    except KeyboardInterrupt:
        pass                   # normal Ctrl-C shutdown — not an error
    finally:
        node.destroy_node()    # clean up subscriptions, publishers, timers
        # rclpy.shutdown() raises RCLError if the context is already down,
        # which happens when the default SIGINT handler got there first.
        if rclpy.ok():
            rclpy.shutdown()   # release DDS middleware


if __name__ == '__main__':
    main()
