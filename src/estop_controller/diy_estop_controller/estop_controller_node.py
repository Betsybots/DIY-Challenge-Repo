"""
estop_controller_node — DIY Challenge 2026
═══════════════════════════════════════════════════════════════════════════════
PURPOSE
  Creates a ROS 2–visible mirror of the STM32 hardware E-stop state so that
  software nodes (Nav2, cmd_vel_mux) can react gracefully to a stop event
  without polling hardware registers directly.

ARCHITECTURE NOTE — WHY ADVISORY ONLY?
  The actual motor-cut is wired in hardware on the STM32 (Rule 1.2.7 of the
  competition rulebook). The STM32 does NOT wait for a ROS 2 message to stop
  the robot — it cuts motors immediately when the E-stop button is pressed.
  This node only mirrors that state to software so Nav2 can pause, plans can
  be discarded, and cmd_vel_mux can enter ESTOP_LOCK to prevent stale commands
  from re-driving the robot when the stop is released.

DATA FLOW
    /stm32/estop_active ──┐
                          ├──► [EStopControllerNode] ──► /estop_active  (latched)
    /stm32/heartbeat    ──┘                          └──► /estop_reason

FAIL-SAFE DESIGN
  The node fires an E-stop if ANY of the following is true:
    1. /stm32/estop_active carries True
    2. /stm32/heartbeat has not arrived within heartbeat_timeout_ms (default 800 ms)
    3. /stm32/heartbeat has NEVER been received (micro-ROS agent not started)
  Only condition 1 clearing AND a fresh heartbeat will allow the output
  /estop_active to go False. Any ambiguity resolves to E-stop active.

LATCHED QoS ON /estop_active
  The publisher uses TRANSIENT_LOCAL durability (equivalent to "latched" in
  ROS 1). This means any node that subscribes AFTER this node has already
  published will immediately receive the most-recent value on connection,
  rather than waiting for the next publish cycle. This is critical for Nav2
  which may start up after the E-stop controller has already declared a state.

TOPIC CONTRACT
  Subscribers:
    /stm32/estop_active  (std_msgs/Bool)   — micro-ROS bridge from STM32
    /stm32/heartbeat     (std_msgs/Bool)   — periodic watchdog pulse from STM32

  Publishers:
    /estop_active        (std_msgs/Bool, TRANSIENT_LOCAL) — current stop state
    /estop_reason        (std_msgs/String)                — diagnostic reason

PARAMETERS
  heartbeat_timeout_ms  (int,   default 800)   — watchdog window in milliseconds
  publish_rate_hz       (float, default 20.0)  — /estop_active publish frequency
═══════════════════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from std_msgs.msg import Bool, String


# ─────────────────────────────────────────────────────────────────────────────
# Main node class
# ─────────────────────────────────────────────────────────────────────────────
class EStopControllerNode(Node):
    """
    ROS-side mirror of the STM32 E-stop state.

    See module docstring for full architecture description.
    """

    def __init__(self):
        super().__init__('estop_controller')  # node name in ros2 node list

        # ── Declare parameters (overridable via launch YAML or ros2 param set) ─
        self.declare_parameter('heartbeat_timeout_ms', 800)   # watchdog window
        self.declare_parameter('publish_rate_hz', 20.0)        # output rate

        # ── Read parameter values ─────────────────────────────────────────────
        timeout_ms = self.get_parameter(
            'heartbeat_timeout_ms').get_parameter_value().integer_value
        pub_rate = self.get_parameter(
            'publish_rate_hz').get_parameter_value().double_value

        # Convert ms → rclpy.time.Duration for easy comparison with clock deltas
        self._timeout = Duration(nanoseconds=timeout_ms * 1_000_000)

        # ── Internal state ────────────────────────────────────────────────────
        # _stm32_estop: mirrors the last seen /stm32/estop_active value
        self._stm32_estop    = False
        # _last_heartbeat: rclpy.Time of the last received heartbeat pulse.
        #   None until the first heartbeat arrives (used to detect "never received")
        self._last_heartbeat = None
        # _watchdog_fired: debounce flag — prevents repeated ERROR logs when the
        #   watchdog condition persists across multiple publish cycles
        self._watchdog_fired = False

        # ── Subscriptions ─────────────────────────────────────────────────────
        # Both topics are published by the micro-ROS agent bridging the STM32.
        # If micro-ROS is not running, _last_heartbeat stays None and watchdog fires.
        self.create_subscription(
            Bool, '/stm32/estop_active', self._stm32_estop_cb, 10)
        self.create_subscription(
            Bool, '/stm32/heartbeat', self._heartbeat_cb, 10)

        # ── Publisher with TRANSIENT_LOCAL (latched) QoS ──────────────────────
        # TRANSIENT_LOCAL means the last published message is stored by the
        # middleware and delivered to any new subscriber immediately on connection.
        # This is critical: Nav2 or cmd_vel_mux might start after this node and
        # must immediately know if E-stop is active.
        from rclpy.qos import QoSProfile, DurabilityPolicy
        latched_qos = QoSProfile(
            depth=1,                                     # only need the latest state
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # deliver on late-join
        )
        self._estop_pub  = self.create_publisher(Bool,   '/estop_active',  latched_qos)
        self._reason_pub = self.create_publisher(String, '/estop_reason',  10)

        # ── Periodic publish timer ────────────────────────────────────────────
        # Publishes the current E-stop state at pub_rate Hz so subscribers
        # always have a fresh value even if the STM32 state has not changed.
        self.create_timer(1.0 / pub_rate, self._publish_cb)

        # Warn on startup so anyone watching the logs knows to expect this
        self.get_logger().warn(
            'estop_controller started. Waiting for STM32 heartbeat on '
            '/stm32/heartbeat — if micro-ROS agent is not running, '
            'estop will fire after %d ms.' % timeout_ms)

    # ─────────────────────────────────────────────────────────────────────────
    # Subscriber callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _stm32_estop_cb(self, msg: Bool):
        """
        React to explicit E-stop flag from the STM32.

        Logs a single transition message on each edge change (False→True and
        True→False) to avoid flooding the log with repeated identical messages.
        Does NOT publish here — publication happens in _publish_cb so the
        output rate stays consistent regardless of input message rate.
        """
        # Log E-stop activation edge (False → True)
        if msg.data and not self._stm32_estop:
            self.get_logger().error('STM32 reported E-stop active')

        # Log E-stop clearance edge (True → False)
        if not msg.data and self._stm32_estop:
            self.get_logger().info('STM32 reported E-stop cleared')

        # Store the new value for use in the publish loop
        self._stm32_estop = msg.data

    def _heartbeat_cb(self, msg: Bool):
        """
        Record the arrival time of the latest STM32 heartbeat pulse.

        The actual heartbeat content (msg.data) is ignored — the presence of
        the message is sufficient to reset the watchdog timer. If the STM32
        halts or micro-ROS drops, no messages arrive and the timer expires.
        """
        # Stamp the arrival time; the publish loop compares this to now
        self._last_heartbeat = self.get_clock().now()

        # If the watchdog had fired, log the recovery so the operator knows
        if self._watchdog_fired:
            self.get_logger().info(
                'STM32 heartbeat resumed — watchdog cleared')
            self._watchdog_fired = False  # reset debounce flag

    # ─────────────────────────────────────────────────────────────────────────
    # Publish loop — runs at pub_rate Hz
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_cb(self):
        """
        Evaluate all E-stop conditions and publish the resolved state.

        Evaluation order (fail-safe — any True wins):
          1. STM32 explicit flag (_stm32_estop)
          2. Watchdog: heartbeat never received (_last_heartbeat is None)
          3. Watchdog: heartbeat too old (now - _last_heartbeat > _timeout)

        /estop_reason is only published when active to avoid log noise.
        /estop_active is published every cycle regardless.
        """
        now = self.get_clock().now()
        watchdog_fired = False
        reason = ''

        # ── Watchdog evaluation ───────────────────────────────────────────────
        if self._last_heartbeat is None:
            # The micro-ROS agent has not started or the STM32 has not booted.
            # Fire immediately — do not wait for the timeout window.
            watchdog_fired = True
            reason = 'WATCHDOG: No STM32 heartbeat received yet'

        elif (now - self._last_heartbeat) > self._timeout:
            # A heartbeat was received previously but went silent.
            # Compute the actual elapsed time for a helpful error message.
            watchdog_fired = True
            reason = 'WATCHDOG: STM32 heartbeat lost (>{:.0f} ms)'.format(
                self._timeout.nanoseconds / 1_000_000)

        # ── Log the first occurrence of watchdog firing ───────────────────────
        # _watchdog_fired is a debounce flag — we only log once per event
        if watchdog_fired and not self._watchdog_fired:
            self.get_logger().error(reason)
            self._watchdog_fired = True

        # ── Resolve final E-stop state (fail-safe: OR of all conditions) ─────
        active = self._stm32_estop or watchdog_fired

        # ── Publish /estop_active ─────────────────────────────────────────────
        estop_msg      = Bool()
        estop_msg.data = active
        self._estop_pub.publish(estop_msg)

        # ── Publish /estop_reason when active ─────────────────────────────────
        if active:
            reason_msg = String()
            if self._stm32_estop:
                # STM32 explicitly set the flag — hardware stop path
                reason_msg.data = 'STM32_ESTOP: hardware E-stop active'
            else:
                # Watchdog — heartbeat-based path
                reason_msg.data = reason
            self._reason_pub.publish(reason_msg)


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 entry point
# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    """
    Standard rclpy spin loop entry point.
    Registers a clean shutdown handler for Ctrl-C and always releases DDS
    resources via destroy_node() + rclpy.shutdown().
    """
    rclpy.init(args=args)
    node = EStopControllerNode()
    try:
        rclpy.spin(node)       # blocks until Ctrl-C or rclpy.shutdown()
    except KeyboardInterrupt:
        pass                   # clean exit — not an error condition
    finally:
        node.destroy_node()    # release pub/sub/timer resources
        rclpy.shutdown()       # shut down DDS middleware


if __name__ == '__main__':
    main()
