#!/usr/bin/env bash
# Switch cmd_vel_mux mode at runtime.
# Usage:  ./set_mux_mode.sh JOYSTICK|AUTONOMOUS|BLIND_DRIVE
# Note:   ESTOP_LOCK cannot be set manually — hardware only.
MODE="${1:-}"
if [[ -z "$MODE" ]]; then
    echo "Usage: $0 JOYSTICK|AUTONOMOUS|BLIND_DRIVE"
    echo "Current mode:"
    ros2 topic echo --once --timeout 2 /mux_mode std_msgs/msg/String 2>/dev/null | grep data || echo "  (timeout)"
    exit 1
fi
case "$MODE" in
    JOYSTICK|AUTONOMOUS|BLIND_DRIVE)
        echo "Setting mux mode → $MODE"
        ros2 param set /cmd_vel_mux_node mode "$MODE"
        ;;
    ESTOP_LOCK)
        echo "ERROR: ESTOP_LOCK cannot be set manually. Use the hardware e-stop button."
        exit 1
        ;;
    *)
        echo "ERROR: Unknown mode '$MODE'. Valid: JOYSTICK, AUTONOMOUS, BLIND_DRIVE"
        exit 1
        ;;
esac
