# Health of map_localizer's map->odom correction, published each time ICP
# attempts an alignment (at the node's configured update_hz).
std_msgs/Header header

# True once ICP has locked on at least once. Can still be true while
# consecutive_align_failures > 0 -- that means map->odom is holding the last
# accepted (potentially stale) correction rather than tracking a fresh one.
bool localized

# Consecutive align() failures since the last successful alignment (or since
# startup). Non-zero means map->odom is not currently being refreshed.
int32 consecutive_align_failures

# Most recent align() fitness scores (lower is better). -1.0 if that ICP
# stage never ran (e.g. no map loaded yet, or the rough stage already failed).
float64 rough_fitness
float64 refine_fitness
