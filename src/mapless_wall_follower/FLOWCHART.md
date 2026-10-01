# Mapless Ackermann Wall-Follower Flowchart

```mermaid
flowchart TD
    Start([Control cycle at 20 Hz]) --> Cloud{Point cloud received?}

    Cloud -- No --> Wait[WAITING_FOR_CLOUD<br/>Command stop]
    Cloud -- Yes --> Enabled{Controller enabled?}

    Enabled -- No --> Disabled[DISABLED<br/>Command stop]
    Enabled -- Yes --> Fresh{Cloud age less than 0.30 s?}

    Fresh -- No --> Stale[STALE_CLOUD<br/>Command stop]
    Fresh -- Yes --> Extract[Filter point cloud<br/>and extract left and right points]

    Extract --> Fit[Fit local lines to walls<br/>max_x = 1.50 m]
    Fit --> Validate[Validate point counts,<br/>inliers, RMS, and wall side]
    Validate --> Pair{Both wall fits initially valid?}

    Pair -- Yes --> Width{Corridor width<br/>between 0.70 and 1.40 m?}
    Width -- Yes --> KeepBoth[Keep both walls]
    Width -- No --> SelectBest[Set pair_inconsistent=true<br/>Keep fit with more inliers<br/>Use lower RMS on tie]

    Pair -- No --> Missing[One or both walls missing]
    Missing --> Recent{Recent valid wall<br/>within 0.20 s?}
    Recent -- Yes --> Hold[Temporarily reuse wall<br/>Set left_held or right_held]
    Recent -- No --> Current[Use current valid walls only]

    SelectBest --> FinalWalls
    KeepBoth --> FinalWalls
    Hold --> Recheck[Recheck corridor consistency]
    Current --> FinalWalls
    Recheck --> FinalWalls{Final wall state}

    FinalWalls --> Clearance{Predicted path clearance<br/>at most 0.30 m?}

    Clearance -- Yes --> Attempts{Recovery attempts remaining?}
    Attempts -- No --> Emergency[EMERGENCY_FRONT_STOP]
    Attempts -- Yes --> Brake[RECOVERY_BRAKING<br/>Decelerate to zero]
    Brake --> Pause1[Pause 0.30 s]
    Pause1 --> Reverse[RECOVERY_REVERSING<br/>Reverse 0.08 m and steer away]
    Reverse --> Settle[RECOVERY_SETTLING<br/>Stop and pre-steer]
    Settle --> Pause2[Pause 0.30 s]
    Pause2 --> Start

    Clearance -- No --> Walls{Which walls are valid?}

    Walls -- Both --> Center[CENTERING<br/>Follow corridor center]
    Walls -- Left only --> Left[LEFT_WALL<br/>Follow left wall]
    Walls -- Right only --> Right[RIGHT_WALL<br/>Follow right wall]
    Walls -- Neither --> NoWall[NO_WALL_DETECTED<br/>Command stop]

    Center --> CenterSpeed[Initial ceiling<br/>straight_speed = 1.50 m/s]
    Left --> TurnSpeed[Initial ceiling<br/>turn_speed = 0.35 m/s]
    Right --> TurnSpeed

    CenterSpeed --> Limits[Apply curve-speed,<br/>clearance-speed, and yaw limits]
    TurnSpeed --> Limits

    Limits --> Floor[Apply min_speed = 0.20 m/s<br/>while driving]
    Floor --> RateLimit[Limit acceleration,<br/>deceleration, and steering rate]
    RateLimit --> Publish[Publish Twist command]
    Publish --> Start

    NoWall --> Start
    Wait --> Start
    Disabled --> Start
    Stale --> Start
    Emergency --> Start
```

## Wall Selection

| Detected walls | Controller behavior |
|---|---|
| Both valid and consistent | `CENTERING` |
| Both valid but inconsistent | Keep the higher-quality wall |
| Left only | `LEFT_WALL` |
| Right only | `RIGHT_WALL` |
| Brief detection dropout | Hold the recent wall for `0.20 s` |
| Neither valid after the hold | `NO_WALL_DETECTED` and stop |
| Path clearance at most `0.30 m` | Run the recovery sequence |

When a wall pair is inconsistent, the fit with more inliers is retained. If
both fits have the same number of inliers, the fit with the lower RMS error is
retained.
