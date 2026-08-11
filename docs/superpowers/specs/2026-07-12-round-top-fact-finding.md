# Round-Top Fact-Finding Results

## Decision

The mandatory Q1-Q4 gate passes for offline `round_top` implementation. Live
arm motion remains staged and is not authorized by these measurements alone.

## Q1 - Static Object Geometry

Values are axis-aligned `textured.obj` bounds in object coordinates, in metres.

| Object | Center | Bounding-box size |
| --- | --- | --- |
| apple | `[0.000859, -0.003784, 0.035552]` | `[0.075448, 0.074871, 0.071889]` |
| lemon | `[-0.010579, 0.021654, 0.026274]` | `[0.060588, 0.059299, 0.053017]` |
| peach | `[-0.014270, 0.005633, 0.029108]` | `[0.062123, 0.062632, 0.058645]` |
| pear | `[-0.033320, 0.017995, 0.032652]` | `[0.066546, 0.100455, 0.065663]` |
| orange | `[-0.006935, -0.018360, 0.035428]` | `[0.072158, 0.073986, 0.071352]` |
| plum | `[-0.007818, 0.019424, 0.026223]` | `[0.057190, 0.054941, 0.053040]` |
| baseball | `[-0.010081, -0.048245, 0.036077]` | `[0.073078, 0.073712, 0.072563]` |
| tennis_ball | `[0.008212, -0.044278, 0.033132]` | `[0.066975, 0.067030, 0.066457]` |
| racquetball | `[-0.009053, -0.122232, 0.027596]` | `[0.055778, 0.056056, 0.055574]` |

## Q2 - Apple Candidate Coverage

The real library at `/home/ws/grasps/013_apple` contains 5,013 poses. With the
provisional 20-degree tool-`+Z` to world-down limit, 1,382 poses remain.

| Metric | Min | P10 | Median | P90 | Max |
| --- | ---: | ---: | ---: | ---: | ---: |
| Approach angle (deg) | 0.226 | 3.476 | 9.166 | 16.800 | 19.997 |
| Geometry-center TCP-plane offset (m) | 0.000062 | 0.000587 | 0.001677 | 0.003712 | 0.007310 |
| TCP height relative to center / bbox height | 0.0071 | 0.0464 | 0.0805 | 0.1337 | 0.1840 |

Decision: pass. The library has ample centered vertical candidates; V1 does
not need an offline geometric candidate supplement.

## Q3 - Fingertip Geometry

The active source Xacro selects `finger_type="default"` and places TCP at
`0.18 m` from `tool0`. The default URDF gripper-base offsets total about
`0.041 m`, so TCP is about `0.139 m` beyond the gripper base. MuJoCo places its
pinch site at `0.145 m` from the gripper base, a roughly 6 mm model difference.

At simulated open and closed equilibria, the conservative lowest moving-finger
geometry is respectively `-0.10295 m` and `-0.10385 m` along TCP Z. V1 uses
`-0.104 m` as the conservative TCP-to-lowest-finger value.

The generated MoveIt URDF containing `tool0_to_tcp=0.303 m` represents the
`custom_160` variant and is stale for the active default-finger source path.

## Q4 - Command And Opening Mapping

The application command uses radians: `0.0` is open and `0.79` is closed. The
sim adapter maps `0..0.8 rad` monotonically to Robotiq `0..255`; MuJoCo maps
the same direction to its finger actuator.

At the open equilibrium, pad center separation is `0.09316 m`. Each pad has a
4 mm half-thickness along the closing axis, giving an effective inside opening
of about `0.08516 m`. Apple requires the smaller horizontal bbox dimension,
`0.07487 m`, leaving about `0.01029 m` margin.

Decision: pass. The margin exceeds the required 5 mm staged-live threshold.
