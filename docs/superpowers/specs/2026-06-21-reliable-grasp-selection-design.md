# Reliable Grasp Selection Design

## Goal

Select a grasp candidate that is likely to be physically executable, rather
than selecting only the most top-down pose. The first supported object is
`tomato_soup_can`; the design must extend to the other provided YCB objects
without duplicating the selection pipeline.

The course completion target remains: lift the selected object 20 cm above
the table and hold it for 5 seconds.

## Data and Frames

- Stable grasps are loaded from one object directory, such as
  `Data/grasps/005_tomato_soup_can` locally or `~/Data/grasps/005_tomato_soup_can`
  on the workstation.
- Each `.npz` archive has a `poses` array of shape `(N, 4, 4)` in object
  coordinates. The observed tomato-can data has 51 archives: 50 contain 100
  poses and one contains 2,391, for 7,391 candidates total.
- FoundationPose supplies `T_camera_object`; TF supplies `T_world_camera`.
  Each candidate is transformed as:

  ```text
  T_world_grasp = T_world_camera @ T_camera_object @ T_object_grasp
  ```

- The grasp dataset defines the grasp-frame `+Z` axis as the approach
  direction. A top-down grasp therefore has world-frame `+Z` close to
  world-frame `-Z`.

## Selection Pipeline

### 1. Candidate loading

The loader reads every `.npz` file in the selected object's grasp directory,
validates that `poses` has shape `(N, 4, 4)`, and concatenates all candidates.
The grasp-root directory is configurable, so the same code can use local data
or the workstation copy.

### 2. Geometry ranking

For each candidate, the selector transforms it into the world frame and
computes:

- approach angle: angle between grasp `+Z` and world `-Z`;
- grasp position; and
- a deterministic score based on the approach angle.

The first policy ranks top-down poses first. It returns an ordered top-K list,
not a single pose, so a later verifier can reject an infeasible candidate and
continue with the next one.

### 3. Motion verification

On the workstation, candidates are checked in score order. A candidate is
accepted only when all three motions succeed without collision:

1. current pose to pre-grasp;
2. pre-grasp to grasp; and
3. grasp to a lift pose 20 cm higher in world Z.

The first accepted candidate is executed. If no candidate is accepted, the
system reports a controlled failure with rejection counts by stage.

### 4. Execution and metrics

The executor closes the gripper, lifts the object 20 cm in world Z, and holds
the lift pose for 5 seconds. One record is emitted per attempt with candidate
count, planning outcomes, collision outcomes, selection time, execution time,
and lift success.

## Extension to Other Objects

The pipeline remains shared. A configuration maps normalized object names to
their YCB grasp directory and a category policy:

- cans: top-down preferred;
- boxes: top-down preferred, then side approach;
- round objects: centered top-down preferred; and
- elongated objects: side approach preferred.

Category policies alter ranking only. Motion verification remains the final
authority for every object and scene.

## First Implementation Slice

Implement the pure-Python candidate loading and ranking layer first. It has no
ROS or MoveIt dependency and is tested locally against synthetic transforms and
the downloaded tomato-can data. Integrate the ranked top-K list with MoveIt in
a second slice on the workstation.

## Tests

- valid archives concatenate into the expected candidate count and shape;
- malformed archive contents are rejected with clear errors;
- identity object pose preserves candidate transforms;
- a candidate whose `+Z` is world `-Z` outranks a side-facing candidate; and
- deterministic ranking returns the same ordered indices on repeated runs.
