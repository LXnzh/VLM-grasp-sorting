
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R



def plot_trajectory_positions(trajectory: list[np.ndarray], show: bool = True) -> None:
    """
    Plot the XYZ positions of all poses in a trajectory as points in 3D space.

    Parameters
    ----------
    trajectory : list of np.ndarray
        List of (6,) poses (as returned by plan_pick_place_trajectory).
    show : bool, optional
        Whether to immediately call plt.show(). Defaults to True.

    Notes
    -----
    - Only the positions are plotted (orientation ignored).
    - The plot shows a connected 3D scatter/line path.
    - The start and end points are highlighted for clarity.
    """
    if not trajectory:
        raise ValueError("Trajectory is empty — nothing to plot.")

    # Extract XYZ positions
    positions = np.array([pose[:3] for pose in trajectory])
    xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(xs, ys, zs, "-o", markersize=3, label="Trajectory")

    # Highlight start & end
    ax.scatter(xs[0], ys[0], zs[0], color="green", s=50, label="Start")
    ax.scatter(xs[-1], ys[-1], zs[-1], color="red", s=50, label="End")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Pick-Place Trajectory (positions only)")
    ax.legend()
    ax.grid(True)
    ax.set_box_aspect([1, 1, 0.7])  # balanced aspect ratio

    if show:
        plt.show()

def plot_trajectory_with_frames(
    trajectory: list[np.ndarray],
    frame_scale: float = 0.03,
    frame_stride: int = 1,
    show: bool = True,
) -> None:
    """
    Plot the XYZ positions of all poses in a trajectory as points in 3D space,
    including small coordinate frames (X, Y, Z axes) at each waypoint.

    Parameters
    ----------
    trajectory : list of np.ndarray
        List of (6,) poses (as returned by plan_pick_place_trajectory).
    frame_scale : float, optional
        Length of axis arrows for each local coordinate frame [m].
    frame_stride : int, optional
        Plot every Nth frame only (to reduce clutter).
    show : bool, optional
        Whether to immediately call plt.show().

    Notes
    -----
    - X/Y/Z axes of each frame are drawn in red/green/blue respectively.
    - Start and end poses are highlighted.
    - All poses are assumed in [x, y, z, roll, pitch, yaw] with radians.
    - All axes in the 3D plot are scaled equally (1:1:1).
    """
    if not trajectory:
        raise ValueError("Trajectory is empty — nothing to plot.")

    # Extract XYZ positions
    positions = np.array([pose[:3] for pose in trajectory])
    xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Plot trajectory line and waypoints
    ax.plot(xs, ys, zs, "-o", color="gray", markersize=3, label="Trajectory")

    # Start/end markers
    ax.scatter(xs[0], ys[0], zs[0], color="green", s=50, label="Start")
    ax.scatter(xs[-1], ys[-1], zs[-1], color="red", s=50, label="End")

    # Plot small coordinate frames
    for i, pose in enumerate(trajectory[::frame_stride]):
        p = pose[:3]
        roll, pitch, yaw = pose[3:6]
        rot = R.from_euler("zyx", [yaw, pitch, roll], degrees=False)
        Rm = rot.as_matrix()

        # local axes in world coordinates
        x_axis = p + Rm[:, 0] * frame_scale
        y_axis = p + Rm[:, 1] * frame_scale
        z_axis = p + Rm[:, 2] * frame_scale

        ax.plot([p[0], x_axis[0]], [p[1], x_axis[1]], [p[2], x_axis[2]], color="r")
        ax.plot([p[0], y_axis[0]], [p[1], y_axis[1]], [p[2], y_axis[2]], color="g")
        ax.plot([p[0], z_axis[0]], [p[1], z_axis[1]], [p[2], z_axis[2]], color="b")

    # Equal axis scaling
    def set_axes_equal(ax):
        """Set 3D plot axes to equal scale so that spheres look like spheres."""
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()
        x_range = abs(x_limits[1] - x_limits[0])
        y_range = abs(y_limits[1] - y_limits[0])
        z_range = abs(z_limits[1] - z_limits[0])
        max_range = 0.5 * max(x_range, y_range, z_range)

        mid_x = np.mean(x_limits)
        mid_y = np.mean(y_limits)
        mid_z = np.mean(z_limits)
        ax.set_xlim3d([mid_x - max_range, mid_x + max_range])
        ax.set_ylim3d([mid_y - max_range, mid_y + max_range])
        ax.set_zlim3d([mid_z - max_range, mid_z + max_range])

    set_axes_equal(ax)

    # Labels, grid, legend
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Pick-Place Trajectory with Orientation Frames")
    ax.legend()
    ax.grid(True)

    if show:
        plt.show()