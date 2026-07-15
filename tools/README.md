# 2D Hand-Eye Calibration Tools

This directory contains the two operator scripts for the RGB 2D hand-eye
workflow:

- `calibrate_handeye_2d.py`: collect pixel-to-arm samples and compute the 2D
  homography.
- `test_handeye_2d.py`: click an image point, project it through the saved
  homography, and command the active manipulation backend over Atlas/gRPC.

The homography maps image pixels `[u, v, 1]` from `/camera/color/image_raw` to
XY coordinates in `arm/base_link`. It is saved under:

```bash
rbnx-boot/hand-eye-data/2d_homography.npy
```

## Before Calibration

Start the robot stack:

```bash
rbnx boot
```

If camera resolution, camera mounting, observation pose, or tabletop layout
changes, remove the old calibration files before collecting new samples:

```bash
rm rbnx-boot/hand-eye-data/2d_image_points.npy \
   rbnx-boot/hand-eye-data/2d_end_poses.npy \
   rbnx-boot/hand-eye-data/2d_robot_points.npy \
   rbnx-boot/hand-eye-data/2d_homography.npy \
   rbnx-boot/hand-eye-data/2d_homography_report.txt
```

## Calibrate

Run:

```bash
python3 tools/calibrate_handeye_2d.py
```

Workflow:

1. The script enables the arm and calls `/moveit_control/reset` to move to the
   fixed observation pose.
2. Click a visible calibration point in the RGB image.
3. Press `Enter` or `Space`.
4. The arm moves to `/moveit_control/teach_safe`.
5. Press `d` to confirm it is safe to disable.
6. Manually drag the arm tip to the clicked physical point.
7. Press `Space` to record the pair.
8. Repeat for at least 4 points. Use more points, such as 8-12, for a better
   fit.
9. Press `Esc` or `Ctrl-C` to finish.

Outputs:

```bash
rbnx-boot/hand-eye-data/2d_image_points.npy
rbnx-boot/hand-eye-data/2d_end_poses.npy
rbnx-boot/hand-eye-data/2d_robot_points.npy
rbnx-boot/hand-eye-data/2d_homography.npy
rbnx-boot/hand-eye-data/2d_homography_report.txt
```

If samples are already collected and only the homography/report needs to be
recomputed, run:

```bash
python3 tools/calibrate_handeye_2d.py --mode compute
```

## Check The Report

Open:

```bash
rbnx-boot/hand-eye-data/2d_homography_report.txt
```

The most useful fields are:

- `inlier_mask`: points used by OpenCV's homography fit.
- `mean_error_m`: average reprojection error in meters.
- `max_error_m`: largest reprojection error in meters.

Large errors usually mean a wrong click, an inaccurate manual arm placement, or
mixing samples from different camera resolutions.

## Test Calibration

After `2d_homography.npy` exists, run:

```bash
python3 tools/test_handeye_2d.py \
  --z -0.19 \
  --orientation-mode vertical
```

Workflow:

1. The script calls `/moveit_control/reset`.
2. Click a point in the RGB image.
3. Press `Enter` or `Space`.
4. The script projects the clicked pixel through the homography.
5. It calls `robonix/service/manipulation/execute_grasp` through Atlas/gRPC.

Useful options:

```bash
--z -0.19
```

Target Z in `arm/base_link`, in meters.

```bash
--orientation-mode vertical
```

Use a fixed vertical-down grasp orientation.

```bash
--orientation-mode current
```

Keep the end-effector orientation observed after reset.

```bash
--skip-reset
```

Use the current camera/arm pose instead of calling `/moveit_control/reset`.

## Notes

- The image window uses `cv2.WINDOW_AUTOSIZE` so click coordinates correspond
  to the original image pixels.
- Changing camera resolution invalidates the old homography. Recalibrate after
  changing `color_width` or `color_height`.
- The production grasp pipeline reads the same `2d_homography.npy` through
  `grasp_pose.config.hand_eye_calibration_file`.
