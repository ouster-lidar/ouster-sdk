# Changelog

## [1.10.0] - 2019-01-27
### Added
- `os1_node` now queries and uses calibrated beam angles from the sensor
- `os1_node` now queries and uses imu / lidar frames from the sensor
- `os1_node` reads and writes metadata to `${ROS_HOME}` to support replaying
  data with calibration
- ROS example code now publishes tf2 transforms for imu / lidar frames
  (addresses #12)
- added `metadata` parameter to `os1.launch` to override location of metadata
- added `viz` parameter to `os1.launch` to run the example visualizer with ROS
- added `image` parameter to `os1.launch` to publish image topics to rviz
  (addresses #21)
- added range field to `PointOS1`

### Changed
- split point-cloud publishing out of `os1_node` into `os1_cloud_node`
- example visualizer controls:
    + press `m` to cycle through color modes instead of `i`, `z`, `Z`, `r`
    + `r` now resets the camera position
    + range/intensity images automatically resized to fit window height
- updated OS-1 client to use newer TCP configuration commands
- updated OS-1 client to set the requested lidar mode, reinitialize on connection
- changed point cloud batching to be based on angle rather than scan duration
- `ouster_client` now depends on the `jsoncpp` library
- switched order of fields in `PointOS1` to be compatible with `PointXYZI`
  (addresses #16)
- moved example visualizer VTK rendering into the main thread (merged #23)

### Removed
- removed keyboard camera controls in example visualizer
- removed panning and rotating of the image panel in example visualizer

### Fixed
- no longer dropping UDP packets in 2048 mode on tested hardware
- example visualizer:
    + point cloud display focus no longer snaps back on rotation
    + fixed clipping issues with parallel projection
    + fixed point coloring issues in z-color mode
    + improved performance
