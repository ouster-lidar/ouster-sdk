# Changelog

## [1.13.0] - 2020-03-16
### Added
- post-processing to improve ambient image uniformity in visualizer
- make timestamp mode configurable via the client (PR #97)

### Changed
- turn on position-independent code by default to make using code in libraries
  easier (PR #65)
- use random ports for lidar and imu data by default when unspecified

### Fixed
- prevent legacy tf prefix from making invalid frame names (PR #56)
- use `iterator_traits` to make `batch_to_iter` work with more types (PR #70)
- use correct name for json dependency in `package.xml` (PR #116)
- handle udp socket creation error gracefully in client

## [1.12.0] - 2019-05-02
### Added
- install directives for `ouster_ros` build (addresses #50)

### Changed
- flip the sign on IMU acceleration output to follow usual conventions
- increase the update rate in the visualizer to ~60hz

### Fixed
- visualizer issue where the point cloud would occasionally occasionally not be
  displayed using newer versions of Eigen

## [1.11.0] - 2019-03-26
### Added
- allow renaming tf ids using the `tf_prefix` parameter

### Changed
- use frame id to batch packets so client code deals with reordered lidar
  packets without splitting frames
- use a uint32_t for PointOS1 timestamps to avoid unnecessary loss of precision

### Fixed
- bug causing ring and reflectivity to be corrupted in os1_cloud_node output
- misplaced sine in azimuth angle calculation (addresses #42)
- populate timestamps on image node output (addresses #39)

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
- the timestamp field of PointOS1 now represents time since the start of the
  scan (the timestamp of the PointCloud2 message) in nanoseconds

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
