from __future__ import annotations

import numpy as np
import os

# [doc-stag-po-imports]
import ouster.sdk.mapping as mapping
# [doc-etag-po-imports]

osf_filename = os.environ.get("POSE_OPTIMIZER_OSF", "mapping/loop.osf")

# [doc-stag-po-construct]
po = mapping.PoseOptimizer(
    osf_filename=osf_filename,
    key_frame_distance=2.0,
)
# [doc-etag-po-construct]

# [doc-stag-po-pose-to-pose]
pose_to_pose_constraint = mapping.PoseToPoseConstraint(
    timestamp1=np.uint64(1765338059263057992),
    timestamp2=np.uint64(1765338723963851640),
    relative_pose=np.eye(4),
    rotation_weight=1.0,
    translation_weight=np.array([1.0, 1.0, 1.0]),
)
po.add_constraint(pose_to_pose_constraint)
# [doc-etag-po-pose-to-pose]

# [doc-stag-po-auto-loop]
loop_constraints_added = po.add_relative_loop_constraints(
    min_distance_m=50.0,
    cell_size_m=2.0,
    icp_score_threshold=0.6,
)
# [doc-etag-po-auto-loop]

# [doc-stag-po-point-to-point]
point_to_point_constraint = mapping.PointToPointConstraint(
    timestamp1=np.uint64(1765338003762995576),
    row1=27, col1=1894, return_idx1=1,
    timestamp2=np.uint64(1765338746963563448),
    row2=29, col2=1221, return_idx2=1,
    translation_weight=np.array([0.2, 0.2, 0.2]))
po.add_constraint(point_to_point_constraint)
# [doc-etag-po-point-to-point]

# [doc-stag-po-absolute-point]
absolute_point_constraint = mapping.AbsolutePointConstraint(
    timestamp=np.uint64(1765338003762995576),
    row=27,
    col=1894,
    return_idx=1,
    absolute_position=np.array([40.0, 30.0, 10.0]),
    translation_weight=np.array([1.0, 1.0, 1.0]))
po.add_constraint(absolute_point_constraint)
# [doc-etag-po-absolute-point]

# [doc-stag-po-absolute-pose]
abs_pose = np.eye(4)
abs_pose[:3, 3] = np.array([40.0, 30.0, 10.0])
absolute_pose_constraint = mapping.AbsolutePoseConstraint(
    timestamp=np.uint64(1765338003762995576),
    pose=abs_pose,
    rotation_weight=1.0,
    translation_weight=np.array([1.0, 1.0, 1.0]))
absolute_pose_constraint_id = po.add_constraint(absolute_pose_constraint)
# [doc-etag-po-absolute-pose]

# [doc-stag-po-remove-constraint]
po.remove_constraint(absolute_pose_constraint_id)
# [doc-etag-po-remove-constraint]

# [doc-stag-po-solve]
po.solve()
# [doc-etag-po-solve]

# [doc-stag-po-save-trajectory]
ts = po.get_timestamps(mapping.SamplingMode.COLUMNS)
poses = po.get_poses(mapping.SamplingMode.COLUMNS)
mapping.save_trajectory("loop_test_traj.csv", ts, poses)
# [doc-etag-po-save-trajectory]

output_osf = os.environ.get("POSE_OPTIMIZER_OUTPUT", "po_output.osf")
# [doc-stag-po-save-osf]
po.save(output_osf)
# [doc-etag-po-save-osf]
