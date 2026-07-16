.. _pose-optimizer-api:

Using the API
=============

This section walks through the Pose Optimizer workflow using the canonical example shipped with the SDK. Every snippet is live code, so you can copy the pattern, adjust timestamps/weights for your data, and combine constraints to refine trajectories programmatically.

Imports
-------

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-imports]
         :end-before: # [doc-etag-po-imports]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-po-imports]
         :end-before: //! [doc-etag-po-imports]

Construct the Optimizer
-----------------------

Instantiate ``PoseOptimizer`` with your SLAM-produced OSF and desired key-frame spacing.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-construct]
         :end-before: # [doc-etag-po-construct]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-construct]
         :end-before: //! [doc-etag-po-construct]

Pose-to-Pose Constraint
-----------------------

Constrains the relative transformation between two frame poses using all available point cloud data through algorithms like ICP (Iterative Closest Point) or by registering poses with external measurements like GPS. Each pose corresponds to the first valid column of a LiDAR frame.

.. math::

   \mathbf{r}_{\text{pose2pose}} = \log\left(\mathbf{T}_{\text{measured}}^{-1} \mathbf{T}_{\text{target}}\right)

Where :math:`\mathbf{T}_{\text{measured}}` is the current relative transform, :math:`\mathbf{T}_{\text{target}}` is the desired transform, and :math:`\log(\cdot)` maps the matrix error to a 6‑vector.

This differs from point-to-point constraints which use specific point correspondences rather than all point cloud data.

Typical use cases:

- enforcing odometry relationships between consecutive frames using ICP registration
- loop-closure constraints from full frame-to-frame matching algorithms
- GPS/INS-derived pose measurements integrated with LiDAR poses
- maintaining temporal consistency in long trajectories using all point cloud information

Constrain two frames using a relative pose computed from entire point clouds or external sensors to limit drift over long trajectories.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-pose-to-pose]
         :end-before: # [doc-etag-po-pose-to-pose]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-pose-to-pose]
         :end-before: //! [doc-etag-po-pose-to-pose]

Automatic Loop-Closure Constraints
----------------------------------

Use ``add_relative_loop_constraints()`` to scan the current key-frame
trajectory, detect nearby revisits with a spatial hash, and add
``PoseToPoseConstraint`` loop closures automatically. The method returns the
number of constraints added.

This is the same API used by ``ouster-cli source ... pose_optimize --auto-loop``.
Tune ``min_distance_m`` to avoid dense duplicate detections, ``cell_size_m`` to
control the search grid resolution, and ``icp_score_threshold`` to reject weak
matches.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-auto-loop]
         :end-before: # [doc-etag-po-auto-loop]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-auto-loop]
         :end-before: //! [doc-etag-po-auto-loop]

Point-to-Point Constraint
-------------------------

Forces two selected 3D points observed in two different frames to occupy the same 3D location. This constraint uses manually selected point correspondences between two frames, rather than using all points in the point clouds.

.. math::

   \mathbf{r}_{\text{pt2pt}} = \mathbf{T}_1 \mathbf{p}_1 - \mathbf{T}_2 \mathbf{p}_2

Where :math:`\mathbf{T}_1, \mathbf{T}_2` are the frame poses and :math:`\mathbf{p}_1, \mathbf{p}_2` are corresponding specific points identified across frames.

This differs from pose-to-pose constraints which match two selected lidar frame poses. The relative transformation between poses can be calculated using ICP algorithms, or we can match one lidar frame pose with a GPS pose.

Typical use cases:

- loop-closure correspondences using specific landmark points in large environments
- feature tracking of distinct geometric features across multiple frames  
- reducing drift by matching structural edges, corners, or distinctive boundaries
- constraining poses using manually selected point correspondences

Tie explicit point correspondences together across frames to align local geometric features.

Row/col indices for point-to-point constraints are specified in the staggered
image (``0 <= row < frame.h``, ``0 <= col < frame.w``).

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-point-to-point]
         :end-before: # [doc-etag-po-point-to-point]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-point-to-point]
         :end-before: //! [doc-etag-po-point-to-point]

Absolute Point Constraint
-------------------------

Pin a specific return to a known global coordinate.

.. math::

   \mathbf{r}_{\text{abs\_point}} = \mathbf{T} \mathbf{p}_{\text{local}} - \mathbf{p}_{\text{world}}

Where :math:`\mathbf{T}` is the pose at the constraint timestamp, :math:`\mathbf{p}_{\text{local}}` is the frame point, and :math:`\mathbf{p}_{\text{world}}` is the surveyed position.

Typical use cases:

- surveyed landmarks, ground control points, or fiducials
- aligning to known infrastructure corners or poles
- tying trajectories to building layouts or survey markers

Lock a single return to an absolute coordinate to stabilize the map.

Row/col indices for absolute point constraints are specified in the staggered
image (``0 <= row < frame.h``, ``0 <= col < frame.w``).

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-absolute-point]
         :end-before: # [doc-etag-po-absolute-point]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-absolute-point]
         :end-before: //! [doc-etag-po-absolute-point]

Absolute Pose Constraint
------------------------

Fix an entire 6‑DoF pose (position + orientation) to a known target frame.

.. math::

   \mathbf{r}_{\text{abs\_pose}} =
   \begin{bmatrix}
   \log\left(\mathbf{R}_{\text{measured}}^T \mathbf{R}_{\text{target}}\right) \\
   \mathbf{t}_{\text{measured}} - \mathbf{t}_{\text{target}}
   \end{bmatrix}

Where :math:`\mathbf{R}` and :math:`\mathbf{t}` are the rotation and translation components.

Typical use cases:

- GPS waypoint anchoring
- sensor calibration or alignment to survey control
- constraining start/end poses in a loop to prevent drift
- multi-session data fusion

Pin an entire 6DoF pose in place (with optional removal downstream).

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-absolute-pose]
         :end-before: # [doc-etag-po-absolute-pose]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-absolute-pose]
         :end-before: //! [doc-etag-po-absolute-pose]

Remove a Constraint by ID
-------------------------

Once you capture a constraint identifier (e.g., the result of ``add_constraint``), you can remove it to re-run the optimizer without that constraint.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-remove-constraint]
         :end-before: # [doc-etag-po-remove-constraint]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-remove-constraint]
         :end-before: //! [doc-etag-po-remove-constraint]

Run the Optimization
--------------------

Call ``solve()`` to run the constrained least-squares optimization. Each invocation advances the solver with the current constraint set; repeat as needed after adding or removing constraints.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-solve]
         :end-before: # [doc-etag-po-solve]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-solve]
         :end-before: //! [doc-etag-po-solve]

Export a Trajectory
-------------------

After solving, query timestamps/poses in your preferred sampling mode (columns or key frames) and save them to CSV/TUM using ``save_trajectory`` for downstream comparison or evaluation.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-save-trajectory]
         :end-before: # [doc-etag-po-save-trajectory]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-save-trajectory]
         :end-before: //! [doc-etag-po-save-trajectory]

Save the Optimized OSF
----------------------

Persist the updated poses back to an OSF so other tools (e.g., ``ouster-cli`` viz, map generation, localization) can consume the refined trajectory without rerunning the optimization.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-save-osf]
         :end-before: # [doc-etag-po-save-osf]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :dedent: 4
         :start-after: //! [doc-stag-po-save-osf]
         :end-before: //! [doc-etag-po-save-osf]

Complete Example Scripts
------------------------

Prefer to copy everything at once? The canonical example scripts used throughout this page are available below without the snippet boundaries, so you can run them end-to-end and tweak paths or timestamps as needed.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/po_example.py
         :language: python
         :start-after: # [doc-stag-po-construct]
         :end-before: # [doc-etag-po-save-osf]

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/po_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-po-construct]
         :end-before: //! [doc-etag-po-example]
         :dedent: 4
