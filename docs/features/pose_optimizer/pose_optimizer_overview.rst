.. _pose-optimizer-overview:

Getting started
===============

The Ouster SDK provides a Pose Optimizer API that allows developers to refine SLAM-generated trajectories and improve point cloud quality through constraint-based optimization.

The API is built around two primary components:

* **PoseOptimizer** – The main class that processes trajectories, applies constraints, runs optimization algorithms export the refined poses
* **Constraints** – Various constraint types (``PoseToPoseConstraint``, ``PointToPointConstraint``, ``AbsolutePoseConstraint``, ``AbsolutePointConstraint``) that define relationships for optimization

Prerequisites
-------------

Before running the Pose Optimizer, download the SLAM-processed OSF sample (``loop.osf``) from `here <https://studio.ouster.com/share/T29BEI2EIL55T648>`__. You will also need a JSON configuration file that defines both the solver parameters and the optimization constraints described below—you can grab a ready-to-test example for ``loop.osf`` at :ref:`Example JSON File <ouster-po-example-json>`.

JSON Configuration File
-----------------------

The Pose Optimizer uses a single JSON configuration file to control two aspects of the run:

* **Solver parameters** – Tunables that influence the optimizer’s behavior (iteration limits, loss configuration, trajectory constraint weights, etc.).
* **Constraint definitions** – Rules that guide trajectory refinement by enforcing relationships between poses or specific points in the map. During optimization, these constraints let you impose limits, incorporate prior knowledge, or keep loop closures/keyframes consistent.

Constraint JSON File Structure
------------------------------

A valid constraint configuration file must contain:

1. ``key_frame_distance`` – Defines the spacing between key nodes (meters). Smaller values create more keyframes for finer corrections, at the cost of longer runtime. Value 0 means that all poses are keyframes.
2. ``constraints`` – A list of constraint objects. Each entry must follow one of the schemas explained later on this page (``ABSOLUTE_POSE``, ``ABSOLUTE_POINT``, ``POINT_TO_POINT``, ``POSE_TO_POSE``).

**Example:**

.. code-block:: json

    {
        "key_frame_distance": 1.0,
        "constraints": [
            // constraint objects described in the next sections
        ]
    }

Solver Parameters
-----------------

The JSON file includes the following solver parameters with their default values:

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Parameter
     - Default
     - Description
   * - ``max_num_iterations``
     - 200
     - Maximum solver iterations before termination
   * - ``loss_function``
     - TRIVIAL_LOSS
     - Robust loss function: HUBER_LOSS, CAUCHY_LOSS, SOFT_L_ONE_LOSS, ARCTAN_LOSS, TRIVIAL_LOSS
   * - ``loss_scale``
     - 1.0
     - Scaling parameter for robust loss function (higher = less sensitive to outliers)
   * - ``traj_rotation_weight``
     - 1.0
     - Weight for rotational constraints during trajectory optimization
   * - ``traj_translation_weight``
     - 1.0
     - Weight for translational constraints during trajectory optimization
   * - ``key_frame_distance``
     - 1.0
     - Distance threshold for keyframe selection (in meters)
   * - ``process_printout``
     - true
     - Enable/disable solver progress printout during optimization
   * - ``fix_first_node``
     - false
     - Fix the first node of the trajectory during optimization

Constraint Definitions
----------------------

Constraints represent known relationships between poses (position and orientation) or points (specific measured locations) in the trajectory that should be maintained or enforced during optimization. By applying these known relationships, we can refine the trajectory and the corresponding map, removing for drift and achieving global consistency across the entire path

The Pose Optimizer supports four constraint types currently:

.. list-table::
   :header-rows: 1
   :widths: 18 45 50

   * - Constraint Type
     - Use Cases
     - Frame selection details
   * - ``PoseToPoseConstraint``
     - Constrain relative pose between two frame poses. Useful for loop closure refinement
     - The timestamps in constraint have to match the timestamps of the first pose in each frame
   * - ``PointToPointConstraint``
     - Match two selected points between two frames. Useful for loop closure or feature alignment
     - The point indexes have to match the row and column indices of the staggered 2D image in each frame
   * - ``AbsolutePoseConstraint``
     - Fix poses to known world coordinates like GPS integration or reference frame alignment
     - The timestamp can be any time within the whole recording duration
   * - ``AbsolutePointConstraint``
     - Constrain a point to surveyed positions for Landmark, known targets alignment
     - The point indexes have to match the row and column indices of the staggered 2D image in each frame

.. note::

   If neither an ``ABSOLUTE_POSE`` nor an ``ABSOLUTE_POINT`` constraint is provided, the optimizer automatically fixes the first node in the trajectory to keep the problem well-posed. The solver parameter ``fix_first_node`` in the config overrides this behavior (set it to ``true`` to always fix the first node, or ``false`` to make it free to move even absolute constraints exist).

Automatic Constraint Helpers
----------------------------

In addition to loading constraints from JSON or constructing them manually, the
Pose Optimizer provides helper workflows for common refinement tasks:

**Loop-Closure Detection**
  ``PoseOptimizer.add_relative_loop_constraints(min_distance_m, cell_size_m,
  icp_score_threshold)`` walks the current key-frame trajectory, identifies
  spatial revisits with a grid-based hash, and adds loop-closure
  ``POSE_TO_POSE`` constraints automatically. It returns the number of
  constraints added. See the :ref:`API page <pose-optimizer-api>` for code
  examples.

**CLI Wrappers**
  ``ouster-cli source ... pose_optimize --auto-gps`` and ``--auto-loop`` expose
  the same workflows from the command line. In batch mode the combined flow
  solves once after GPS alignment before running loop detection; in viz mode
  loop generation is deferred until you press ``L``.

ABSOLUTE_POSE
~~~~~~~~~~~~~

Defines an absolute pose measurement relative to the world coordinate frame.

**Required fields:**

- ``type``: Must be exactly ``"ABSOLUTE_POSE"``.
- ``timestamp``: Any timestamp within the OSF recording window (it does **not** have to match a frame’s first column).
- ``pose``: One of:

  * A dictionary with keys: ``rx``, ``ry``, ``rz``, ``x``, ``y``, ``z`` (rotation angles in **radians**, applied in **X-Y-Z order**).
  * A list of 6 values (Euler angles in **radians** with **X-Y-Z order** and position) or 16 values (4x4 matrix).

**Optional fields:**

- ``rotation_weight`` (float):

  * Scales the quaternion axis-alignment residual applied to this pose.
  * Higher values increase sensitivity to rotational deviations; setting it to ``0`` disables the rotational penalty.
- ``translation_weight``:

  * A list of three float values.
  * Corresponds to confidence in the **x**, **y**, and **z** position.

.. _weight-fields:

.. note::

   If ``translation_weight`` is omitted the optimizer assumes ``[1.0, 1.0, 1.0]`` internally.
   If ``rotation_weight`` is omitted the optimizer assumes ``1.0`` internally.
   These defaults apply to any constraint type below that supports the corresponding weight fields.
   Setting a weight to ``0`` disables the corresponding penalty.

Below is an example JSON code block for an ``ABSOLUTE_POSE`` constraint:

.. code-block:: json

    {
      "constraints": [
        {
          "type": "ABSOLUTE_POSE",
          "timestamp": 1765338003762995576,
          "pose": {
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
            "x": 40,
            "y": 30,
            "z": 10
          },
          "rotation_weight": 1.0,
          "translation_weight": [1.0, 1.0, 1.0]
        }
      ]
    }

ABSOLUTE_POINT
~~~~~~~~~~~~~~

Fixes a single point from a frame to known coordinates in the trajectory map frame.

**Required fields:**

- ``type``: Must be exactly ``"ABSOLUTE_POINT"``.
- ``timestamp``: Lidar frame timestamp (first valid column timestamp).
- ``row``: Row index in the staggered image (``0 <= row < frame.h``).
- ``col``: Column index in the staggered image (``0 <= col < frame.w``).
- ``return_idx``: Which range return to use (``1`` or ``2``).
- ``absolute_position``: Either an object with keys ``x``, ``y``, ``z`` or a list ``[x, y, z]``.

**Optional fields:**

- ``translation_weight``: List of three values describing confidence along ``x``, ``y``, and ``z``.

Below is an example JSON code block for an ``ABSOLUTE_POINT`` constraint:

.. code-block:: json

    {
      "constraints": [
        {
          "type": "ABSOLUTE_POINT",
          "timestamp": 1765338003762995576,
          "row": 27,
          "col": 1894,
          "return_idx": 1,
          "absolute_position": [40.0, 30.0, 10.0],
          "translation_weight": [1.0, 1.0, 1.0]
        }
      ]
    }

POINT_TO_POINT
~~~~~~~~~~~~~~

The ``POINT_TO_POINT`` constraint aligns a point from one point cloud to a corresponding point in another point cloud using 2D image coordinates from the lidar frames.

**Required fields:**

- ``type``: Must be ``"POINT_TO_POINT"``.
- ``timestamp1``: Frame timestamp for the first point (must be the frame's first valid timestamp).
- ``row1``: Staggered image row index for the first point.
- ``col1``: Staggered image column index for the first point.
- ``return_idx1``: Which range return to use for the first point (``1`` or ``2``).
- ``timestamp2``: Frame timestamp for the second point (frame's first valid timestamp).
- ``row2``: Staggered image row index for the second point.
- ``col2``: Staggered image column index for the second point.
- ``return_idx2``: Which range return to use for the second point (``1`` or ``2``).

**Optional fields:**

- ``translation_weight``: List of three numbers specifying axis confidence.

See :ref:`Weight Fields <weight-fields>` for a detailed explanation.

Below is an example JSON code block for a ``POINT_TO_POINT`` constraint:

.. code-block:: json

    {
      "constraints": [
        {
          "type": "POINT_TO_POINT",
          "timestamp1": 1765338003762995576,
          "row1": 27,
          "col1": 1894,
          "return_idx1": 1,
          "timestamp2": 1765338746963563448,
          "row2": 29,
          "col2": 1221,
          "return_idx2": 1,
          "translation_weight": [0.2, 0.2, 0.2]
        }
      ]
    }

POSE_TO_POSE
~~~~~~~~~~~~

The ``POSE_TO_POSE`` constraint links two lidar frames by matching the poses associated with each frame’s first valid column. Conceptually, it tells the optimizer “these two frames should maintain this relative motion,” which is especially useful for loop-closures or any place where you can align two point clouds. You can supply the translation/rotation directly (e.g., from manually aligned frames) or let the optimizer estimate it via ICP between the two frame point clouds.

**Required fields:**

- ``type``: Must be ``"POSE_TO_POSE"``.
- ``timestamp1``: Frame timestamp for the first pose (frame's first valid timestamp).
- ``timestamp2``: Frame timestamp for the second pose (frame's first valid timestamp).

**Optional fields:**

- ``transformation``: Defines the relative pose from pose 1 to pose 2.

  * A dictionary (``rx``, ``ry``, ``rz``, ``x``, ``y``, ``z``) with rotation angles in **radians**, applied in **X-Y-Z order**, or
  * A 16-element list (4x4 matrix)
  * If omitted, the transformation is auto-estimated using ICP matching.
- ``rotation_weight`` (float): Scales the quaternion axis-alignment residual for the relative pose. Higher values enforce the measured rotation more strongly.
- ``translation_weight``: List of three values.

Below is an example JSON code block for a ``POSE_TO_POSE`` constraint:

.. code-block:: json

    {
      "constraints": [
        {
          "type": "POSE_TO_POSE",
          "timestamp1": 1765338059263057992,
          "timestamp2": 1765338723963851640,
          "rotation_weight": 1.0,
          "translation_weight": [1.0, 1.0, 1.0]
        }
      ]
    }

For detailed information about constraint types, mathematical foundations, and implementation examples, see :doc:`pose_optimizer_api`.

.. _ouster-po-example-json:

Example JSON File
-----------------

Here’s an example constraint configuration file that you can run with the sample ``loop.osf`` data:

.. literalinclude:: _snippets/cli/pose_optimizer_cli_constraints.json
   :language: json
   :caption: Constraint JSON Example
