Use Pose Optimizer with the Ouster-CLI
======================================


.. _ouster-cli-pose-optimizer:


The pose optimizer tool can be used with the ``ouster-cli`` to refine the trajectory produced by a SLAM algorithm.
To learn how to use the Pose Optimizer with python API, refer to the :ref:`Pose Optimizer Python API Example <pose-optimize-example>`.

Prerequisites
-------------

Before using the Pose Optimizer, one must have: 

- A :ref:`SLAM processed OSF file <ouster-cli-mapping>` as input which contains the trajectory.
- Optionally, a JSON file that defines constraints for optimization (pass it via ``--config``) and Optimizer parameters.

The following section describes the expected structure and content of the JSON file and provides
guidelines for creating it. The constraints mentioned in examples below correspond to the `Sample Data`_ .

Constraint JSON File Structure
------------------------------

The constraint JSON file must include:

- A ``key_frame_distance`` parameter which defines the distance (in meters) between the key nodes.
- A top-level key ``constraints`` that maps to a list of constraint objects.

**Example:**

.. code-block:: json

    {
        "key_frame_distance": 1.0,
        "constraints": [
            // List of constraint objects explained in the next sections
        ]
    }

Optional Config Parameters
--------------------------

The constraint JSON file may also include the following optional parameters to fine-tune the optimization process:

- ``traj_rotation_weight`` (float): The weight for rotational constraints during trajectory optimization (default: ``10.0``). Higher values enforce stronger rotation consistency. Must be a float ``> 0``.
- ``traj_translation_weight`` (float): The weight for translational constraints during trajectory optimization (default: ``10.0``). Higher values enforce stronger position consistency. Must be a float ``> 0``.
- ``max_num_iterations`` (int): The maximum number of iterations the solver will perform before terminating.
- ``loss_function`` (str): The name of the robust loss function to use (e.g., ``HUBER_LOSS``, ``CAUCHY_LOSS``, ``SOFT_L_ONE_LOSS``, ``ARCTAN_LOSS``, ``TRIVIAL_LOSS``).
- ``loss_scale`` (float): The scaling parameter for the chosen loss function. Higher values make the loss less sensitive to outliers.
- ``fix_first_node`` (bool): Flag to fix the first node of the trajectory during optimization. Default is ``False``.
- ``function_tolerance`` (float): The tolerance threshold for changes in the cost function. Solver stops when improvements fall below this value.
- ``gradient_tolerance`` (float): The tolerance threshold for changes in the gradient. Solver stops when gradient magnitude falls below this value.
- ``parameter_tolerance`` (float): The tolerance threshold for changes in parameters. Solver stops when parameter changes fall below this value.
- ``process_printout`` (bool): Flag to enable or disable detailed printout of the optimization process.

These parameters are optional and can be omitted for default behavior. If included, they should be specified at the top level of the JSON file alongside ``key_frame_distance`` and ``constraints``.


.. _weight-fields:

Weight Fields: Rotation and Translation
---------------------------------------

Several constraint types (explained later) support optional weight fields that express your confidence in the rotation or translation information.

**rotation weight**

- ``rotation_weight`` (single float) scales the quaternion axis-alignment residual for ``ABSOLUTE_POSE`` and ``POSE_TO_POSE`` constraints. Higher values enforce the orientation more strongly; set it to ``0`` to ignore the rotational component.
- Must be a single float value ``>= 0``.

**Example (rotation_weight for ABSOLUTE_POSE):**

.. code-block:: json

    "rotation_weight": 2.0

This doubles the strength of the orientation penalty for that absolute pose.

**translation_weight**

- Must be a list of three float values ``>= 0``.
- Applies to the positional components: **x**, **y**, and **z**.
- Higher values signal stronger confidence in the position, and lower values give the optimizer more freedom to adjust.
- Values are treated as relative weights for each axis.

**Example:**

.. code-block:: json

    "translation_weight": [1.0, 1.0, 0.1]

This enforces strict alignment in x and y, while allowing more flexibility in z.


.. note::

    If weights are omitted, the optimizer assigns default value of ``1`` internally.


Constraint Types
----------------

The Pose Optimizer supports four constraint types:

- ``ABSOLUTE_POSE`` fixes a node to a global pose measurement.
- ``ABSOLUTE_POINT`` pins a single scan point to global coordinates.
- ``POINT_TO_POINT`` enforces correspondence between two scan points.
- ``POSE_TO_POSE`` constrains the relative motion between two trajectory nodes.


Each type requires specific fields and validations.


ABSOLUTE_POSE
~~~~~~~~~~~~~

Defines an absolute pose measurement relative to the world coordinate frame.


**Required fields:**

- ``type``: Must be exactly ``"ABSOLUTE_POSE"``.
- ``timestamp``: A numeric timestamp that must lie within the start and end timestamps of the OSF file, not limited to any specific LidarScan.
- ``pose``: One of:
  
  * A dictionary with keys: ``rx``, ``ry``, ``rz``, ``x``, ``y``, ``z``.
  * A list of 6 values (6x1 Euler angle and position vector) or 16 values (4x4 matrix).

**Optional fields:**

- ``rotation_weight`` (float):

  * Scales the quaternion axis-alignment residual applied to this pose.
  * Higher values increase sensitivity to rotational deviations; setting it to ``0`` disables the rotational penalty.
  * Must be a float value ``>= 0``.
- ``translation_weight``:
  
  * A list of three float values ``>= 0``.
  * Corresponds to confidence in the **x**, **y**, and **z** position.

See :ref:`weight-fields` for a detailed explanation.

.. note::
  
  If no ``ABSOLUTE_POSE`` constraint is provided, the system will assume the first node to be fixed i.e. first node pose value will not be changed.

Below is an example JSON code block for an ABSOLUTE_POSE constraint:


.. code-block:: json

    {
      "constraints": [
        {
          "type": "ABSOLUTE_POSE",
          "timestamp": 411107223370,
          "pose": {
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
            "x": 12,
            "y": 30,
            "z": 0.0
          },
          "rotation_weight": 1.0,
          "translation_weight": [1.0, 1.0, 1.0]
        }
      ]
    }


ABSOLUTE_POINT
~~~~~~~~~~~~~~

Fixes a single point from a scan to known coordinates in the trajectory map frame.

**Required fields:**

- ``type``: Must be exactly ``"ABSOLUTE_POINT"``.
- ``timestamp``: Lidar frame timestamp (first valid column timestamp).
- ``row``: Row index of the point (>= 0 && < scan.h).
- ``col``: Column index of the point (>= 0 && < scan.w).
- ``return_idx``: Which range return to use (1 or 2).
- ``absolute_position``: Either an object with keys ``x``, ``y``, ``z`` or a list of three numbers ``[x, y, z]``.

**Optional fields:**

- ``translation_weight``: List of three float values ``>= 0`` describing confidence along ``x``, ``y``, and ``z``.

.. note::

   If ``translation_weight`` is omitted the optimizer assumes ``[1.0, 1.0,
   1.0]`` internally.

Below is an example JSON code block for an ``ABSOLUTE_POINT`` constraint:

.. code-block:: json

    {
      "constraints": [
        {
          "type": "ABSOLUTE_POINT",
          "timestamp": 411107223370,
          "row": 10,
          "col": 100,
          "return_idx": 1,
          "absolute_position": [5.0, 2.0, 1.5],
          "translation_weight": [1.0, 1.0, 1.0]
        }
      ]
    }


POINT_TO_POINT
~~~~~~~~~~~~~~~~~~~~~~~~
The ``POINT_TO_POINT`` constraint is used to align a point from one point cloud to a corresponding point in another point cloud.

This constraint uses 2D image coordinates to select points from the lidar scans:

**Required fields:**

- ``type``: Must be ``"POINT_TO_POINT"``.
- ``timestamp1``: The frame timestamp for the first point (must be lidar scan's first valid timestamp)
- ``row1``: A numeric value > 0 for the first point
- ``col1``: A numeric value > 0 for the first point
- ``return_idx1``: Specifies which range return to use (1 for first return or 2 for second return)
- ``timestamp2``: The frame timestamp for the second point (must be lidar scan's first valid timestamp)
- ``row2``: A numeric value > 0 for the second point
- ``col2``: A numeric value > 0 for the second point
- ``return_idx2``: Specifies which range return to use (1 for first return or 2 for second return)

**Optional field:**

- ``translation_weight``: A list of three float values ``>= 0``.

See :ref:`weight-fields` for a detailed explanation.

**How to get valid timestamps:**

The frame timestamp can be obtained from the LidarScan object, which contains a list of valid timestamps for each point in the scan.

.. code-block:: python

    import numpy as np
    from ouster.sdk import open_source
    from ouster.sdk.core import LidarScan

    def main():
        osf = 'sample.osf'
        source = open_source(osf)

        idx = 0
        for scans in source:
            for sensor_id, scan in enumerate(scans):
                if not isinstance(scan, LidarScan):
                    continue

                first_valid_ts = scan.get_first_valid_column_timestamp()
    main()
                

Below is an example JSON code block for a ``POINT_TO_POINT`` constraint:

.. code-block:: json
   
    {
      "constraints": [
        {
          "type": "POINT_TO_POINT",
          "timestamp1": 411107223370,
          "row1": 9,
          "col1": 542,
          "return_idx1": 1,
          "timestamp2": 531097181460,
          "row2": 5,
          "col2": 545,
          "return_idx2": 1,
          "translation_weight": [1.0, 1.0, 1.0]
        }
      ]
    }

POSE_TO_POSE
~~~~~~~~~~~~~~~~~~~~~~

The ``POSE_TO_POSE`` constraint defines a constraint between two poses in the trajectory.

**Required fields:**

- ``type``: Must be ``"POSE_TO_POSE"``.
- ``timestamp1``: The frame timestamp for the first pose (lidar scan's first valid timestamp)
- ``timestamp2``: The frame timestamp for the second pose (lidar scan's first valid timestamp)

**Optional fields:**

- ``transformation``: Defines the relative pose from pose1 to pose2
  
  * A dictionary (with ``rx``, ``ry``, ``rz``, ``x``, ``y``, ``z``), or
  * A 16-element list (4x4 matrix)
  * If omitted, transformation is auto-estimated using ICP matching.
- ``rotation_weight`` (float): Scales the quaternion axis-alignment residual for the relative pose. Setting it higher enforces the measured rotation more strongly; setting it to ``0`` disables the rotational penalty. Must be a float value ``>= 0``.
- ``translation_weight``: List of three float values ``>= 0``.

See :ref:`weight-fields` for a detailed explanation.

Below is an example JSON code block for a ``POSE_TO_POSE`` constraint:

.. code-block:: json

    {
      "constraints": [
        {
          "type": "POSE_TO_POSE",
          "timestamp1": 411107223370,
          "timestamp2": 531097181460,
          "transformation": {
            "rx": -0.000396419,
            "ry": -0.0103704,
            "rz": -0.0289703,
            "x": 1.29628,
            "y": 0.61089,
            "z": -0.138204
          },
          "rotation_weight": 1.0,
          "translation_weight": [1.0, 1.0, 1.0]
        }
      ]
    }


Example JSON File
-----------------

Below is an example JSON file that demonstrates all supported constraint types.
You can use this file with the downloaded OSF file to run the Pose Optimizer.


.. code-block:: json

        {
            "key_frame_distance": 2.0,
            "constraints": [
                {
                    "type": "ABSOLUTE_POSE",
                    "translation_weight": [1.0, 1.0, 1.0],
                    "rotation_weight": 2.0,
                    "timestamp": 411107223340,
                    "pose": {
                        "rx": 0,
                        "ry": 0,
                        "rz": 0,
                        "x": 12,
                        "y": 30,
                        "z": 1.3
                    }
                },
                {
                    "type": "ABSOLUTE_POINT",
                    "translation_weight": [1.0, 1.0, 1.0],
                    "timestamp": 411107223370,
                    "row": 10,
                    "col": 100,
                    "return_idx": 1,
                    "absolute_position": [5.0, 2.0, 1.5]
                },
                {
                    "type": "POINT_TO_POINT",
                    "translation_weight": [1.0, 1.0, 1.0],
                    "timestamp1": 411107223370,
                    "row1": 9,
                    "col1": 542,
                    "return_idx1": 1,
                    "timestamp2": 531097181460,
                    "row2": 5,
                    "col2": 545,
                    "return_idx2": 1
                },
                {
                    "type": "POSE_TO_POSE",
                    "translation_weight": [2.0, 2.0, 2.0],
                    "rotation_weight": 2.0,
                    "timestamp1": 411107223370,
                    "timestamp2": 531097181460,
                    "transformation": {
                        "rx": -0.000396419,
                        "ry": -0.0103704,
                        "rz": -0.0289703,
                        "x": 1.29628,
                        "y": 0.61089,
                        "z": -0.138204
                    }
                }
            ]
        }


Pose Optimizer Command
----------------------

This guide uses the `Sample Data`_ to demonstrate the Pose Optimizer. 
After installing the Ouster SDK, downloading the OSF file and creating the JSON constraints file as described earlier, you can run the Pose Optimizer using the command:

.. code:: bash

   ouster-cli source <OSF_FILENAME> pose_optimize --config <CONSTRAINT_JSON_FILE> <OUTPUT_OSF_FILENAME>

The ``--config`` option is optional. But if it's omitted, no constraints are loaded and optimizer will use its default settings.:


Using the Visualizer (--viz)
-----------------------------

Add ``--viz`` to start the interactive trajectory visualizer alongside the
optimizer:

.. code:: bash

   ouster-cli source <OSF_FILENAME> pose_optimize --viz --config <CONSTRAINT_JSON_FILE> <OUTPUT_OSF_FILENAME>

Key bindings in the viz:

- ``n`` solve one optimization step.
- ``Shift`` + ``n`` solve ten optimization steps.
- ``Shift`` + ``Ctrl`` + ``n`` iterate until solver convergence.
- ``p`` increase point cloud point-size (hold ``Shift`` to decrease).
- ``Shift`` + ``s`` save the optimized trajectory to the output OSF.
- ``g`` toggle sampled scan clouds on/off.
- ``r`` toggle the raw trajectory overlay on/off.
- ``t`` toggle constraint labels.
- ``m`` toggle sampling mode between keyframes and full columns.
- ``o`` toggle node orientation arrows (RGB axes).
- ``x`` toggle absolute pose axes (RGB axes).


Visualizing the Results
-----------------------
After completing pose optimization, you can view the point cloud which is registered to the refined trajectory using these Ouster CLI commands:

- Replay the recording and visualize the full registered point cloud

.. code:: bash

   ouster-cli source <OUTPUT_OSF_FILENAME> viz --map

- Export a point cloud file and visualize the complete point cloud

.. code:: bash
   
   ouster-cli source <OUTPUT_OSF_FILENAME> save refined_point_cloud.ply
   ouster-cli source refined_point_cloud-000.ply viz


Automatic GPS Constraints
-------------------------
You can also auto-generate GPS absolute pose constraints from an OSF with GPS data (in
addition to any constraints loaded from ``--config``). Remember the ``--config`` option is optional.

To record a dataset, connect your sensor to receive GPS data over a serial interface and set
the udp_profile_imu to ACCEL32_GYRO32_NMEA. Detailed instructions for configuring the sensor to accept
GPS input can be found in the `Sensor Time Sync`_ documentation:
Note: ``ACCEL32_GYRO32_NMEA`` is only available in firmware 3.2 or later.

- ``--auto-constraints``: Automatically generate and add GPS absolute pose constraints.
- ``--gps-constraints-every-m``: Approximate spacing in meters between constraints,
  computed from distance traveled using lidar scan poses.
- ``--gps-constraints-weights``: Translation weights ``WX,WY,WZ`` (three
  comma-separated numbers >= 0).
- ``--no-initial-align``: In ``--viz`` mode, disables the initial alignment step
  using absolute constraints.

.. code:: bash

   ouster-cli source <OSF_FILENAME> pose_optimize --auto-constraints <OUTPUT_OSF_FILENAME>

   ouster-cli source <OSF_FILENAME> pose_optimize --auto-constraints --config <CONSTRAINT_JSON_FILE> <OUTPUT_OSF_FILENAME>

.. _Sample Data: https://studio.ouster.com/share/QP8WDMOTC3HOI0TD
.. _Sensor Time Sync: https://static.ouster.dev/sensor-docs/image_route1/image_route3/time_sync/time-sync.html#sensor-time-source
