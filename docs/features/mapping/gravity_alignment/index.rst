.. _gravity-alignment:

=========================
Plumb (Gravity Alignment)
=========================

Plumb, or *gravity alignment*, reports the rotation that brings a frame set
level with gravity, so that applying it leaves Z pointing straight up. A sensor
bolted to a mast, a tripod, or a vehicle roof is almost never perfectly level,
and that residual roll and pitch propagates into every algorithm that reasons
about "up" or "down".

The estimate is a rotation-only correction; nothing is written and the frame
set is never modified. Composing it onto a sensor's
``SensorInfo.sensor_to_body`` extrinsic, as the examples below do, levels
everything read through that sensor afterwards, and preserves the translation
and any previously calibrated mounting offset already there. The ``plumb`` CLI
command does this for you, estimating and applying a correction per sensor so
that each is levelled on its own geometry.

:doc:`Ground segmentation </features/perception/ground_seg/index>`,
:doc:`point cloud alignment <../point_cloud_alignment/index>`, and SLAM all
expect gravity-aligned extrinsics, so plumbing is usually the first command in a
chain. Use the ``plumb`` CLI command, or call
:ouster:func:`plumb <py=ouster.sdk.algorithm.plumb|cpp=ouster::sdk::algorithm::plumb>`
from Python or C++.

Estimation methods
==================

Three ``--method`` choices are available: two base estimators (``imu`` and ``walls``) and
a combined default (``all``):

.. list-table::
   :header-rows: 1
   :widths: 8 52 40

   * - Method
     - How it estimates
     - When to use it
   * - ``imu``
     - Averages the accelerometer samples of the sampled frame to find which way
       gravity points. Each sample is first derotated into the frame of the
       first sample by integrating the gyro, so rotation during the frame does
       not smear the average. Needs no particular scene content, but requires
       the FW 3.2 or later IMU profile; legacy IMU packets are not supported.
     - The right default for most workflows. A single average over the frame's
       accelerometer samples returns immediately, and downstream consumers such
       as ``align`` and ``ground`` do not need the extra fraction of a degree
       the wall refinement buys.
   * - ``walls``
     - Uses scene geometry only, never the IMU. A pixel counts as a wall point
       only when its surface normal is near horizontal and it stands on one
       continuous vertical surface large enough to be a wall, so foliage,
       parked cars and fence posts do not vote. The roll/pitch that minimizes
       the footprint of those points when projected along Z is the one that
       renders the walls plumb: a truly vertical wall collapses to a thin line
       under that projection, while a tilted wall smears into a band.
     - The only option on recordings without the FW 3.2+ IMU profile. The wall
       mask is defined relative to the current orientation, so it assumes the
       sensor already sits roughly level, and it needs a real wall in view: a
       scene that offers none is reported as untrustworthy rather than guessed
       at.
   * - ``all``
     - Default. Estimates gravity from the IMU, then refines that estimate with
       the wall search, keeping the IMU-only result when the scene carries too
       little vertical structure to trust the refinement.
     - Use it when a downstream step is sensitive to small tilts, and for a
       recording that never stands still and shows enough wall: motion leans
       the accelerometer average, the walls do not.

The wall refinement corrects a bounded amount of residual tilt and needs a
minimum number of points standing on a wall; a scene that offers less than
either is reported as untrustworthy instead of applied. Each frame is dewarped
by its own column poses before the walls are read off it, so with
``--method walls`` on a source that carries no poses, prefer a frame where the
sensor is close to stationary.

Using the CLI
=============

The chainable ``plumb`` command estimates a correction for each sensor from the
first frame set and writes it into that sensor's extrinsic. Each stage logs its
estimate, the command prints the resulting correction matrices, and downstream
commands see the updated extrinsics:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-plumb-viz]
   :end-before: [doc-etag-cli-plumb-viz]
   :dedent: 0
   :class: doc-snippet

Select the estimator with ``--method``. Chain ``save`` to persist the levelled
extrinsics into a new OSF file, so later runs do not have to plumb again:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-plumb-imu-save]
   :end-before: [doc-etag-cli-plumb-imu-save]
   :dedent: 0
   :class: doc-snippet

On a source without the FW 3.2+ IMU profile, derive the alignment from scene
geometry instead:

.. literalinclude:: ../../../../python/tests/documentation/test_cli_commands.py
   :language: bash
   :start-after: [doc-stag-cli-plumb-walls-save]
   :end-before: [doc-etag-cli-plumb-walls-save]
   :dedent: 0
   :class: doc-snippet

Each stage logs what it found, and the command prints the correction it
applied:

.. code-block:: text

   [ouster::sdk::core] [info] plumb_imu: IMU of sensor 992616000389 gives the body frame gravity direction (-0.2674, -0.0090, 0.9635)
   [ouster::sdk::core] [info] plumb_walls: segmenting 1 frames, reusing existing WALL fields from 0
   [ouster::sdk::core] [info] plumb_walls: 72334 wall points, correction +1.30 deg about X, -1.60 deg about Y, contrast 1.29
   Plumb correction:
   [[ 0.970674 -0.000979  0.240397  0.      ]
    [ 0.004223  0.999907 -0.01298   0.      ]
    [-0.240362  0.013614  0.970588  0.      ]
    [ 0.        0.        0.        1.      ]]
   Updated the sensor_to_body extrinsic.

A few behaviors are worth knowing:

* **Each sensor is levelled independently**, from its own IMU and its own wall
  geometry. A rig whose relative extrinsics are not yet established still gets
  every sensor's Z axis levelled, but the relative yaw and translation between
  sensors is left untouched. Run :doc:`align <../point_cloud_alignment/index>`
  afterwards to resolve those.
* **The estimate comes from the first frame set at the command's position in the
  chain.** Commands placed before ``plumb``, such as ``slice``, move the
  sampling window.
* ``--method imu`` **errors** when no sensor has usable IMU data, while
  ``--method all`` warns and degrades to wall geometry.
* **A weakly constrained refinement is applied with a warning.** A scene with
  only a single wall or fence constrains one axis and little else; ``plumb``
  says so and applies the refinement anyway.
* **Only the extrinsics change, and only in memory.** The column poses the
  deskew step wrote are restored, so commands after ``plumb`` see the frames as
  they were, and the source file itself is never edited -- chain ``save`` to
  write the levelled extrinsics into a new OSF file.

Using the API
=============

:ouster:func:`plumb <py=ouster.sdk.algorithm.plumb|cpp=ouster::sdk::algorithm::plumb>`
is the estimator behind the ``plumb`` command: it levels from the IMU, then
refines that levelling from wall geometry, and returns one rotation-only
correction to left-multiply onto the current orientation, or none (``None`` in
Python, an empty optional in C++) when no requested stage produced one. Its
``use_imu`` and ``use_walls`` flags select the stages, as ``--method`` does.
Without usable IMU data the wall refinement runs from the current orientation
alone, and without a trustworthy wall solution the IMU levelling stands; each
outcome is logged. Compose the correction onto the existing extrinsics:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/plumb.py
         :language: python
         :start-after: [doc-stag-plumb]
         :end-before: [doc-etag-plumb]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/gravity_alignment/_snippets/python/plumb.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/plumb_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-plumb]
         :end-before: //! [doc-etag-plumb]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/gravity_alignment/_snippets/cpp/plumb_example.cpp>`__
         :dedent: 4

The two stages are also available on their own and return the same result
type.
:ouster:func:`plumb_imu <py=ouster.sdk.algorithm.plumb_imu|cpp=ouster::sdk::algorithm::plumb_imu>`
levels from the accelerometer alone, with no wall search:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/plumb.py
         :language: python
         :start-after: [doc-stag-plumb-imu]
         :end-before: [doc-etag-plumb-imu]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/gravity_alignment/_snippets/python/plumb.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/plumb_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-plumb-imu]
         :end-before: //! [doc-etag-plumb-imu]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/gravity_alignment/_snippets/cpp/plumb_example.cpp>`__
         :dedent: 4

:ouster:func:`plumb_walls <py=ouster.sdk.algorithm.plumb_walls|cpp=ouster::sdk::algorithm::plumb_walls>`
refines from wall geometry; its ``gravity_rotation`` argument is the anchor the
refinement corrects: pass identity to refine the orientation the extrinsics
already carry, or the rotation ``plumb_imu`` returns to refine that estimate
instead. A frame that already carries a ``WALL`` field is read as it lies
rather than segmented again. The frame set is read, never modified.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/plumb.py
         :language: python
         :start-after: [doc-stag-plumb-walls]
         :end-before: [doc-etag-plumb-walls]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/gravity_alignment/_snippets/python/plumb.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/plumb_example.cpp
         :language: cpp
         :start-after: //! [doc-stag-plumb-walls]
         :end-before: //! [doc-etag-plumb-walls]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/mapping/gravity_alignment/_snippets/cpp/plumb_example.cpp>`__
         :dedent: 4

Always check that a result was returned (``None`` in Python, an empty optional
in C++) before applying it, and read the log for the reason when it was not. For
:ouster:func:`plumb_imu <py=ouster.sdk.algorithm.plumb_imu|cpp=ouster::sdk::algorithm::plumb_imu>`
that means no sensor had usable IMU data. For
:ouster:func:`plumb_walls <py=ouster.sdk.algorithm.plumb_walls|cpp=ouster::sdk::algorithm::plumb_walls>`
it means there were no usable frames, too few pixels stood on a wall
(``PLUMB_WALLS_MIN_WALL_POINTS``), or the solution pinned at the edge of the
search range. The wall stage also logs how sharp its minimum was: a scene with
only a single wall or fence constrains one axis and little else, which is
reported as a warning rather than a rejection.
