:tocdepth: 3

==============================
Migration from 1.0.0 to 2.0.0
==============================


The Ouster SDK 2.0.0 release introduces a **camera data API** and, to support
it, unifies lidar and camera data under a small set of shared base types. It
also lets ``VoxelHashMap`` and ``voxel_downsample`` keep per-point
attributes in their original types (a ``uint32`` RANGE stays a ``uint32``)
instead of widening every attribute to ``double``. This guide covers the API
changes that follow from that work, along with every other breaking change in
2.0.0 (including ``ouster-cli`` and visualizer changes), with **breaking
changes presented first**. Other, backward-compatible 2.0.0 changes are listed
in the changelog.

**Lidar-only code is affected too.** Even if you never touch a camera, the
shared base types change how you name metadata (``SensorInfo`` →
``LidarInfo``), how you reach a frame's metadata (in C++,
``frame.sensor_info`` is now an accessor), and what a ``FrameSet`` yields when you iterate it (``Frame``,
not ``LidarFrame``). All of these are covered below.

How to find code that needs updating
++++++++++++++++++++++++++++++++++++

1. **Build / import failures** — most of the breaking changes produce a compile
   or import error once you upgrade.
2. **Check the changelog** for the ``[BREAKING]`` change summary.

   - `CHANGELOG <../reference/changelog.html>`__
3. **Search your codebase** for ``SensorInfo``, ``SensorConfig``,
   ``sensor_config``, ``sensor_info``, ``packet_timestamp``, and
   ``LidarFrame``; and for ``voxel_downsample``, ``voxel_downsample_xd``,
   ``voxel_downsample_3d``, ``VoxelHashMap`` (any of its ``3d``/``Xd``, ``Average``,
   ``AccumulateAttrib`` or ``PointNormal`` variants), ``point_cloud``, ``min_pts_threshold``
   (and ``--min-pts-threshold``), ``get_local_map``, ``ouster/algorithm/voxel_downsample.h``,
   and any custom voxel bucket or insertion strategy.
4. **Check scripts that call** ``ouster-cli``, in particular ones that use
   ``save`` to a point cloud file (``-f``, ``--decimate``, ``--field``),
   ``plumb``, or open several files at once. Some CLI changes alter results
   without raising an error.

.. warning::

   ``SensorInfo`` is the one to watch. It still exists in 2.0, so code
   mentioning it often still *compiles* while meaning something different: it
   is no longer the lidar metadata type but the shared base of all sensor
   metadata. Anywhere you read a lidar-specific field (``format``,
   ``beam_azimuth_angles``, ``lidar_to_sensor_transform``, ``config``,
   ``prod_line``, ...) you now need a ``LidarInfo``. In Python,
   ``core.SensorInfo(metadata_json)`` still returns a ``LidarInfo`` for lidar
   metadata (see below), but type annotations and ``isinstance`` checks against
   ``SensorInfo`` now match camera metadata too.

Breaking changes
++++++++++++++++

Renamed metadata and configuration types
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The lidar metadata and configuration types were renamed so that ``SensorInfo``
and ``SensorConfig`` can become the shared base classes for **all** sensor
metadata and configuration (lidar and camera).

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - 1.0 name
     - 2.0 name
   * - ``SensorInfo`` (lidar metadata)
     - ``LidarInfo``
   * - ``SensorConfig`` (lidar configuration)
     - ``LidarConfig``

In 2.0, ``SensorInfo`` is repurposed as the shared metadata base class for
``LidarInfo`` and ``CameraInfo``, and ``SensorConfig`` becomes the shared,
currently attribute-less, base class of ``LidarConfig``. Because
``SensorConfig`` no longer has any configuration fields of its own, code that
constructs ``core.SensorConfig()`` and sets attributes on it (``udp_dest``,
``lidar_mode``, ...) must switch to ``core.LidarConfig``.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (1.0)
         from ouster.sdk import core
         info = core.SensorInfo(metadata_json)
         config = core.SensorConfig()

         # After (2.0)
         from ouster.sdk import core
         info = core.LidarInfo(metadata_json)
         config = core.LidarConfig()
         # core.SensorInfo is now the shared base of LidarInfo and CameraInfo,
         # and core.SensorConfig is the shared base of LidarConfig.

      ``core.SensorInfo(metadata_json)`` still works: it acts as a factory and
      returns a ``LidarInfo`` (or a ``CameraInfo`` for camera metadata), so
      1.0 code that parses lidar metadata this way keeps running. The other
      1.0 constructors, ``core.SensorInfo()`` and
      ``core.SensorInfo.from_default(mode)``, are not kept: use
      ``core.LidarInfo()`` and ``core.LidarInfo.from_default(mode)``. Type
      checkers see the factory's result as a ``SensorInfo``, so annotate it as
      ``LidarInfo`` (or construct a ``LidarInfo`` directly) where you read
      lidar-specific fields.

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (1.0)
         ouster::sdk::core::SensorInfo info(metadata);
         ouster::sdk::core::SensorConfig config;

         // After (2.0)
         ouster::sdk::core::LidarInfo info(metadata);
         ouster::sdk::core::LidarConfig config;
         // ouster::sdk::core::SensorInfo and ::SensorConfig are now the shared
         // metadata and configuration base classes.

      The C++ header ``ouster/core/sensor_config.h`` remains, but it now
      declares the shared base ``SensorConfig``. The lidar-specific
      ``LidarConfig`` moved to ``ouster/core/lidar_config.h`` (which includes
      ``sensor_config.h``); include it to get the concrete type:

      .. code-block:: cpp

         // Before (1.0)
         #include "ouster/core/sensor_config.h"

         // After (2.0)
         #include "ouster/core/lidar_config.h"

      The C++ header ``ouster/core/sensor_info.h`` remains, but it now declares the shared base ``SensorInfo``.
      To ensure you get the concrete lidar type, include ``ouster/core/lidar_info.h`` instead.

      .. code-block:: cpp

         // Before (1.0)
         #include "ouster/core/sensor_info.h"

         // After (2.0)
         #include "ouster/core/lidar_info.h"


Source options
^^^^^^^^^^^^^^

The ``sensor_config`` option on ``open_source`` (``PacketSourceOptions`` and
``FrameSetSourceOptions``) keeps its name. In both languages it carries
``LidarConfig`` values, following the ``SensorConfig`` → ``LidarConfig`` rename.
In Python the option accepts the ``SensorConfig`` base type and downcasts each
entry to the concrete ``LidarConfig`` that lidar sources require, so passing a
bare ``SensorConfig`` raises (it carries no configuration). In C++ the option is
typed ``std::vector<LidarConfig>`` directly, so there is no downcast: pass
``LidarConfig`` instances. The ``sensor_info`` option likewise keeps its name but
now takes ``LidarInfo`` values.

.. code-block:: python

   # Before (1.0): config was a core.SensorConfig
   source = open_source("sensor.local", sensor_config=[config])

   # After (2.0): the keyword is unchanged, but config is now a core.LidarConfig
   source = open_source("sensor.local", sensor_config=[config])

Frame metadata accessors
^^^^^^^^^^^^^^^^^^^^^^^^^

In C++, a frame's metadata was a public data member; it is now an accessor, and
writes go through a setter that checks the metadata kind at compile time.

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - 1.0
     - 2.0
   * - ``lidar_frame.sensor_info`` (public member)
     - ``lidar_frame.sensor_info()`` (accessor)
   * - assign to ``lidar_frame.sensor_info``
     - ``lidar_frame.set_sensor_info(info)``

.. code-block:: cpp

   // Before (1.0)
   auto info = lidar_frame.sensor_info;         // public data member
   lidar_frame.sensor_info = new_info;          // direct assignment

   // After (2.0)
   auto info = lidar_frame.sensor_info();       // -> shared_ptr<LidarInfo>
   lidar_frame.set_sensor_info(new_info);       // checked at compile time

The accessor name is uniform across frame kinds, and its result type follows the
type of the frame you hold: the same ``sensor_info()`` call returns a
``CameraInfo`` from a ``CameraFrame``. This is why the accessor is more specific
than the base: ``Frame::sensor_info()`` returns the shared ``SensorInfo``, so
generic code still compiles.

In **Python this section requires no changes** — ``sensor_info`` was already a
property, and remains readable and assignable on ``LidarFrame``. Only its type
changed, from ``SensorInfo`` to ``LidarInfo`` (see the rename above).

.. note::

   There is deliberately no setter on the base ``Frame``, because a base
   ``Frame`` cannot verify that the metadata matches its sensor kind at compile
   time. Assign through the concrete frame type. For generic C++ code that holds
   metadata as a base ``SensorInfo`` and only learns the sensor kind at runtime,
   use ``ouster::sdk::core::impl::assign_sensor_info(frame, info)``, which
   validates the kind and throws ``std::invalid_argument`` on a mismatch.

Source sensor-info accessors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A ``FrameSetSource`` can now expose lidar and camera sensors together, so its
sensor-info accessors changed:

- ``sensor_info`` now returns the shared ``SensorInfo`` base type for **every**
  sensor (lidar or camera), not lidar metadata specifically. The returned list
  is index-aligned with the frame slots of the ``FrameSet`` objects the source
  produces: ``sensor_info[i]`` describes the sensor whose frame is at slot ``i``.

A source deliberately exposes only this one combined list, with no type-filtered
variant. A filtered list drops entries, so its positions no longer line up with
the frame slots — a common source of subtle bugs when a source mixes lidar and
camera streams. Keep the single ``sensor_info`` list and downcast individual
entries (``isinstance`` in Python, ``std::dynamic_pointer_cast`` in C++) when
you need type-specific fields.

.. code-block:: python

   # Before (1.0): sensor_info returned lidar metadata (LidarInfo)
   source = open_source("data.osf")
   lidar_meta = source.sensor_info

   # After (2.0): sensor_info is the combined, frame-slot-aligned list. For a
   # lidar sensor the entry is still a LidarInfo at runtime, so lidar code keeps
   # working; downcast/filter explicitly when handling mixed sources.
   source = open_source("data.osf")
   all_meta = source.sensor_info                     # every sensor (SensorInfo)
   lidar_meta = [m for m in all_meta
                 if isinstance(m, core.LidarInfo)]   # lidar-only, if needed

.. note::

   A ``PacketSource`` went the *other* way. Packet sources remain lidar-only,
   so ``PacketSource::sensor_info()`` narrowed from ``SensorInfo`` to
   ``LidarInfo`` and needs no downcast. Only ``FrameSetSource`` returns the
   shared base.

``FrameSet`` holds ``Frame``, not ``LidarFrame``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Because a ``FrameSet`` can now mix sensor kinds, iterating one yields the
``Frame`` base rather than ``LidarFrame``. Code that reached straight for
lidar-specific members — ``frame_id``, ``measurement_id``, ``frame_status``,
and so on — needs to select by type first.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (1.0): every entry was a LidarFrame
         for frame in frame_set.valid_frames():
             mids = frame.measurement_id

         # After (2.0): select the lidar frames explicitly
         for frame in frame_set.valid_lidar_frames():
             mids = frame.measurement_id

      ``valid_frames()`` still works but now returns ``Frame`` objects. New in
      2.0: ``frames()``, ``valid_lidar_frames()``, and
      ``valid_camera_frames()``.

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (1.0): iterators yielded shared_ptr<LidarFrame>
         for (const auto& frame : frame_set) {
             auto shot = frame->shot_limiting();
         }

         // After (2.0): iterators yield shared_ptr<Frame>
         for (const auto& frame : frame_set.valid_frames_of<LidarFrame>()) {
             auto shot = frame->shot_limiting();
         }

      ``begin()``/``end()``, ``valid_frames()``, and ``valid_indices()`` are
      now expressed in terms of ``Frame``. New in 2.0:
      ``valid_frames_of<FrameT>()``, plus the named helpers
      ``valid_lidar_frames()`` and ``valid_camera_frames()``.

.. note::

   The positions returned by ``valid_lidar_frames()`` /
   ``valid_camera_frames()`` / ``valid_frames_of<T>()`` are **not** frame-slot
   indices — filtering drops entries. Use ``valid_indices()`` when you need to
   line a frame up with its ``sensor_info`` entry.

OSF writer changes
^^^^^^^^^^^^^^^^^^^

Writing OSF is otherwise unchanged, but three pieces of the C++ API moved:

- ``Writer::sensor_info()`` was **removed**, with no replacement on ``Writer``.
  Read metadata back through a reader or ``FrameSetSource::sensor_info()``.
- ``AsyncWriter``'s constructor now takes
  ``const std::vector<std::shared_ptr<SensorInfo>>&`` instead of
  ``const std::vector<SensorInfo>&``, so one writer can mix sensor kinds.
- ``Writer::save`` and ``AsyncWriter::save`` accept ``const Frame&`` instead of
  ``const LidarFrame&``, so they take a ``CameraFrame`` too.

``osf::LidarSensor`` also gained a ``sensor_info()`` **method** returning the
shared base, replacing the ``LidarSensor::sensor_info`` *typedef*, which was
removed. Its existing ``info()`` accessor now throws ``std::runtime_error``
when the entry holds camera metadata; prefer ``sensor_info()`` and downcast
when the sensor kind is not known up front.

``osf::CollationStream`` now addresses each collation slot by its unified
sensor index, so it can reference camera frames. This only affects code that
writes collations through ``CollationStream`` directly:

- ``INVALID_FRAME_UID`` was removed. Use ``INVALID_COLLATION_FRAME_INDEX`` to
  mark a missing slot in ``frame_indices``, and ``LEGACY_INVALID_SCAN_ID`` to
  mark one in ``legacy_scan_ids``.
- ``ResolveFrameFn`` returns ``std::shared_ptr<Frame>`` instead of
  ``std::shared_ptr<LidarFrame>``.
- ``CollationStream::save()`` and ``CollationStream::make_msg()`` take
  ``frame_indices`` (one message index per collation slot) and
  ``legacy_scan_ids`` (lidar slots only, so older SDKs can still read them)
  instead of ``frame_uids``.

OSF reading drops unknown fields instead of throwing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When a frame in an OSF file holds a field whose field type, field class, or
channel field id this SDK version doesn't recognize (for example, one written
by a newer SDK),
decoding no longer throws. The reader reports the field to its error handler
with ``Severity::OUSTER_WARNING`` and drops that field; the rest of the frame
decodes normally. The default error handler logs the warning and continues.

Code that relied on the exception to reject such files must now check the
fields of the decoded frame, or pass an error handler that throws on warnings
(``osf::Reader(file, error_handler)`` and the ``error_handler`` option of
``FrameSetSourceOptions`` in C++, ``osf.Reader(file, error_handler=...)`` in
Python):

.. code-block:: python

   from ouster.sdk import osf

   def strict_handler(severity, message):
       raise RuntimeError(message)

   reader = osf.Reader("data.osf", error_handler=strict_handler)

SLAM and localization engine constructors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``SlamEngine``, ``LocalizationEngine``, ``DeskewMethod``, and
``ActiveTimeCorrection`` are now ``explicit``, so brace-initialization and
implicit conversion at the call site no longer compile. They still take
``std::vector<std::shared_ptr<SensorInfo>>`` — the shared base — so they accept
camera metadata as well.

.. code-block:: cpp

   // Before (1.0)
   SlamEngine engine = infos;

   // After (2.0)
   SlamEngine engine(sensor_infos);

.. warning::

   ``std::vector<std::shared_ptr<LidarInfo>>`` does **not** implicitly convert
   to ``std::vector<std::shared_ptr<SensorInfo>>``, even though the element
   types do. If you hold a typed list, rebuild it:

   .. code-block:: cpp

      std::vector<std::shared_ptr<core::SensorInfo>> sensor_infos(
          lidar_infos.begin(), lidar_infos.end());

In Python, the corresponding **keyword argument was renamed** from ``infos`` to
``sensor_infos`` on ``DeskewMethod``, ``ConstantVelocityDeskewMethod``,
``ActiveTimeCorrection``, and ``DeskewMethodFactory.create``, matching
``SlamEngine.create``. ``infos=`` still works, with a deprecation warning, on
all of these except ``DeskewMethod``, and on ``SlamEngine.create``, which
accepted it silently in 1.0. Use ``sensor_infos``.

.. code-block:: python

   # Before (1.0)
   engine = mapping.SlamEngine.create(infos=metas)
   deskew = mapping.DeskewMethod(infos=metas)

   # After (2.0)
   engine = mapping.SlamEngine.create(sensor_infos=metas)
   deskew = mapping.DeskewMethod(sensor_infos=metas)

Positional calls are unaffected.

``min_pts_threshold`` renamed to ``min_points_per_voxel``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The minimum number of points a voxel needs before it appears in the output is
now called ``min_points_per_voxel`` everywhere, to match
``max_points_per_voxel``. The old name is not kept as an alias, so a 1.0 call
that passes it by name fails: a ``TypeError`` in Python, an unknown option in
``ouster-cli``. Calls that pass it by position are not affected by the rename
itself, but the sections below change the argument order of the functions that
take it.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (1.0)
     - After (2.0)
   * - Python keyword ``min_pts_threshold=`` (``voxel_downsample_3d``,
       ``voxel_downsample_xd``, ``voxel_downsample``, ``VoxelHashMap3d``,
       ``VoxelHashMapXd``)
     - ``min_points_per_voxel=`` (``voxel_downsample``, ``VoxelHashMap.create``)
   * - C++ ``VoxelHashMap<...>::min_pts_threshold()``
     - ``VoxelHashMap::min_points_per_voxel()``
   * - ``ouster-cli source ... save --min-pts-threshold N``
     - ``--min-points-per-voxel N``

``voxel_downsample`` replaces the Python downsamplers and keeps attribute types
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Python now has a single voxel downsampler, ``ouster.sdk.core.voxel_downsample``.
``voxel_downsample_xd`` and ``voxel_downsample_3d`` are removed from the Python
API. In 1.0, ``voxel_downsample`` was only an alias of ``voxel_downsample_xd``;
it is now a function of its own with a different signature, so existing
``voxel_downsample`` calls need the same rewrite as ``voxel_downsample_xd``
calls.

``voxel_downsample_xd`` used to take a single ``Nx(3+K)`` double array (``x, y,
z`` followed by ``K`` attribute columns) and return an ``Mx(3+K)`` double array.
``voxel_downsample`` takes the positions and the attributes separately and
returns them the same way, in the shape ``VoxelHashMap.pointcloud()``
uses. Its parameters follow ``VoxelHashMap.create()``, with the attributes
optional and after ``strategy``. Leave the attributes out to downsample
positions only, in place of ``voxel_downsample_3d``.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (1.0)
     - After (2.0)
   * - ``voxel_downsample_xd(frame, voxel_size, max_points_per_voxel=1, min_pts_threshold=1, strategy=RANDOM)``,
       or the same call through the ``voxel_downsample`` alias
     - ``voxel_downsample(points, voxel_size, max_points_per_voxel=1, min_points_per_voxel=1, strategy=RANDOM, attributes=None, internal_storage_mode=AUTO, attribute_index=0)``
   * - ``frame``: ``Nx(3+K)`` ``float64``
     - ``points``: ``Nx3`` ``float64``; ``attributes``: a list with one array per
       attribute, each ``(N,)`` or ``(N, K_i)``, in any dtype ``AttributeType``
       lists (``None``, the default, for positions only)
   * - Returns an ``Mx(3+K)`` ``float64`` array
     - Returns an ``AttributedPointCloud``: ``.xyz`` (``Mx3`` positions) and
       ``.attributes`` (one array per attribute in its input dtype, shaped
       ``(M,)`` or ``(M, K_i)``); it also unpacks as ``(xyz, attributes)``
   * - ``voxel_downsample_3d(frame, voxel_size, max_points_per_voxel=1, min_pts_threshold=1, strategy=RANDOM)``
       returning ``Mx3``
     - ``voxel_downsample(frame, voxel_size, max_points_per_voxel=1, min_points_per_voxel=1, strategy=RANDOM).xyz``

Which points are kept, and their positions, are unchanged.

The direct translation passes every attribute column as one ``(N, K)``
``float64`` array. That reproduces the 1.0 result exactly; passing each
attribute as its own array in its native dtype (``uint32`` RANGE, ``uint8``
REFLECTIVITY, ...) is what lets 2.0 keep their types and save memory. With
``AVERAGE_POINT``, the per-voxel mean of an integer attribute is rounded to the
nearest representable value.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/tests/test_core.py
          :start-after: [doc-stag-voxel-downsample-xd-migration-py]
          :end-before: [doc-etag-voxel-downsample-xd-migration-py]
          :language: python
          :dedent:

      Keeping each attribute in its own type instead:

      .. code-block:: python

         xyz, (ranges, reflectivity) = voxel_downsample(
             xyz, voxel_size, attributes=[ranges, reflectivity])  # uint32, uint8 in and out

      See ``python -m ouster.sdk.examples.voxel_downsample`` for a
      complete example.

   .. tab-item:: C++
      :sync: cpp

      C++ keeps three downsamplers, all taking positions and then the
      parameters of ``VoxelHashMap::create()``:

      - ``voxel_downsample()`` -- the recommended entry point, and the one
        Python exposes -- takes the ``AttributeLayout`` followed by the packed
        attribute records it describes and returns an ``AttributedPointCloud``.
        It sends positions-only input (an empty layout) to
        ``voxel_downsample_3d()`` and anything else to ``voxel_downsample_xd()``.
      - ``voxel_downsample_xd()`` has the same parameters and result, and
        handles input with or without attributes along the same path.
      - ``voxel_downsample_3d()`` takes positions only and returns an ``Mx3``
        array.

      The ``ArrayXXdR voxel_downsample_xd(frame, ...)`` overload is removed. An
      existing ``Nx(3+K)`` double frame can be viewed as all-double records
      without copying it:

      .. literalinclude:: /../tests/voxel_hashmap_test.cpp
          :start-after: [doc-stag-voxel-downsample-xd-migration]
          :end-before: [doc-etag-voxel-downsample-xd-migration]
          :language: cpp
          :dedent:

      For positions only, leave the layout and records out of
      ``voxel_downsample()``, or call ``voxel_downsample_3d()`` directly:

      .. code-block:: cpp

         const AttributedPointCloud out = voxel_downsample(xyz, voxel_size);
         const ArrayX3dR kept = voxel_downsample_3d(xyz, voxel_size);

      ``voxel_downsample`` itself is not new in C++; its old
      ``(frame, voxel_size)`` form changed as described in
      :ref:`cpp-voxel-downsample-changed`.

``VoxelHashMap`` is now one runtime-configured type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In 1.0, Python had two voxel map classes, ``VoxelHashMap3d`` and
``VoxelHashMapXd``. C++ had the class template
``VoxelHashMap<VoxelIndex, PointType, VoxelBucketType, InsertionStrategy>`` and
its aliases ``VoxelHashMap3d``, ``VoxelHashMapXd``, ``AverageVoxelHashMap3d``,
``AverageVoxelHashMapXd``, ``AccumulateAttribVoxelHashMap`` and
``PointNormalVoxelHashMap3d``. None of these are part of the public API any
more; the name ``VoxelHashMap`` now belongs to a single, non-template class.
Build a map with ``VoxelHashMap::create()`` instead, choosing the per-voxel
reduction with a ``VoxelDownsampleStrategy``. It picks
the positions-only implementation when no attribute layout is given and the
attributed one otherwise, and exposes the same methods (``add_points``,
``pointcloud``, ``get_closest_neighbor``,
``remove_voxels_far_from_location``, ...).

``create()`` does not take its arguments in the old constructors' order:
``max_distance`` moved from second to sixth place, after ``strategy`` and
``attribute_layout``. Rewrite calls with keywords rather than by position.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (1.0)
     - After (2.0)
   * - ``VoxelHashMap3d(voxel_size, max_distance=100.0,
       max_points_per_voxel=20, min_pts_threshold=1)`` (Python and C++)
     - ``VoxelHashMap.create(voxel_size, max_points_per_voxel=20, min_points_per_voxel=1,
       strategy=VoxelDownsampleStrategy.FIRST_N_POINT, max_distance=100.0)``
   * - ``VoxelHashMapXd(voxel_size, max_distance=100.0,
       max_points_per_voxel=20, min_pts_threshold=1, num_attributes=K)``
       (Python and C++)
     - Python: ``VoxelHashMap.create(voxel_size, max_points_per_voxel=20,
       min_points_per_voxel=1, strategy=VoxelDownsampleStrategy.FIRST_N_POINT,
       attribute_layout=[(np.float64, 1)] * K, max_distance=100.0)``;
       C++: the same with ``AttributeLayout::all_double(K)``. Pass no layout for
       ``K=0``.
   * - C++ ``AverageVoxelHashMap3d`` / ``AverageVoxelHashMapXd`` /
       ``AccumulateAttribVoxelHashMap(voxel_size, max_distance,
       max_points_per_voxel, min_pts_threshold[, num_attributes])``
     - ``VoxelHashMap::create(voxel_size, max_points_per_voxel, min_points_per_voxel,
       VoxelDownsampleStrategy::AVERAGE_POINT, layout, max_distance)``, with an
       empty layout for the ``3d`` form
   * - C++ ``PointNormalVoxelHashMap3d``
     - No public replacement

``num_attributes=K`` meant K separate double attributes, hence
``[(np.float64, 1)] * K`` -- not ``[(np.float64, K)]``, which is a single
attribute K values wide (``pointcloud()`` would return one (M, K)
array instead of K (M,) arrays). The ``num_attributes`` argument is gone.

Adding points keeps the 1.0 forms: ``add_points(points)`` with an ``Nx(3+K)``
double array, and in C++ also a ``std::vector`` of points. 2.0 adds
``add_points(xyz, attributes)`` (C++: ``add_points(xyz, records)``) for
attributes in their own types.

Reading points back changed. In 1.0, C++ ``pointcloud()``, Python
``point_cloud()`` (note the underscore) and ``extract_voxels_far_from_location()``
returned a single ``Mx(3+K)`` double array. In 2.0 the Python method is
``pointcloud()``, like C++, and they all return the positions and the
attributes in their own types, as an ``AttributedPointCloud``: ``.xyz`` holds
the positions and, in Python, ``.attributes`` one array per attribute. The
Python object also unpacks like an ``(xyz, attributes)`` pair, and ``len()`` of
it is the number of points, as it was for the 1.0 array.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (1.0)
     - After (2.0)
   * - C++ ``map.pointcloud()``, Python ``voxel_map.point_cloud()``
       (positions only)
     - ``map.pointcloud().xyz`` (C++ and Python)
   * - C++ ``map.pointcloud()``, Python ``voxel_map.point_cloud()``
       (with attributes, all double)
     - C++: ``cloud.attribute<T>(i)`` or ``cloud.attributes_as<Struct>(...)`` on
       ``const AttributedPointCloud cloud = map.pointcloud();``;
       Python: ``xyz, attributes = voxel_map.pointcloud()``, or
       ``np.column_stack([xyz, *attributes])`` for the old single double array
   * - ``map.extract_voxels_far_from_location(origin)``
     - Same as ``pointcloud()`` above: C++ ``.xyz`` / typed accessors; Python
       ``xyz, attributes = voxel_map.extract_voxels_far_from_location(origin)``
   * - C++ ``other.add_points(map.pointcloud())``, Python
       ``other.add_points(voxel_map.point_cloud())``
     - C++: ``other.add_points(map.pointcloud())``; Python:
       ``other.add_points(*voxel_map.pointcloud())``

Mind the defaults: ``create()`` follows ``voxel_downsample()``
(``max_points_per_voxel=1``, ``strategy=RANDOM``) and has no default
``voxel_size``. In 1.0 the ``VoxelHashMap3d``/``Xd`` constructors, in C++ and
Python, defaulted to ``max_points_per_voxel=20`` and kept the first N points
of each voxel; the Python constructors also defaulted to ``voxel_size=0.1``.
Code that relied on those defaults, rather than passing them, now gets one
random point per voxel. Pass them explicitly to keep the old behavior;
``max_distance`` (100.0) and ``min_points_per_voxel`` (1) keep their
defaults.

``VoxelHashMap`` speaks ``Eigen::VectorXd`` points (xyz followed by the
attributes as doubles) in C++; for a positions-only map a point is exactly
three values. For per-point nearest-neighbor queries in hot loops use
``get_closest_neighbor_position(Eigen::Vector3d, max_distance_sq)``, which
returns only the position and does not allocate.

Code that uses these maps through other APIs changes with them:

- ``SlamEngine::get_local_map()`` and ``LocalizationEngine::get_local_map()``
  return ``VoxelHashMap&``. In Python they now return a live view of the
  engine's map, kept valid by the engine, instead of a copy. Code that used to
  modify the returned map (``add_points``, ``remove_voxels_far_from_location``,
  ``clear``, ...) now modifies the map the engine aligns against. To work on a
  snapshot, copy the points out and build your own map:

  .. code-block:: python

     local = slam.get_local_map()
     snapshot = VoxelHashMap.create(voxel_size, max_points_per_voxel=20,
                                    strategy=VoxelDownsampleStrategy.FIRST_N_POINT)
     snapshot.add_points(*local.pointcloud())

- ``ICPRegistration::align_points_to_map`` takes a ``const VoxelHashMap&``,
  replacing its ``VoxelHashMap3d`` and ``VoxelHashMapXd`` overloads.

Custom voxel buckets and insertion strategies are no longer a supported
extension point.

.. _cpp-voxel-downsample-changed:

C++ ``voxel_downsample`` has a new signature and behavior
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The name ``ouster::sdk::core::voxel_downsample`` is kept but now belongs to a
different function. In 1.0 it reservoir-sampled one point per voxel and
returned the kept input points together with the original row index of each.
In 2.0 it is the general downsampler described above: it takes the parameters
of ``voxel_downsample_xd`` and returns an ``AttributedPointCloud``, sending
positions-only input to ``voxel_downsample_3d``.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Before (1.0)
     - After (2.0)
   * - ``voxel_downsample(const std::vector<Eigen::Vector3d>& frame, double voxel_size)``
     - ``voxel_downsample(const Eigen::Ref<const ArrayX3dR>& xyz, voxel_size,
       max_points_per_voxel = 1, min_points_per_voxel = 1, strategy = RANDOM,
       attribute_layout = {}, records = {}, internal_storage_mode = AUTO,
       attribute_index = 0)``; view a ``std::vector`` of points as an ``Nx3`` array
       (``Eigen::Map<const ArrayX3dR>``)
   * - Returns ``std::pair<std::vector<Eigen::Vector3d>, std::vector<std::uint32_t>>``:
       the kept points and the original row of each
     - Returns ``AttributedPointCloud``: kept positions, plus attribute records
       (zero columns without attributes); no indices
   * - Kept points are exact copies of input points
     - Kept positions are quantized to a fraction of the voxel under the default
       ``AUTO`` storage; pass ``VHMInternalStorageMode::ABS`` for exact ones

A ``std::vector<Eigen::Vector3d>`` does not convert to the new ``Nx3`` array
parameter, so every 1.0 call fails to compile. Once you pass an array, the
call resolves to the new function with the same defaults; also update the code
that reads the result as a pair (``.first``, ``.second``,
``auto [points, indices] = ...``).

To get the old result back, carry each point's row as a ``uint32`` attribute
through a random selection, with ``ABS`` storage so the kept points come back
exactly:

.. code-block:: cpp

   const AttributeLayout layout({{AttributeType::U32, 1}});
   std::vector<std::uint32_t> indices(static_cast<std::size_t>(xyz.rows()));
   std::iota(indices.begin(), indices.end(), 0u);
   AttributeRecords rows(xyz.rows(), layout.stride());
   layout.pack(0, indices, rows);
   // ABS keeps the kept positions exact, as the 1.0 voxel_downsample() returned
   // them; the default AUTO storage would quantize and dither them.
   const AttributedPointCloud kept = voxel_downsample(
       xyz, voxel_size, /*max_points_per_voxel=*/1, /*min_points_per_voxel=*/1,
       VoxelDownsampleStrategy::RANDOM, layout, rows, VHMInternalStorageMode::ABS);
   // kept.xyz are the input points; kept_indices[i] is the original row of kept.xyz.row(i).
   const std::vector<std::uint32_t> kept_indices = kept.attribute<std::uint32_t>(0);

The SDK keeps a private copy of the old function as
``ouster::sdk::algorithm::impl::voxel_downsample_with_index`` (in
``ouster/algorithm/impl/custom_voxel_downsample.h``). It is not supported for
external use, and may change or disappear without notice.

In Python, ``ouster.sdk.core.voxel_downsample`` was an alias of
``voxel_downsample_xd`` and never had the index-returning form; see the
previous section for how it changed.

``ouster/algorithm/voxel_downsample.h`` removed from the public API
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The C++ header ``ouster/algorithm/voxel_downsample.h`` no longer exists, so
``#include <ouster/algorithm/voxel_downsample.h>`` fails to compile. Its only
declaration, ``ouster::sdk::algorithm::voxel_downsample_with_normals``, has no
public replacement: the SDK keeps a copy for its own use as
``ouster::sdk::algorithm::impl::voxel_downsample_with_normals`` (in
``ouster/algorithm/impl/custom_voxel_downsample.h``), which is not supported
for external use. To downsample points while keeping a per-point normal,
carry the normal as a ``{F32, 3}`` or ``{F64, 3}`` attribute through
``voxel_downsample()``. With ``AVERAGE_POINT`` the normals are averaged but,
unlike the old function, not re-normalized; with the other strategies each
kept point keeps its own normal.

``ouster-cli ... save`` writes typed PLY and PCD fields
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Exported fields now keep their own type in PLY and PCD files (e.g. ``property
uint range``, ``property uchar reflectivity``, PCD ``TYPE U``) instead of every
field being written as a 32-bit float. Tools that read these files through the
header adapt automatically; code that assumed every property is a ``float``
needs to read the declared type instead.

``ouster-cli ... save`` point cloud options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Saving to a PLY, PCD, LAS, or LAZ file changed in three ways that affect
existing command lines:

- ``-f`` is now the short form of ``--field`` instead of ``--pts-per-file``.
  ``-f N`` now requests a field named ``N`` and fails because the frame has no
  such field. Use the long form ``--pts-per-file N``, which is unchanged.
- ``--decimate`` is removed. Pass ``--voxel-size 0`` (any value ``<= 0``)
  instead of ``--decimate False`` to write every point without downsampling.
- ``--field`` no longer defaults to ``REFLECTIVITY``. When ``--field`` is not
  given, the first available field from ``RGB``, ``REFLECTIVITY``, ``SIGNAL``,
  ``NEAR_IR`` is exported, so a source that has an ``RGB`` field now exports
  colors instead of reflectivity. Pass ``--field REFLECTIVITY`` to keep the 1.0
  output. ``--field NONE`` is no longer special-cased; use ``--field-none`` to
  export only XYZ points.

.. code-block:: bash

   # Before (1.0)
   ouster-cli source data.osf save map.ply -f 1000000 --decimate False
   ouster-cli source data.osf save map.ply --field NONE

   # After (2.0)
   ouster-cli source data.osf save map.ply --pts-per-file 1000000 --voxel-size 0 --field REFLECTIVITY
   ouster-cli source data.osf save map.ply --field-none

``ouster-cli ... plumb`` also levels from wall geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``plumb`` has a new ``--method [imu|walls|all]`` option that defaults to
``all``. In 1.0, ``plumb`` leveled each sensor from the IMU alone. In 2.0, a
bare ``plumb`` aligns with the IMU and then refines the result from the
scene's wall geometry, so the ``sensor_to_body`` it writes can differ from the
1.0 result. Pass ``--method imu`` to level from the IMU alone. Its estimate
comes from the first sampled frame set, so the result can still differ
slightly from 1.0, which averaged readings over the start of the stream.

The IMU methods also require the FW 3.2+ IMU profiles (``IMU_ACC`` and related
fields in the lidar frames); legacy IMU packets are no longer read. On a
source without those fields, ``--method imu`` fails, and ``--method all``
(the default) warns and levels from wall geometry only. When ``ouster-cli``
opens a FW 3.2+ sensor, it switches the IMU profile to
``ACCEL32_GYRO32_NMEA`` unless you pass ``--no-defaults``. Recordings made
with the legacy IMU profile can only be leveled with ``--method walls``.

.. code-block:: bash

   # Before (1.0): IMU only
   ouster-cli source data.osf plumb save leveled.osf

   # After (2.0): same IMU-only behavior
   ouster-cli source data.osf plumb --method imu save leveled.osf

Multiple file sources play back in lockstep
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In 1.0, ``ouster-cli source --glob`` was the only way to open several file
sources at once (a comma-separated list of files raised an error), and it
merged them into one stream ordered by timestamp. In 2.0, several non-sensor
sources, given with ``--glob`` or as a comma-separated list, play back side by
side in lockstep for comparison instead. Pass ``--time`` for the 1.0 behavior.
``--sensor-idx`` is only supported together with ``--time``.

.. code-block:: bash

   # Before (1.0): merged by timestamp
   ouster-cli source --glob 'logs/*.osf' viz

   # After (2.0)
   ouster-cli source --glob --time 'logs/*.osf' viz

``XYZLut`` no longer takes ``use_extrinsics``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``use_extrinsics`` argument is removed from the ``XYZLut`` and
``XYZLutFloat`` constructors. A LUT now always applies the sensor info's
``sensor_to_body`` transform and returns points in the body frame, which is
what the cached ``xyzlut()`` accessors already did.

In Python, ``XYZLut(info)`` previously defaulted to ``use_extrinsics=False``,
so code that relied on the default now gets body-frame points. The two frames
only differ when ``sensor_to_body`` is not the identity matrix. To get
sensor-frame points, build the LUT from a copy of the sensor info with
``sensor_to_body`` set to the identity matrix. Both ``LidarInfo`` and
``CameraInfo`` support ``copy.copy``.

If you project frames from one sensor repeatedly, prefer the LUT cached on the
sensor info over constructing a new ``XYZLut`` each time. The cache is not
updated automatically: call ``clear_cache()`` after changing
``sensor_to_body``.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (1.0)
         body_lut = core.XYZLut(info, use_extrinsics=True)
         sensor_lut = core.XYZLut(info, use_extrinsics=False)

         # After (2.0)
         body_lut = core.XYZLut(info)  # or the cached info.xyzlut_double
         sensor_frame_info = copy.copy(info)
         sensor_frame_info.sensor_to_body = np.eye(4)
         sensor_lut = core.XYZLut(sensor_frame_info)

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (1.0)
         core::XYZLut body_lut(info, true);
         core::XYZLut sensor_lut(info, false);

         // After (2.0)
         core::XYZLut body_lut(info);  // or the cached info.xyzlut<double>()
         core::LidarInfo sensor_frame_info = info;
         sensor_frame_info.sensor_to_body = core::mat4d::Identity();
         core::XYZLut sensor_lut(sensor_frame_info);

Because a LUT always applies ``sensor_to_body``, ``SensorInfo`` (and so both
``LidarInfo`` and ``CameraInfo``) now defaults it to the identity matrix
instead of the zero matrix. This applies when the info is default-constructed
and when ``CameraInfo`` metadata omits the key; ``LidarInfo`` metadata
without an extrinsic already parsed to the identity.

A ``CameraInfo`` without a ``model``, or with a zero width or height, produces
an empty (0x0) ``XYZLut`` instead of raising. Calling that LUT on a frame or
range image raises, and dewarping a ``FrameSet`` (including SLAM and
localization) skips such frames. Dewarping a ``FrameSet`` also skips camera
frames whose LUT can't be built, such as a ``model`` whose focal length was
changed to zero after construction; errors building a lidar LUT still raise.

The C++ ``core::dewarp()`` overload for a ``FrameSet`` no longer takes a list
of LUTs: ``core::dewarp<double>(frame_set, min_range, max_range)`` projects each
frame with the LUT cached on its own ``sensor_info()``. To project with a
different LUT, dewarp each frame with
``core::dewarp<double>(frame, xyzlut, min_range, max_range)``.

``UDPProfileLidar`` is a string
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``UDPProfileLidar`` is no longer an enum, so profiles registered at runtime
need no enum value. A lidar profile is now its name as a string, e.g.
``"RNG19_RFL8_SIG16_NIR16"``, and ``UDPProfileLidar`` only holds constants for
the built-in names: a C++ namespace of ``const char*`` constants, and a plain
Python class of ``str`` attributes in ``ouster.sdk.core``. Members such as
``UDPProfileLidar::RNG19_RFL8_SIG16_NIR16`` still exist, but they are strings
now. Fields that held the enum hold a ``std::string`` instead:
``DataFormat::udp_profile_lidar`` and ``PacketFormat::udp_profile_lidar``, and
``LidarConfig::udp_profile_lidar`` is an ``optional<std::string>``.

- C++ ``to_string(UDPProfileLidar)`` and ``udp_profile_lidar_of_string()``
  are removed, along with ``MAX_NUM_PROFILES``. The profile already is the
  string.
- In Python, the enum methods are gone: ``UDPProfileLidar.from_string()``,
  ``UDPProfileLidar.values()``, ``.value`` / ``int()``, and ``.name``. Use the
  string itself.
- ``add_custom_profile(name, fields, chan_data_size)`` returns the profile's
  name instead of an allocated enum value. It throws ``std::invalid_argument``
  (``ValueError`` in Python) if the name is already registered, and there is
  no longer a limit on the number of profiles.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Before (1.0)
         profile = core.UDPProfileLidar.from_string("RNG19_RFL8_SIG16_NIR16")
         name = str(info.format.udp_profile_lidar)

         # After (2.0)
         profile = "RNG19_RFL8_SIG16_NIR16"  # or core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16
         name = info.format.udp_profile_lidar

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Before (1.0)
         core::UDPProfileLidar profile = info.format.udp_profile_lidar;
         std::string name = core::to_string(profile);

         // After (2.0)
         const std::string& profile = info.format.udp_profile_lidar;
         if (profile == core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16) { /* ... */ }

Other signature changes
^^^^^^^^^^^^^^^^^^^^^^^^

These are mostly source-compatible — a ``LidarFrame`` still binds to a
``Frame&`` parameter — but they will show up in explicit template arguments,
function pointers, and overrides.

- **Widened from** ``LidarFrame`` **to** ``Frame``: ``align_clouds()``,
  ``clip()``, ``filter_field()``, ``filter_uv()``, ``mask()``, ``dewarp()``,
  ``DetectionEngine::update()``, and ``XYZLut::operator()``.
- **Renamed metadata type in the signature** (``SensorInfo`` → ``LidarInfo``):
  ``PacketFormat``'s constructor, ``get_format()``, the ``validate*()``
  functions, C++ ``destagger()``, ``ZoneMonitor::render()``,
  ``get_field_types()``, ``FrameBatcher``'s constructors,
  ``select_by_index_metadata()``, ``reduce_by_factor_metadata()``,
  ``metadata_from_json()``, ``viz::Cloud``'s constructor,
  ``ObjectOverlay::set_sensor_info()``, and ``osf::Writer``'s single-sensor
  and ``std::vector`` constructors (``osf::Writer`` also accepts
  ``std::vector<std::shared_ptr<SensorInfo>>`` for mixed sensor kinds).
- **Renamed configuration type in the signature** (``SensorConfig`` →
  ``LidarConfig``): ``sensor::get_config()`` and ``sensor::set_config()``.
- ``DetectionEngine::create()`` went the other way: it takes
  ``std::vector<std::shared_ptr<SensorInfo>>`` instead of ``LidarInfo``, so it
  accepts camera metadata. As with ``SlamEngine`` above, a typed
  ``std::vector<std::shared_ptr<LidarInfo>>`` does not implicitly convert and
  has to be rebuilt.
- ``LidarInfo::w()`` and ``LidarInfo::h()`` now return ``size_t`` (they
  returned ``uint32_t`` in 1.0) and override the virtuals on ``SensorInfo``.
- The ``rcvbuf_size`` option of ``PacketSourceOptions`` /
  ``FrameSetSourceOptions`` (``open_source(..., rcvbuf_size=...)`` in Python)
  and the ``SensorPacketSource`` constructor now defaults to ``0``, which
  estimates the UDP receive buffer size from the sensors, instead of 1 MiB.
  Pass ``1024 * 1024`` to keep the old size.
- ``LidarFrame``'s single-argument constructor, ``PacketFormat``'s
  constructor, and ``viz::Cloud``'s metadata constructor are now ``explicit``.
- C++ ``SensorInfo::xyzlut<T>()`` returns a
  ``std::shared_ptr<const XYZLutT<T>>`` that shares ownership of the cached
  LUT, rather than a ``const XYZLutT<T>&``, so the LUT stays valid after
  ``clear_cache()``. Hold the pointer by value
  (``const auto lut = info.xyzlut<double>();``) instead of binding a
  ``const XYZLut&``, and dereference it to apply the LUT
  (``(*lut)(frame)``), to pass it to functions that take an ``XYZLutT``, or to
  read ``direction``, ``offset``, ``w``, and ``h`` (``lut->w``).
- In C++, ``FieldType``, ``LidarFrameFieldTypes``, and their helpers moved out
  of ``ouster/core/lidar_frame.h`` into a new ``ouster/core/field_type.h``.
  Most of ``LidarFrame``'s members moved up into ``Frame``
  (``ouster/core/frame.h``). Both remain reachable through inheritance and
  transitive includes, so this only matters if you were including
  ``lidar_frame.h`` solely for ``FieldType``.

Visualizer changes
^^^^^^^^^^^^^^^^^^

- In Python, ``viz.ObjectOverlay.set_sensor_info(info)`` is replaced by a
  read/write ``sensor_info`` property. The C++
  ``ObjectOverlay::set_sensor_info()`` setter is unchanged.

  .. code-block:: python

     # Before (1.0)
     overlay.set_sensor_info(info)

     # After (2.0)
     overlay.sensor_info = info

- In C++, ``viz::Cloud::set_offset()`` is now protected. A structured cloud's
  offsets come from the LUT or sensor info it is constructed from. To use
  custom offsets, pass them to the constructor with
  ``Cloud(w, h, dir, off, extrinsic)``.

Python-only changes
^^^^^^^^^^^^^^^^^^^^

- ``destagger()`` now takes any ``SensorInfo`` rather than a ``LidarInfo``. A
  camera image is not staggered, so for a non-lidar sensor it validates the
  image dimensions and returns it unchanged.
- The ``xyzlut_float`` and ``xyzlut_double`` properties and the
  ``clear_cache()`` method moved from ``LidarInfo`` up to ``SensorInfo``, so a
  ``CameraInfo`` caches its LUT too.
- ``FrameSet(frames=...)`` accepts ``Frame`` objects and ``frames`` now
  defaults to an empty list.

Removed APIs (deprecated in 1.0)
++++++++++++++++++++++++++++++++

APIs that emitted deprecation warnings in 1.0 are **removed** in 2.0.0, so
code that still uses them fails to compile or import. Their replacements are
listed in the "Recommended updates" and "Scan-to-Frame API renames" sections
of :doc:`migration-0.16.2-1.0`. They include:

- The ``scan``-named type aliases and their ``frame`` replacements:
  ``LidarScan``, ``LidarScanSet``, ``LidarScanFieldTypes``, ``ScanSource``
  and its ``Options``/``Metadata``/``MetadataSet``, ``ScanBatcher``,
  ``ScanIterator``, ``ScanSourceWrapper``, ``AnyScanSource``,
  ``MultiScanSource``, ``PcapScanSource``, ``OsfScanSource``,
  ``SensorScanSource`` (and their ``Options``), ``ClippedScanSource``,
  ``MaskedScanSource``, ``ReducedScanSource``, and in OSF
  ``LidarScanEncoder``, ``PngLidarScanEncoder``, ``ZPngLidarScanEncoder``,
  ``LidarScanStream``, ``LidarScanStreamMeta``, ``ScanSourceMetadataStream``,
  ``OsfDropScanError``, ``ScanId``, ``ResolveScanFn``, ``INVALID_SCAN_ID``,
  ``ScanData`` and ``ScanChannelData``. In Python, also ``LidarScanViz``,
  ``LidarScanVizAccumulatorsConfig`` and ``ls_show``.
- The ``scan``-named forwarding headers (``ouster/core/lidar_scan.h``,
  ``ouster/core/lidar_scan_set.h``, ``ouster/core/scan_source.h``,
  ``ouster/core/scan_source_utils.h``, ``ouster/osf/lidarscan_encoder.h``,
  ``ouster/osf/png_lidarscan_encoder.h``, ``ouster/osf/zpng_lidarscan_encoder.h``,
  ``ouster/osf/stream_lidar_scan.h``, ``ouster/osf/osf_scan_source.h``,
  ``ouster/pcap/pcap_scan_source.h``, ``ouster/sensor/sensor_scan_source.h``) and
  Python modules (``ouster.sdk.core.scan_ops``, ``clipped_scan_source``,
  ``masked_scan_source``, ``reduced_scan_source``,
  ``ouster.sdk.bag.bag_scan_source``, ``ouster.sdk.viz.scans_accumulator``,
  ``ouster.sdk.examples.lidar_scan``).
- Renamed members: ``FieldClass::SCAN_FIELD``, ``FrameSet::scans()`` /
  ``valid_scans()``, ``FrameSetSource::scans_num()``,
  ``SensorFrameSetSource::dropped_scans()`` / ``get_scan()``,
  ``LidarFrame::pose()``, ``get_first_valid_lidar_packet_timestamp()`` /
  ``get_last_valid_lidar_packet_timestamp()``, and Python
  ``SensorInfo.extrinsic``.
- The deprecated Python keyword arguments (``scan=``, ``scans=``,
  ``source_scan=``, ``scan_source=``, ``lidar_scan=``, ...), and the viz
  properties ``scan_num`` and ``scans_per_sec``.
- ``cartesian()``, ``n_cols_of_lidar_mode()``, ``frequency_of_lidar_mode()``,
  the ``add_custom_profile`` overload that takes a profile number, the
  ``LidarFrame`` constructors that lack field types or a shared
  ``SensorInfo``, the default and size-only ``LidarPacket`` / ``ImuPacket`` /
  ``ZonePacket`` constructors, ``FrameBatcher``'s call syntax, and Python
  ``scan_to_packets()`` and ``ouster.sdk.perception.normals``.
- The PCAP reader and writer classes ``PcapReader``, ``IndexedPcapReader``,
  ``PlaybackHandle``, ``PcapWriter``, ``PacketInfo`` and ``replay_*()``, and
  the ``ouster/pcap/pcap.h`` and ``ouster/pcap/indexed_pcap_reader.h`` headers.
  Use ``PcapPacketSource`` to read and ``record_initialize()`` /
  ``record_packet()`` to write.
- ``ouster-cli source <source> metadata``; use ``sensor_info``.

Two 1.0 deprecations remain for now: Python ``PacketFormat.from_metadata()``
(use ``from_info()``) and the ``osf.Encoder(lidar_scan_encoder=...)`` keyword
(use ``frame_encoder=``).

New capabilities
++++++++++++++++

These additions are backward compatible; existing lidar code keeps working.

Unified frame and metadata hierarchy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Lidar and camera types now share common base classes:

- ``Frame`` — base of ``LidarFrame`` and ``CameraFrame``.
- ``SensorInfo`` — base of ``LidarInfo`` and ``CameraInfo``.

Because of this, a single ``FrameSet`` can hold both lidar and camera frames,
and one source may combine sensors of either kind. Collation works across
sensors regardless of type.

``SensorInfo.from_json_string(json_string)`` (C++
``SensorInfo::from_json_string``) parses a metadata string and returns the
right concrete type, inferring the sensor kind from the JSON schema — useful
when you read metadata whose kind you do not know up front. In Python,
``core.SensorInfo(json_string)`` does the same thing.

``SensorInfo`` also exposes, for any sensor kind:

- ``latency`` (C++ ``latency()``) — the delay in seconds between the start of
  a capture and the timestamp reported for the resulting frame.
- ``could_be_synchronized`` (C++ ``could_be_synchronized()``) — whether the
  sensor is configured to synchronize with an external time source, and so may
  share a timebase with other sensors.

Camera data types
^^^^^^^^^^^^^^^^^

New in ``ouster.sdk.core`` (and ``ouster::sdk::core``):

- ``CameraInfo`` — camera metadata (extrinsics, a ``CameraDataFormat`` holding
  the image dimensions and frame rate, and an optional ``CameraModel``); the
  camera counterpart to ``LidarInfo``. When
  ``model`` is ``None`` the imagery can be visualized but not projected into a
  point cloud.
- ``CameraFrame`` — a frame of camera data with named image fields; the camera
  counterpart to ``LidarFrame``.
- ``CameraModel`` — rectified pinhole intrinsics, constructed as
  ``CameraModel(fx, fy, cx, cy)``. It raises ``ValueError`` unless ``fx`` and
  ``fy`` are greater than zero, and has no no-argument form. There are
  deliberately no distortion coefficients: the SDK applies no undistortion, so
  a ``CameraModel`` asserts that its imagery is already rectified. See
  :ref:`camera-data` for that constraint and for the radial-millimeter
  ``RANGE`` convention.
- ``camera_metadata_from_json`` (C++ only) — parse a ``CameraInfo`` from a
  JSON file. In Python, read the file and pass its contents to ``CameraInfo``
  or ``SensorInfo``.

Unified projection with ``XYZLut``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A single ``XYZLut`` now serves both sensor kinds. It constructs from any
``SensorInfo`` (a ``LidarInfo`` or a ``CameraInfo``), and calling it with a
``Frame`` projects that frame's ``RANGE`` field into 3D points.

.. code-block:: python

   # 2.0: one XYZLut for lidar and camera
   lut = core.XYZLut(source.sensor_info[0])  # LidarInfo or CameraInfo
   xyz = lut(frame)                          # LidarFrame or CameraFrame

Camera data in OSF
^^^^^^^^^^^^^^^^^^

Camera data is written and replayed with the same OSF APIs as lidar
(``osf.Writer.add_sensor`` / ``Writer.save`` on write; ``open_source`` /
``OsfFrameSetSource`` on read). Camera calibration is stored in a dedicated
``CameraSensor`` footer entry, separate from the ``LidarSensor`` entries. SDK
1.0 does not recognize that entry and skips it, so it can still read the lidar
streams of a file that also contains cameras.

``Writer.save(stream_index, frame)`` takes a **unified stream index** — the
sensor's position in registration order, the same order returned by
``sensor_info`` on read — for both lidar and camera streams (there is no
separate per-type index). ``add_sensor`` returns that index for both a
``LidarInfo`` and a ``CameraInfo``.
Selecting a single sensor (``--sensor-idx`` / ``.single(i)``) uses the same
unified index, so cameras are selectable too. Per-frame ``timestamp`` and
``body_to_world`` pose both round-trip through OSF for camera frames, just like
lidar. There is no camera ``frame_id``: that is a ``LidarFrame`` member, so a
camera frame is identified by its timestamp.

OSF frame encoders renamed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The OSF frame encoders lost the "Lidar" in their names now that they encode
camera frames as well. The old names remain as **deprecated aliases**, and the
old headers forward to the new ones, so existing code keeps compiling and
running.

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Deprecated (2.0)
     - Replacement
   * - ``LidarFrameEncoder``
     - ``FrameEncoder``
   * - ``PngLidarFrameEncoder``
     - ``PngFrameEncoder``
   * - ``ZPngLidarFrameEncoder``
     - ``ZPngFrameEncoder``
   * - ``Encoder::lidar_frame_encoder()``
     - ``Encoder::frame_encoder()``
   * - ``Encoder(lidar_frame_encoder=...)`` (Python)
     - ``Encoder(frame_encoder=...)``
   * - ``ouster/osf/lidarframe_encoder.h``
     - ``ouster/osf/frame_encoder.h``
   * - ``ouster/osf/png_lidarframe_encoder.h``
     - ``ouster/osf/png_frame_encoder.h``
   * - ``ouster/osf/zpng_lidarframe_encoder.h``
     - ``ouster/osf/zpng_frame_encoder.h``

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Deprecated (2.0)
         encoder = osf.Encoder(osf.PngLidarFrameEncoder(4))

         # Recommended
         encoder = osf.Encoder(osf.PngFrameEncoder(4))

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Deprecated (2.0)
         #include "ouster/osf/png_lidarframe_encoder.h"
         auto encoder = std::make_shared<osf::Encoder>(
             std::make_shared<osf::PngLidarFrameEncoder>(4));

         // Recommended
         #include "ouster/osf/png_frame_encoder.h"
         auto encoder = std::make_shared<osf::Encoder>(
             std::make_shared<osf::PngFrameEncoder>(4));

Camera rendering in the visualizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``SimpleViz`` and ``LidarFrameViz`` accept a mix of ``LidarInfo`` and
``CameraInfo`` and render camera streams alongside lidar: a field-of-view
frustum, the camera images, and a point cloud built from the camera's ``RANGE``
field. Camera frames also support the ``NORMALS`` view mode.

New in ``ouster.sdk.viz``: ``CameraModel`` and its shared base ``StreamModel``
(``SensorModel`` now derives from it), ``CameraClouds``, and the
``camera_frustum_far`` keyword on ``LidarFrameViz`` (default
``DEFAULT_CAMERA_FRUSTUM_FAR``). ``show_all_stream_labels()`` /
``hide_all_stream_labels()`` cover lidar and camera streams; the older
``show_all_sensor_labels()`` / ``hide_all_sensor_labels()`` remain and now
delegate to them.

Note that ``LidarFrameViz.metadata`` returns the **lidars only**; camera
metadata is held separately on the model.

In C++, ``ObjectOverlay::set_camera_info()`` switches an overlay from the lidar
panorama projection to a pinhole projection.

Printing frames of any type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``Frame`` now has a polymorphic string form, so a base ``Frame`` prints as its
concrete type. In C++, ``ouster::sdk::core::to_string(const Frame&)`` works for
lidar and camera frames alike; in Python, ``str(frame)`` works on any frame
(including ``CameraFrame``).

.. code-block:: python

   for frame_set in source:
       for frame in frame_set.valid_frames():   # lidar and camera
           print(frame)                          # prints LidarFrame / CameraFrame

Deprecations
++++++++++++

These APIs still work in 2.0.0 but emit a warning on use and will be removed in
a future release. Update at your convenience.

``packet_timestamp`` renamed to ``receive_timestamp``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A frame's per-packet arrival timestamps are now called ``receive_timestamp``,
since the same header applies to camera frames, which do not arrive as lidar
packets. The old ``packet_timestamp`` names remain as **deprecated aliases**
that forward to the new ones — both the timestamp accessor and the four
``get_*_valid_*`` helpers (including the ``PacketType`` overloads on
``LidarFrame``).

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Deprecated (2.0)
     - Replacement
   * - ``packet_timestamp`` (property / accessor)
     - ``receive_timestamp``
   * - ``get_first_valid_packet_timestamp()``
     - ``get_first_valid_receive_timestamp()``
   * - ``get_last_valid_packet_timestamp()``
     - ``get_last_valid_receive_timestamp()``
   * - ``get_min_valid_packet_timestamp()``
     - ``get_min_valid_receive_timestamp()``
   * - ``get_max_valid_packet_timestamp()``
     - ``get_max_valid_receive_timestamp()``

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. code-block:: python

         # Deprecated (2.0)
         ts = frame.packet_timestamp
         first = frame.get_first_valid_packet_timestamp()

         # Recommended
         ts = frame.receive_timestamp
         first = frame.get_first_valid_receive_timestamp()

   .. tab-item:: C++
      :sync: cpp

      .. code-block:: cpp

         // Deprecated (2.0)
         auto ts = frame.packet_timestamp();
         auto first = frame.get_first_valid_packet_timestamp();

         // Recommended
         auto ts = frame.receive_timestamp();
         auto first = frame.get_first_valid_receive_timestamp();

``viz::Cloud(const XYZLut&)`` constructor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The C++ ``viz::Cloud`` constructor that takes an ``XYZLut`` by reference is
deprecated. It can't share GPU buffers with other clouds built from the same
LUT, so each cloud uploads its own copy. Build the cloud from the sensor info,
or from a ``std::shared_ptr`` to the LUT, such as the one cached on the sensor
info:

.. code-block:: cpp

   // Deprecated (2.0)
   core::XYZLut lut(info);
   viz::Cloud cloud(lut);

   // Recommended
   viz::Cloud cloud(info);
   viz::Cloud cloud_from_lut(info.xyzlut<float>());

Learn more
++++++++++

See the :doc:`Camera Data </features/camera/index>` guide for an overview of the
camera API and end-to-end OSF read/write examples.
