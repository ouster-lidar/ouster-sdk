Using the API
-------------

New types and functions
^^^^^^^^^^^^^^^^^^^^^^^

The Ouster Perception API introduces the following types and functions:

.. include:: index.rst
   :start-after: perception-core-types-start
   :end-before: perception-core-types-end


Using ``DetectionEngine``
^^^^^^^^^^^^^^^^^^^^^^^^^^

To use the API, call :ouster:func:`DetectionEngine.create <py=ouster.sdk.perception.DetectionEngine.create|cpp=ouster::sdk::perception::DetectionEngine::create>` to
instantiate the engine, passing the ``SensorInfo`` list (even for a single sensor) and an optional
``ClassicDetectionConfig`` (defaults to ``ClassicDetectionConfig{}``).
Then call
:ouster:func:`update <py=ouster.sdk.perception.DetectionEngine.update|cpp=ouster::sdk::perception::DetectionEngine::update>`
method, providing either a ``LidarFrame`` or ``FrameSet`` by reference. The following example describes how to instantiate
the default ``DetectionEngine`` and use it to add objects representing lidar point clusters to every frame in the first
sensor stream in the provided source.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/cluster_example.py
         :language: python
         :start-after: [doc-stag-perception-engine]
         :end-before: [doc-etag-perception-engine]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/cluster_example.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/cluster_example.cpp
         :language: cpp
         :start-after: [doc-stag-perception-engine]
         :end-before: [doc-etag-perception-engine]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/cluster_example.cpp>`__
         :dedent:


Adding ``Object`` to frames without a ``DetectionEngine``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

While the ``DetectionEngine`` interface is the most straightforward way to add objects to frames, users can also add
objects directly to a frame without using a ``DetectionEngine``. This may be useful for users who want to add objects
derived from an external source, such as an annotated dataset, or who want to implement their own detection engine
without inheriting from the base class.

To add an ``Object`` to a frame, simply create an instance of the ``Object`` class and add it to the frame's ``objects``
vector.For example, the following code snippet creates an ``Object`` with poses, velocity, and class ID, and adds
it to a frame:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
         :start-after: [doc-stag-add-object]
         :end-before: [doc-etag-add-object]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/osf_writer_example.cpp
         :start-after: [doc-stag-add-object]
         :end-before: [doc-etag-add-object]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/osf_writer_example.cpp>`__
         :dedent:

Using an approach like the above, ``FrameSet`` can also store ``Object`` instances. Since OSF supports saving
``LidarFrame`` and  ``FrameSet``, reading and writing datasets with objects can be done with
:ouster:class:`OsfFrameSetSource <py=ouster.sdk.osf.OsfFrameSetSource|cpp=ouster::sdk::osf::OsfFrameSetSource>` and
:ouster:class:`Writer <py=ouster.sdk.osf.Writer|cpp=ouster::sdk::osf::Writer>` or
:ouster:class:`AsyncWriter <py=ouster.sdk.osf.AsyncWriter|cpp=ouster::sdk::osf::AsyncWriter>`.

Reading and writing ``ClassMap``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In Ouster SDK 1.0, no ``DetectionEngine`` implementation produces ``Object`` s with classifications, yet. However,
``ClassMap``, which creates an association between ``Object::class_id`` and a string, can still be useful for ``Object``
s imported from an external source, like an annotated lidar dataset.

``ClassMaps`` are stored in a :ouster:class:`FrameSetSourceMetadataSet <py=ouster.sdk.core.FrameSetSourceMetadataSet|cpp=ouster::sdk::core::FrameSetSourceMetadataSet>` and associated with a frame source via metadata keys (for example, ``"class_maps"``).

Writing an OSF with ClassMaps is straightforward, since the 
:ouster:class:`Writer <py=ouster.sdk.osf.Writer|cpp=ouster::sdk::osf::Writer>` and
:ouster:class:`AsyncWriter <py=ouster.sdk.osf.AsyncWriter|cpp=ouster::sdk::osf::AsyncWriter>` support saving
``FrameSetSourceMetadataSet`` like so:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
         :language: python
         :start-after: [doc-stag-write-classmaps]
         :end-before: [doc-etag-write-classmaps]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/osf_writer_example.cpp
         :language: cpp
         :start-after: [doc-stag-write-classmaps]
         :end-before: [doc-etag-write-classmaps]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/osf_writer_example.cpp>`__
         :dedent:


Reading a ``ClassMap`` from a ``FrameSetSource`` is as simple as accessing it using its string key from the frame set source metadata.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: /../python/src/ouster/sdk/examples/osf.py
         :language: python
         :start-after: [doc-stag-read-classmaps]
         :end-before: [doc-etag-read-classmaps]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|python/src/ouster/sdk/examples/osf.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: /../examples/osf_writer_example.cpp
         :language: cpp
         :start-after: [doc-stag-read-classmaps]
         :end-before: [doc-etag-read-classmaps]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|examples/osf_writer_example.cpp>`__
         :dedent:

