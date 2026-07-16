Indexing & Slicing
------------------

One of the features of the FrameSetSource API, is the ability to use indexing and slicing when accessing the stored frames within the ``LidarFrame`` source. Currently, this
capability is only supported for indexable sources. That is to say, the functionality we are discussing can only be used
when accessing a pcap or an OSF file with indexing turned on. To turn on indexing simply add the ``index`` flag and set
it ``True`` when opening a pcap or OSF file:

.. rubric:: Imports

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-imports]
         :end-before: [doc-etag-slicing-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent: 4

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-imports]
         :end-before: [doc-etag-slicing-imports]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 0


.. rubric:: Read input source using open_source with indexing enabled

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-open-source]
         :end-before: [doc-etag-slicing-open-source]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-open-source]
         :end-before: [doc-etag-slicing-open-source]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 4


Depending on the file size and the underlying file format there can be some delay before the file is fully indexed (OSF
file take much less time than pcap file to index). A progress bar will appear to indicate progress of the indexing.

Once the index is built up, then we can start using utilizing and interact with the ``FrameSetSource`` object to access frames
in the same manner we are dealing with a python list that holds reference to LidarFrame objects. 


To access the 10th LidarFrame and print its frame id, we can do the following:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-print-nth]
         :end-before: [doc-etag-slicing-print-nth]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-print-nth]
         :end-before: [doc-etag-slicing-print-nth]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 4

One can access a specific frame and sensor in one go — e.g., ``source[-1][0].frame_id`` picks the last frame and the first sensor.


In C++, write ``source[{start, stop, step}]``— leaving any entry out with ``nonstd::nullopt``— to get a  ``FrameSetSource`` restricted to that window but
keeping the same iterators, indexes, and metadata. Negative steps are not allowed; for reverse iteration, collect indices first and iterate backwards manually.


Similarly we can access the last LidarFrame object and print its frame_id using:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-print-last]
         :end-before: [doc-etag-slicing-print-last]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-print-last]
         :end-before: [doc-etag-slicing-print-last]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 4


Alternatively we can instead request a range of frames using the python slice operator. For example, to request the first 9
frames from a FrameSetSource and print their frame ids, we can do the following:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-first-n]
         :end-before: [doc-etag-slicing-first-n]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-first-n]
         :end-before: [doc-etag-slicing-first-n]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 4


**Note:** we don't need to add any break here since the operation `source[0:9]` will only yield the first 9 ``LidarFrame(s)``.

To print ``frame_id`` of the last 8 LidarFrames we do:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-last-n]
         :end-before: [doc-etag-slicing-last-n]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-last-n]
         :end-before: [doc-etag-slicing-last-n]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 4


Finally, as you would expect from a typical slice operation you can also use a step value, though reversed
iteration is not supported.

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-step]
         :end-before: [doc-etag-slicing-step]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-step]
         :end-before: [doc-etag-slicing-step]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 4

.. note:: 
   
   Reverse iteration (negative step sizes) is not supported by FrameSetSource such as ``source[10:0:-1]``.
   Collect indices into a vector and iterate in reverse order if needed.

Slicing operator as a FrameSetSource
--------------------------------

The FrameSetSource slice operator ``[::]`` returns a ``FrameSetSource`` scoped to the indicated slice range. This means that the users can pass the object returned by
the slice operator ``[::]`` to any function or code that expects a ``FrameSetSource`` object. 

The following snippet shows few examples to demonstrate this capability:

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-subsource]
         :end-before: [doc-etag-slicing-subsource]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-subsource]
         :end-before: [doc-etag-slicing-subsource]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 4

.. note::

   Invoking ``source2[10][0].frame_id`` would result in an `out of range exception`` since ``source2`` is scoped to 5 frames.
   Also, invoking ``source2[4].frame_id`` would result in an `AttributeError`` since ``source2`` is a list of LidarFrame objects.

Validating LidarFrames
---------------------

The SDK provides utilities to validate data integrity at both the high-level (assembled frames) and low-level (packet streams).

These functions help validate the integrity of an assembled ``LidarFrame``, allowing you to programmatically identify issues like empty columns or missing poses.

**Python**

The ``ouster.sdk.core`` module provides helpers that return the pose directly:

- **first_valid_column_pose(frame)**: Returns the 4x4 pose matrix of the first valid column.
- **last_valid_column_pose(frame)**: Returns the 4x4 pose matrix of the last valid column.
- **frame.body_to_world[column_index]**: Returns the 4x4 body-to-world matrix for the given column.
- **frame.body_to_world**: Returns the full ``(W, 4, 4)`` array of per-column body-to-world transforms.

**C++**

The ``core::LidarFrame`` class provides methods to find the indices, which can then be used to look up the pose:

- **frame.get_first_valid_column()**: Returns the index of the first valid column. Raises ``std::runtime_error`` / ``RuntimeError`` if no valid columns are available.
- **frame.get_last_valid_column()**: Returns the index of the last valid column. Raises ``std::runtime_error`` / ``RuntimeError`` if no valid columns are available.
- **frame.get_column_pose(index)**: Returns the body-to-world matrix for the given column.
- **frame.body_to_world()**: Returns a reference to the full per-column body-to-world field.


.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-first-valid-column]
         :end-before: [doc-etag-slicing-first-valid-column]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-first-valid-column]
         :end-before: [doc-etag-slicing-first-valid-column]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 4

.. tab-set::
   :sync-group: api-lang

   .. tab-item:: Python
      :sync: py

      .. literalinclude:: _snippets/python/slicing.py
         :language: python
         :start-after: [doc-stag-slicing-last-valid-column]
         :end-before: [doc-etag-slicing-last-valid-column]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/python/slicing.py>`__
         :dedent:

   .. tab-item:: C++
      :sync: cpp

      .. literalinclude:: _snippets/cpp/slicing.cpp
         :language: cpp
         :start-after: [doc-stag-slicing-last-valid-column]
         :end-before: [doc-etag-slicing-last-valid-column]
         :class: doc-snippet
         :caption: `View on GitHub <|github-src|docs/features/processing/_snippets/cpp/slicing.cpp>`__
         :dedent: 4


Together, these helpers make it easier to programmatically identify issues like empty columns, missing poses, or incomplete frames before processing.
