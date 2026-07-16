:tocdepth: 2

Field and ArrayView
===================

In the C++ SDK, channel data inside a :ouster:class:`LidarFrame <cpp=ouster::sdk::core::LidarFrame>` is represented with
:ouster:class:`Field <cpp=ouster::sdk::core::Field>`, a contiguous multidimensional container.

A field is described by a :ouster:class:`FieldDescriptor <cpp=ouster::sdk::core::FieldDescriptor>`
which describes the memory stored by the field and :ouster:class:`FieldClass <cpp=ouster::sdk::core::FieldClass>`,
an enumeration giving LidarFrame some information about how to categorize the field.

On the python side, fields are directly mapped to NumPy arrays so this document will only cover the C++ side.

Constructing a Field
--------------------

This section describes constructing a standalone field.  To construct a field inside the
:ouster:class:`LidarFrame <cpp=ouster::sdk::core::LidarFrame>`, refer to
:doc:`LidarFrame documentation <lidar_frame>`.

The simplest pattern is to describe an array with ``fd_array`` (or a :ouster:class:`FieldDescriptor <cpp=ouster::sdk::core::FieldDescriptor>`) and pass it
to the :ouster:class:`Field <cpp=ouster::sdk::core::Field>` constructor:

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-field-construct-array]
   :end-before: [doc-etag-field-construct-array]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4

Type restrictions
^^^^^^^^^^^^^^^^^

Standalone fields support any primitive and plain-old-data types when used directly. The mechanism
that provides type safety when accessing the data inside fields, however, is not cross platform
so for serialization purposes we mirror commonly used field types with
:ouster:class:`ChanFieldType <cpp=ouster::sdk::core::ChanFieldType>`. Fields inside
:ouster:class:`LidarFrame <cpp=ouster::sdk::core::LidarFrame>` should only be stored with types
available with :ouster:class:`ChanFieldType <cpp=ouster::sdk::core::ChanFieldType>`.

Newly created fields are zeroed out over the entire content of the field.  Neither constructors nor
destructors are called for stored elements, so fields are best suited for storage of primitive types.

.. _sdk-field-docs-field-access:

Accessing data
--------------

Fields are not designed to provide data access directly. To facilitate that, we use a conversion
mechanism instead - fields support converting to multiple wrapper classes and check type and
dimensional validity when the conversion happens, otherwise throwing ``std::invalid_argument``.

As Eigen
^^^^^^^^

One- and two-dimensional fields can be accessed as Eigen arrays:

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-field-access-via-eigen]
   :end-before: [doc-etag-field-access-via-eigen]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4

As ArrayView
^^^^^^^^^^^^

Alternatively, data can be accessed as :ref:`ArrayView<sdk-field-docs-arrayview>` without Eigen’s
dimension limits:

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-field-access-via-arrayview]
   :end-before: [doc-etag-field-access-via-arrayview]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4

As pointer
^^^^^^^^^^

Data can also be directly accessed as a pointer of respective type (in such case, no dimension checks
are made):

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-field-access-via-pointer]
   :end-before: [doc-etag-field-access-via-pointer]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4


.. _sdk-field-docs-arrayview:

ArrayView
---------

:ouster:class:`ArrayView <cpp=ouster::sdk::core::ArrayView>` is a lightweight non-owning view over
multidimensional data. The class's main motivation is to provide slicing functionality similar to that
of NumPy ndarrays and avoid dimensional limitations imposed by Eigen matrices.

Typical sources include implicit conversion from a :ouster:class:`Field <cpp=ouster::sdk::core::Field>` /
:ouster:class:`FieldView <cpp=ouster::sdk::core::FieldView>`, or constructing directly from ``T*`` and shape.

Read-only access uses ``ConstArrayView`` (an alias for ``ArrayView<const T, Dim>``).
Convenience aliases ``ArrayView1`` … ``ArrayView4`` (and matching ``ConstArrayView*``) name common
ranks.


Element access
^^^^^^^^^^^^^^

Scalar reads and writes use the ``operator()`` with **exactly** ``Dim`` integer
indices, outer dimension first, the same convention as indexing an ``Eigen::Array`` with
``a(i, j, …)``. Arity is enforced at compile time.  Out-of-bound indices produce undefined behavior
(there is no runtime check).

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-arrayview-element-access]
   :end-before: [doc-etag-arrayview-element-access]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4

Slicing and ``subview``
^^^^^^^^^^^^^^^^^^^^^^^

``subview`` mirrors NumPy-style indexing:

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-arrayview-subview]
   :end-before: [doc-etag-arrayview-subview]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4


Calling ``subview`` with ``keep()`` keeps the full extent of that axis without changing rank.

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-arrayview-subview-keep]
   :end-before: [doc-etag-arrayview-subview-keep]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4


Calling ``subview`` with ``keep(start, end)`` slices the extent of the axis with half-open range ``[start, end)``:

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-arrayview-subview-band]
   :end-before: [doc-etag-arrayview-subview-band]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4


.. _sdk-field-docs-arrayview-reshape:

``reshape``
^^^^^^^^^^^

``reshape(dims...)`` returns a new ``ArrayView`` over the same data range with a potentially different
compile-time rank and a new shape. After reshaping, indexing follows the usual compact layout for the
new dimensions. Reshaping only works for non-sparse views and will throw an ``std::invalid_argument``
error otherwise.

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-arrayview-reshape]
   :end-before: [doc-etag-arrayview-reshape]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4


.. _sdk-field-docs-fieldview:

FieldView
---------

:ouster:class:`FieldView <cpp=ouster::sdk::core::FieldView>` is a non-owning equivalent of
:ouster:class:`Field <cpp=ouster::sdk::core::Field>`, allowing the use of the slicing and reshaping
functionality of :ref:`ArrayView<sdk-field-docs-arrayview>` before converting into a concrete data accessor.

``Field`` and ``FieldView`` share the same implicit conversions (typed pointer, Eigen, ``ArrayView``). For
usage and type rules, see :ref:`sdk-field-docs-field-access`.

Slicing and ``subview``
^^^^^^^^^^^^^^^^^^^^^^^

``FieldView::subview`` uses the same slicing model as :ref:`ArrayView slicing<sdk-field-docs-arrayview>`.
Each argument is either an index (that dimension is fixed and dropped from the logical rank) or a slice
descriptor. ``subview`` returns a **new** ``FieldView`` sharing the same allocation.

Example:

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-field-subview]
   :end-before: [doc-etag-field-subview]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4


``reshape``
^^^^^^^^^^^

For ``FieldView``, ``reshape(dims...)`` follows the same model as :ref:`ArrayView reshape<sdk-field-docs-arrayview-reshape>`:
it keeps the same underlying data pointer. Sparse views cannot be reshaped.

.. literalinclude:: _snippets/cpp/field_example.cpp
   :language: cpp
   :start-after: [doc-stag-fieldview-reshape]
   :end-before: [doc-etag-fieldview-reshape]
   :class: doc-snippet
   :caption: `View on GitHub <|github-src|docs/features/consumption/_snippets/cpp/field_example.cpp>`__
   :dedent: 4
