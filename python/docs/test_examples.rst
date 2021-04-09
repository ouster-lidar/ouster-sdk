=====================================
Testing Examples (REMOVE when DONE)
=====================================

.. todo::

   [CLEAN] Remove this page when we are done embedding code.

Ok, let's include some part of ``examples.py``:

.. note::

   literalinclude below also works with files outside of sphinx SOURCE_DIR.
   The correct path to the file should be like ``/../src/ouster/client/_digest.py``

.. literalinclude:: /../src/ouster/sdk/examples.py
   :start-after: [doc-stag-single-scan]
   :end-before: [doc-etag-single-scan]
   :emphasize-lines: 1
   :linenos:


Another example with more lines and more emphasize:
   
.. literalinclude:: /../src/ouster/sdk/examples.py
   :start-after: [doc-stag-display-all-2d]
   :end-before: [doc-etag-display-all-2d]
   :emphasize-lines: 2-3,24-32
   :linenos:

And for completeness some include from ``/src/ouster/client`` location:

.. literalinclude:: /../src/ouster/client/data.py
   :start-at: class ChanField(Enum):
   :end-before: def __init__(
   :linenos:

