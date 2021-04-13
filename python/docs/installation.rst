.. _installation-ref:

Installation
============
Installing the Ouster Python SDK on an x86 architecture device requires ``Python 3.6+`` and ``pip >=
19.0``. Once you have those, you can install the python SDK:: 

    $ pip install ouster-sdk[examples]

That should install ``ouster-sdk`` and some libraries needed to run our examples. If you're on
Windows and wish to run the example code, you may also find that you need PyQt5.

If you're running ARM, a non-glibc-based linux distribution, or wish to develop on the Ouster Python
SDK, you can build from source. See :ref:`devel-instructions`. 
