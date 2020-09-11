============
Sensor Tools
============

:Maintainers: Dima Garbuzov <dima.garbuzov@ouster.io>
:Description: Internal cli tools for Ouster sensors
:Project-type: lib/Python


Summary
=======
Currently only contains the sensor udp data checker utility used in the firmware integration
tests. You probably don't need to use this yet.

Contains two scripts: ``ouster-checker`` and ``ouster-example`` which are the sensor udp stream
checking utility and an example program using the python bindings, respectively.


Notes
=====
Pinning a dependency to a directory with ``poetry add <path>`` breaks running tox. There is a
separate mechanism to pass ``--find-links`` to tox that must be used instead.

`This bug`_ prevents poetry from installing manylinux2010 wheels **at all** and has been unresolved
for almost two years, which is pretty troubling. See `more discussion here`_.

.. _this bug: https://github.com/python-poetry/poetry/issues/1661
.. _more discussion here: https://github.com/python-poetry/poetry/issues/732
