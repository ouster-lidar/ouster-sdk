version.h
=========

.. contents::
   :local:

Variables
----------

.. doxygenvariable:: ouster::util::invalid_version
   :project: cpp_api

Structs
-------

.. doxygenstruct:: ouster::util::version
   :project: cpp_api
   :members:

Functions
---------

.. doxygenfunction:: ouster::util::version_from_string(const std::string &ver)
   :project: cpp_api


Operators
---------

.. doxygenfunction:: ouster::util::operator==(const version &u, const version &v)
   :project: cpp_api

.. doxygenfunction:: ouster::util::operator!=(const version &u, const version &v)
   :project: cpp_api

.. doxygenfunction:: ouster::util::operator<(const version &u, const version &v)
   :project: cpp_api

.. doxygenfunction:: ouster::util::operator<=(const version &u, const version &v)
   :project: cpp_api

.. doxygenfunction:: ouster::util::operator>(const version &u, const version &v)
   :project: cpp_api

.. doxygenfunction:: ouster::util::operator>=(const version &u, const version &v)
   :project: cpp_api