# Ros OSF Utilities

## Contents
* `ouster_osf/` contains basic utilities for working with and generating OSF data.

## Building the Ouster OSF utilities
* Ouster OSF requires `yaml-cpp+` which may be installed on Ubuntu with `sudo apt install libyaml-cpp-dev`
* Ouster OSF requires `flatbuffers`. To get it :
  * cd `<flatbuffers_installation_folder>`
  * git clone https://github.com/google/flatbuffers.git
  * `cd flatbuffers; mkdir build; cd build; cmake ..; make; sudo make install`
* To build `ouster_osf` you might need to provide the path to the `ouster_client` directory when you run cmake with `mkdir build; cd build; cmake -DCMAKE_PREFIX_PATH=<path_to_ouster_example> ..; make`.


## Running the OSF utilities

To build and run OSF utilities consider using `warden-dev` and `ros` app which nicely build everything in an integrated fashion. For the list of available tools and some info refer to the [RFC 0010: OSF Explore](apps/doc-rfc/source/rfc/0010-osf-explore.rst).


## Testing ouster_osf

Test facilities embedded into `ouster-ros` project (`apps/ros`) and can be run in a standard `warden-dev` fashion. Though with one caveat ...

Because `osf_<name>_test` uses test-data from the GCS (not yet fully adopted way of storing test data files) it skip the tests that relate on external test data files and reports success for such tests too. In order to use the full test capabilities for osf tests one need to `fetch-test-data` first for the used container name.

Example:
``` bash

### ON HOST MACHINE

# Create container
warden-dev container create wd-osf-test

# Fetch test data from GCS for the created container\
warden-dev container fetch-test-data wd-osf-test

# Go inside container and build the ros project
warden-dev container attach wd-osf-test


### INSIDE warden-dev container 'wd-osf-test'

# Make deps + build
warden-dev app make-deps ros && warden-dev app build ros

# Runs tests with a specifier that we want to run full test set and 
# don't skip them if data is missing
TEST_FULL_SET=true warden-dev app test ros
```

NOTE: In CI tests run in full and test_data is fetching from GCS.

## OSF test data files

Test data files for OSF tests live in the GCS bucket `gs://ouster-build/test-data/lib-osf` this is the common subset of files and variations of data composition that we want to check for all OSF readers: C++/Python/JavaScript.

If one need to add aditional files please add them to the folder under `ouster-build/lib-osf` with descriprtive name.

Test Data related info can be found in [RFC 0012: Test Data](apps/doc-rfc/source/rfc/0012-test-data.rst).

NOTE: If you don't have permissions to add files to GCS bucket, ask Jd/Dima about it:)