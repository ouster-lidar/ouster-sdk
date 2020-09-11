# Preclude false positive calls (-e), report all errors (-x)
set -e -x

# Prepare a build dir
rm -rf build && mkdir build


# Docker build
BASE=$(dirname $(readlink -f "$0"))
DOCK=/shared_sw
docker build -t shared_sw .
docker run --rm \
       -v /${BASE}/build:/${DOCK}/build/ \
       -v /${BASE}/CMakeLists.txt:/${DOCK}/CMakeLists.txt:ro \
       -v /${BASE}/ouster_example:/${DOCK}/ouster_example/:ro \
       -v /${BASE}/sensor_utils:/${DOCK}/sensor_utils/:ro \
       -v /${BASE}/ouster_pcap:/${DOCK}/ouster_pcap/:ro \
       -v /${BASE}/python-client:/${DOCK}/python-client/:ro \
       -v /etc/passwd:/etc/passwd:ro \
       -u $(id -u):$(id -g) \
       shared_sw /bin/bash -c  " \
               set -xe &&\
               cd ${DOCK}/build &&\
               export CMAKE_PREFIX_PATH=\"${DOCK}/ouster_pcap:${DOCK}/ouster_example:${DOCK}/python-client\" &&\
               pip3 wheel -v --no-deps ${DOCK}/python-client &&\
               pip3 wheel -v --no-deps ${DOCK}/ouster_pcap/python-pcap &&\
               cmake .. && make -j$(nproc)
               "
