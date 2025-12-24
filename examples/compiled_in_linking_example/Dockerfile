ARG BASE="ubuntu:22.04"
FROM ${BASE}
ARG APT_PROXY=""

ENV WORKSPACE=/root

RUN /bin/sh -c "if ! [ -z \"$APT_PROXY\" ]; then \
        echo 'Using Proxy for APT'; \
        echo 'Acquire::http::Proxy \"$APT_PROXY\";' > /etc/apt/apt.conf.d/01proxy; \
    fi"

RUN apt-get update \
    && apt-get install -y python3 python3-venv python3-pip \
    && mkdir -p $WORKSPACE/scripts/dev_script_library

COPY ./scripts/dev.py ./scripts/requirements.txt \
    $WORKSPACE/scripts/
COPY ./scripts/dev.py ./scripts/requirements.txt \
    ./scripts/dev_script_library/build_libs.py \
    ./scripts/dev_script_library/context.py \
    ./scripts/dev_script_library/dev_dependencies.py \
    $WORKSPACE/scripts/dev_script_library/

RUN python3 -m venv $WORKSPACE/venv && \
    . $WORKSPACE/venv/bin/activate && \
    cd $WORKSPACE && \
    python3 ./scripts/dev.py utils install-system-packages

COPY . $WORKSPACE/sdk/
COPY tests/pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap examples/compiled_in_linking_example/CMakeLists.txt \
     examples/compiled_in_linking_example/main.cpp  $WORKSPACE/

RUN cd $WORKSPACE && \
    cmake -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF . && \
    cmake --build . --parallel 4

CMD $WORKSPACE/pcap_test /root/OS-2-32-U0_v2.0.0_1024x10.pcap
