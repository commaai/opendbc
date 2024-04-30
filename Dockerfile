FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    autoconf \
    build-essential \
    ca-certificates \
    capnproto \
    clang \
    cppcheck \
    curl \
    git \
    libtool \
    make \
    libbz2-dev \
    libffi-dev \
    libcapnp-dev \
    liblzma-dev \
    libncurses5-dev \
    libncursesw5-dev \
    libreadline-dev \
    libssl-dev \
    libsqlite3-dev \
    libzmq3-dev \
    llvm \
    ocl-icd-opencl-dev \
    opencl-headers \
    tk-dev \
    python3-pip \
    python3-dev \
    python3-openssl \
    python-is-python3 \
    xz-utils \
    zlib1g-dev \
    cmake \
  && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/
RUN pip3 install --break-system-packages --no-cache-dir -r /tmp/requirements.txt
RUN pip3 install --break-system-packages --no-cache-dir pre-commit==2.15.0 pylint==2.17.4

ENV PYTHONPATH=/project

RUN git config --global --add safe.directory '*'

WORKDIR /project
RUN git clone https://github.com/commaai/cereal.git /project/cereal && \
    cd /project/cereal && \
    git checkout 861144c136c91f70dcbc652c2ffe99f57440ad47 && \
    rm -rf .git && \
    scons -j$(nproc) --minimal

COPY SConstruct .
COPY ./site_scons /project/site_scons
