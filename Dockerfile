FROM ubuntu:20.04

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
    python-openssl \
    xz-utils \
    zlib1g-dev \
    cmake \
  && rm -rf /var/lib/apt/lists/*

RUN curl -L https://github.com/pyenv/pyenv-installer/raw/master/bin/pyenv-installer | bash
ENV PATH="/root/.pyenv/bin:/root/.pyenv/shims:${PATH}"
RUN pyenv install 3.11.4
RUN pyenv global 3.11.4
RUN pyenv rehash

COPY requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt
RUN pip install --no-cache-dir pre-commit==2.15.0 pylint==2.17.4

ENV PYTHONPATH=/project

RUN git config --global --add safe.directory '*'

WORKDIR /project
RUN git clone https://github.com/commaai/cereal.git /project/cereal && \
    cd /project/cereal && \
    git checkout a4255106b7255e00ae04162f7aa14aa3cae339c3 && \
    rm -rf .git && \
    scons -j$(nproc) --minimal

COPY SConstruct .
COPY ./site_scons /project/site_scons
