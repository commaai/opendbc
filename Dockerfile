FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    autoconf \
    build-essential \
    ca-certificates \
    clang \
    cppcheck \
    curl \
    git \
    libtool \
    make \
    libffi-dev \
    liblzma-dev \
    libncurses5-dev \
    libncursesw5-dev \
    libreadline-dev \
    libssl-dev \
    libsqlite3-dev \
    libzmq3-dev \
    llvm \
    python3-pip \
    python3-dev \
    python3-openssl \
    zlib1g-dev \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /project/opendbc
ENV PYTHONPATH=/project/opendbc

COPY . .
RUN pip3 install --break-system-packages --no-cache-dir .
RUN ls && rm -rf .git && \
    scons -c && scons -j$(nproc) \
