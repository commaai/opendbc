FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    clang \
    cppcheck \
    curl \
    git \
    python3-pip \
    python3-dev \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /project/opendbc
ENV PYTHONPATH=/project/opendbc

COPY . .
RUN pip3 install --break-system-packages --no-cache-dir .[testing,docs]
RUN ls && rm -rf .git && \
    scons -c && scons -j$(nproc) \
