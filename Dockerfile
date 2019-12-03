from ubuntu:16.04

RUN apt-get update && apt-get install -y libzmq3-dev clang wget git autoconf libtool curl make build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev llvm libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev python-openssl

RUN curl -L https://github.com/pyenv/pyenv-installer/raw/master/bin/pyenv-installer | bash
ENV PATH="/root/.pyenv/bin:/root/.pyenv/shims:${PATH}"
RUN pyenv install 3.7.3
RUN pyenv global 3.7.3
RUN pyenv rehash
RUN pip3 install pyyaml==5.1.2 Cython==0.29.14 scons==3.1.1 pycapnp==0.6.4 Jinja2==2.10.3


ENV PYTHONPATH=/project

RUN git clone -b master https://github.com/commaai/cereal.git /project/cereal
RUN /project/cereal/install_capnp.sh

WORKDIR /project

COPY SConstruct .
COPY . /project/opendbc

RUN scons -c && scons -j$(nproc)
