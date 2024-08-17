FROM python:3.12

WORKDIR /project/opendbc
ENV PYTHONPATH=/project/opendbc

COPY . .
RUN pip3 install --break-system-packages --no-cache-dir .[testing,docs]
RUN ls && rm -rf .git && \
    scons -c && scons -j$(nproc) \
