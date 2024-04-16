##################################################
# Section 1: Build the application
FROM registry.git.chalmers.se/courses/dit638/students/2024-group-21/ubuntu:22.04 as builder
MAINTAINER Christian Berger christian.berger@gu.se

RUN apt-get update -y --allow-unauthenticated && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y

RUN apt-get install -y --no-install-recommends \
        cmake \
        build-essential \
        g++ \
        make \
        lcov

ADD . /opt/sources
WORKDIR /opt/sources

# Tag for test coverage
ENV CXXFLAGS="--coverage"

RUN cd /opt/sources && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Debug .. && \
    make && make test && cp helloworld /tmp

# Runs tests, generates coverage report
WORKDIR /opt/sources/build
RUN lcov --directory . --capture --output-file coverage.info
RUN lcov --remove coverage.info '/usr/*' --output-file coverage.info # filter system-files

###################################################
# Section 2: Bundle the application.
FROM registry.git.chalmers.se/courses/dit638/students/2024-group-21/ubuntu:22.04
MAINTAINER Christian Berger christian.berger@gu.se


RUN apt-get update -y --allow-unauthenticated && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y

COPY --from=builder /opt/sources/build/coverage.info /opt/coverage.info
WORKDIR /opt
COPY --from=builder /tmp/helloworld .
ENTRYPOINT ["/opt/helloworld"]