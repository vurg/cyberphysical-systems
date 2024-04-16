##################################################
# Section 1: Build the application
FROM registry.git.chalmers.se/courses/dit638/students/2024-group-21/ubuntu:20.04 as builder
MAINTAINER Christian Berger christian.berger@gu.se

# Environment variable set to avoid the ci/cd pipeline getting stuck on interactive prompts
ENV TZ=Europe/Stockholm
# Specifies non-interaction to stop the pipeline environment getting stuck
ENV DEBIAN_FRONTEND=noninteractive

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

# Tags for test coverage
ENV CXXFLAGS="--coverage"
ENV LDFLAGS="--coverage"

RUN cd /opt/sources && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Debug .. && \
    make && make test && cp helloworld /tmp

# Runs tests, generates coverage report
WORKDIR /opt/sources/build
RUN lcov --directory . --capture --output-file coverage.info --rc geninfo_adjust_src_path="/opt/sources/ => "
RUN lcov --remove coverage.info '/usr/*' --output-file coverage.info # filter system-files
RUN cp /opt/sources/build/coverage.info /opt/sources/coverage.info

WORKDIR /opt/sources
RUN mkdir -p out && chmod 777 out
RUN lcov --list /opt/sources/coverage.info
RUN genhtml /opt/sources/coverage.info --output-directory /opt/sources/out > genhtml.stdout 2> genhtml.stderr || cat genhtml.log;
RUN echo "GenHTML Exit Status: $?" ;
RUN cat genhtml.stdout
RUN cat genhtml.stderr
RUN ls -l;
RUN ls -la out

###################################################
# Section 2: Bundle the application.
FROM registry.git.chalmers.se/courses/dit638/students/2024-group-21/ubuntu:20.04
MAINTAINER Christian Berger christian.berger@gu.se

RUN apt-get update -y --allow-unauthenticated && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y

COPY --from=builder /opt/sources/coverage.info /opt/coverage.info
COPY --from=builder /tmp/helloworld /opt/helloworld

# Adds entrypoint script
COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

WORKDIR /opt
ENTRYPOINT ["/opt/entrypoint.sh"]