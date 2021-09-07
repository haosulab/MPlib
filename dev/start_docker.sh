#!/usr/bin/env bash

# DOCKER_IMAGE=quay.io/pypa/manylinux2014_x86_64
DOCKER_IMAGE=mplib-build:latest

docker run --name mplib-dev -v `pwd`:/workspace/MPlib -u user -i -d --rm ${DOCKER_IMAGE}
