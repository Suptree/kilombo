FROM ubuntu:20.04

ENV APP_DIR /workspace

USER root

RUN apt-get update && \
    apt-get install build-essential git gcc-avr gdb-avr binutils-avr avr-libc avrdude libsdl1.2-dev libjansson-dev libsubunit-dev cmake check
WORKDIR ${APP_DIR}