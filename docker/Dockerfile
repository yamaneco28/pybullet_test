FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

# install utility
RUN apt-get -o Acquire::Check-Valid-Until=false -o Acquire::Check-Date=false update && \
    apt-get install -y \
    wget \
    curl \
    git \
    vim \
    python3-pip \
    ffmpeg

# install python package
WORKDIR /root/requirements
ADD requirements.txt /root/requirements
RUN pip3 install -r requirements.txt

WORKDIR /root/workspace
