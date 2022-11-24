FROM ubuntu:18.04

LABEL maintainer="2942826077@qq.com"

# use root for dependency installation:
USER root

# set up environment:
## -- DEBIAN_FRONTEND告知操作系统应该从哪儿获得用户输入
## -- 如果设置为”noninteractive”，你就可以直接运行命令，而无需向用户请求输入
ENV DEBIAN_FRONTEND noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV HOME=/root SHELL=/bin/bash

# Ubuntu: 换中文的源
COPY ${PWD}/docker/image/etc/apt/sources.list /etc/apt/sources.list

# install:
RUN apt-get update -q --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
    ## -- add-apt-repository是由software-properties-common这个工具包提供的用来添加源的一个工具
    ## -- 所以要先安装software-properties-common才能使用add-apt-repository，否则会显示“command not found”
    software-properties-common \
    ## -- dirmngr证书列表管理工具，和gnupg一起工作
    dirmngr gnupg2 apt-utils && \
    ## -- gcc-9的源
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys 1EE2FF37CA8DA16B && \
    add-apt-repository ppa:ubuntu-toolchain-r/test


## -- libsparse: bugs修复，在ceres安装为共享库时产生
RUN add-apt-repository -r ppa:bzindovic/suitesparse-bugfix-1319687

RUN apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        # package utils:
        sudo dpkg pkg-config apt-utils \
        # security:
        openssh-server pwgen ca-certificates \
        # network utils:
        curl wget iputils-ping net-tools \
        # command line:
        vim grep sed patch \
        # io:
        pv zip unzip bzip2 \
        # version control:
        git mercurial subversion \
        # daemon & services:
        supervisor nginx \
        # dev. tools:
        terminator \
        # potential image & rich text IO:
        libsdl1.2-dev libsdl-net1.2-dev libsdl-image1.2-dev \
        lxde \
        gnome-themes-standard \
        xvfb dbus-x11 x11-utils libxext6 libsm6 x11vnc \
        gtk2-engines-pixbuf gtk2-engines-murrine pinta ttf-ubuntu-font-family \
        mesa-utils libgl1-mesa-dri libxrender1 \
        gnuplot \
        # c++:
        gcc-7 g++-7 gcc-8 g++-8 gcc-9 g++-9 \
        make cmake build-essential autoconf automake libtool \
        libglib2.0-dev libboost-dev libboost-all-dev \
        libomp-dev libtbb-dev \
        libgoogle-glog-dev \
        # lua:
        lua5.3 liblua5.3-dev libluabind-dev \
        # numerical optimization:
        libeigen3-dev \
        # 3D graphics:
        freeglut3-dev \
        libqt4-dev libqt4-opengl-dev \
        qt5-default qt5-qmake \
        qtdeclarative5-dev libqglviewer-dev-qt5 libglew-dev glew-utils \
        # base library
        libglm-dev libglfw3-dev apt-transport-https dbus \
        # slam library
        libopencv-dev libpcl-dev libceres-dev libyaml-cpp-dev proj-bin&& \
        apt-get autoclean && \
        apt-get autoremove && rm -rf /var/lib/apt/lists/*

# 默认gcc-9环境
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100


COPY ${PWD}/docker/installers /tmp/installers
WORKDIR /tmp/installers

# install cmake
## -- case1：离线安装
RUN tar -zxvf cmake-3.15.0-Linux-x86_64.tar.gz && \
## -- case2：在线安装
# RUN wget https://cmake.org/files/v3.15/cmake-3.15.0-Linux-x86_64.tar.gz && tar -zxvf cmake-3.15.0-Linux-x86_64.tar.gz && \
    mv cmake-3.15.0-Linux-x86_64/ /opt/cmake-3.15.0 && \
    ln -sf /opt/cmake-3.15.0/bin/*  /usr/bin/ && \
    rm -rf cmake-3.15.0-Linux-x86_64.tar.gz

# install g2o -- https://github.com/RainerKuemmerle/g2o:
## -- case1：离线安装
RUN tar -zxvf g2o-20201223_git.tar.gz && cd g2o-20201223_git && \
## -- case2：在线安装
# RUN git clone https://github.com/RainerKuemmerle/g2o.git -o g2o && cd g2o && git checkout 1b89597b4e495bef5447c5b8001f5f93ccf32ec4 && \
    mkdir build && cd build && \
    cmake .. && make -j8 && make install && rm -rf g2o-20201223_git.tar.gz

# install tini:
## -- case1：离线安装
RUN dpkg -i tini.deb && apt-get clean
## -- case2：在线安装
# RUN curl -L "https://github.com/krallin/tini/releases/download/v0.19.0/tini_0.19.0.deb" > ./tini.deb && \
    # dpkg -i tini.deb && apt-get clean

COPY ${PWD}/docker/image/etc/supervisor /etc/supervisor
COPY ${PWD}/docker/image/startup.sh /startup.sh


RUN rm -rf /tmp/installers


# 暴露监控接口
EXPOSE 9001

WORKDIR /workspace


# ------------------ DONE -----------------------

ENTRYPOINT ["/startup.sh"]
