
########################################################################
########################################################################
#
#       Faustservice (remote Faust compiler) in a docker
#                 (L. Champenois & Y. Orlarey)
#
########################################################################
########################################################################

FROM grame/faustready-ubuntu-1604:004

########################################################################
# the environment below is used by the osx cross compiler
ENV UNATTENDED=1
ENV FORCERECOMPILE=002
ENV CLANG_VERSION=7.0.0
ENV MACOSX_DEPLOYMENT_TARGET=10.11
ENV PATH="${PATH}:/osxcross/bin:/osxcross/compiler/target/bin"
########################################################################

########################################################################
# Install OSX cross compilation (first part)
########################################################################
RUN		apt-get update
RUN 	mkdir osxcross
WORKDIR /osxcross
RUN 	git clone --depth 1 https://github.com/tpoechtrager/osxcross.git compiler
WORKDIR /osxcross/compiler
RUN 	sh tools/get_dependencies.sh
COPY   	faustcrossosx/MacOSX10.11.sdk.tar.xz tarballs/
RUN 	./build.sh


########################################################################
# Now we can clone and compile all the Faust related git repositories
########################################################################

RUN echo "CHANGE THIS NUMBER TO FORCE REGENERATION : 008"

RUN wget -q https://services.gradle.org/distributions/gradle-4.10.1-bin.zip \
    && unzip gradle-4.10.1-bin.zip -d /opt/gradle \
    && rm gradle-4.10.1-bin.zip

COPY faustservice /faustservice
RUN  make -C /faustservice

COPY faust /faust
RUN  make -C /faust; \
    make -C /faust install

# copy precompiled android libraries needed for OSC support (-osc option)
COPY libs /usr/local/share/faust/osclib/android/libs

########################################################################
# Install the ESP32 Toolchain
########################################################################

RUN apt-get update && apt-get install -y \
    apt-utils \
    bison \
    ca-certificates \
    ccache \
    check \
    cmake \
    curl \
    flex \
    git \
    gperf \
    lcov \
    libncurses-dev \
    libusb-1.0-0-dev \
    make \
    ninja-build \
    python3 \
    python3-pip \
    unzip \
    wget \
    xz-utils \
    zip \
   && apt-get autoremove -y \
   && rm -rf /var/lib/apt/lists/* \
   && update-alternatives --install /usr/bin/python python /usr/bin/python3 10

RUN python -m pip install --upgrade pip virtualenv

# To build the image for a branch or a tag of IDF, pass --build-arg IDF_CLONE_BRANCH_OR_TAG=name.
# To build the image with a specific commit ID of IDF, pass --build-arg IDF_CHECKOUT_REF=commit-id.
# It is possibe to combine both, e.g.:
#   IDF_CLONE_BRANCH_OR_TAG=release/vX.Y
#   IDF_CHECKOUT_REF=<some commit on release/vX.Y branch>.

ARG IDF_CLONE_URL=https://github.com/espressif/esp-idf.git
ARG IDF_CLONE_BRANCH_OR_TAG=master
ARG IDF_CHECKOUT_REF=

ENV IDF_PATH=/opt/esp/idf
ENV IDF_TOOLS_PATH=/opt/esp

RUN echo IDF_CHECKOUT_REF=$IDF_CHECKOUT_REF IDF_CLONE_BRANCH_OR_TAG=$IDF_CLONE_BRANCH_OR_TAG && \
    git clone --recursive \
      ${IDF_CLONE_BRANCH_OR_TAG:+-b $IDF_CLONE_BRANCH_OR_TAG} \
      $IDF_CLONE_URL $IDF_PATH && \
    if [ -n "$IDF_CHECKOUT_REF" ]; then \
      cd $IDF_PATH && \
      git checkout $IDF_CHECKOUT_REF && \
      git submodule update --init --recursive; \
    fi

RUN $IDF_PATH/install.sh && \
  rm -rf $IDF_TOOLS_PATH/dist


########################################################################
# Tune image by forcing Gradle upgrade
########################################################################
ENV GRADLE_USER_HOME=/tmp/gradle

RUN echo "process=+;" > tmp.dsp; \
    faust2android tmp.dsp; \
    faust2smartkeyb -android tmp.dsp; \
    rm tmp.apk

########################################################################
# Install OSX cross compilation (second part)
########################################################################

RUN echo "CHANGE THIS NUMBER TO FORCE CROSS OSX REGENERATION : 003"
WORKDIR /osxcross
COPY   	faustcrossosx/Qt5.9.1	/osxcross/Qt5.9.1 
COPY   	faustcrossosx/sdks 	/osxcross/sdks
COPY   	faustcrossosx/scripts	/osxcross/scripts
COPY   	faustcrossosx/tests 	/osxcross/tests
RUN		ln -s Qt5.9.1 Qt && \
    sh scripts/install.sh && \
    ln -s /usr/include/boost compiler/target/SDK/MacOSX10.11.sdk/usr/include/

# install RUST (temp here should be moved)
WORKDIR /faustservice
RUN curl https://sh.rustup.rs -sSf > rustup
RUN chmod a+x ./rustup
RUN ./rustup -y
RUN cp /root/.cargo/bin/* /usr/bin/
ENV USER=faust

########################################################################
# And starts Faustservice
########################################################################
EXPOSE 80
WORKDIR /faustservice
RUN cp ./bin/dockerOSX /usr/local/bin/; \ 
    rm -rf makefiles/osx; \
    rm -rf makefiles/dockerosx; \
    mv makefiles/crossosx makefiles/osx; \
    rm -rf makefiles/ros makefiles/unity/Makefile.all makefiles/unity/Makefile.android makefiles/unity/Makefile.ios makefiles/unity/Makefile.osx

CMD ./faustweb --port 80 --sessions-dir /tmp/sessions --recover-cmd /faustservice/faustweb



########################################################################
# HowTo run this docker image
########################################################################
# For local tests:
#-----------------
# docker run -it -p 80:80 eu.gcr.io/faust-cloud-208407/faustservicecloud:latest
#
# For production:
#-----------------
# docker run -d --restart=always -p 80:80 eu.gcr.io/faust-cloud-208407/faustservicecloud:latest
