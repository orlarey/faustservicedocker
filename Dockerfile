
########################################################################
########################################################################
#
#       Faustservice (remote Faust compiler) in a docker
#                 (L. Champenois & Y. Orlarey)
#
########################################################################
########################################################################

FROM grame/faustready-ubuntu-1804:v4

RUN apt update 
RUN apt install -y python3.8 python3.8-venv

########################################################################
# Install the ESP32 Toolchain
########################################################################


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


COPY 	 esp32/faustBasic	/usr/local/share/faust/esp32/faustBasic
COPY 	 esp32/gramo-precomp	/usr/local/share/faust/esp32/gramo-precomp
COPY 	 esp32/gramo-bt-precomp	/usr/local/share/faust/esp32/gramo-bt-precomp

#RUN /bin/bash -c "source /opt/esp/idf/export.sh; make -C /usr/local/share/faust/esp32/faustBasic; make -C /usr/local/share/faust/esp32/gramo-precomp; make -C /usr/local/share/faust/esp32/gramo-bt-precomp"


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
# install RUST (temp here should be moved)
########################################################################

WORKDIR /rust
RUN curl https://sh.rustup.rs -sSf > rustup
RUN chmod a+x ./rustup
RUN ./rustup -y
RUN cp /root/.cargo/bin/* /usr/bin/
ENV USER=faust



########################################################################
# install SOUL
########################################################################

WORKDIR /soul
RUN wget https://github.com/soul-lang/SOUL/releases/download/0.9.66/binaries-linux-combined.zip
RUN unzip binaries-linux-combined.zip
RUN cp linux/x64/soul /usr/local/bin/
RUN cp linux/x64/libSOUL_PatchLoader.so /usr/lib/


########################################################################
# Now we can clone and compile all the Faust related git repositories
########################################################################

RUN echo "CHANGE THIS NUMBER TO FORCE REGENERATION : 003"

RUN git clone https://github.com/grame-cncm/faust.git /faust; 
WORKDIR /faust
RUN git fetch && git checkout a8b732e3372429fac329d00e57045914c3476791
RUN echo "Revert to old faus2puredata because Apple M1 target not available with our osxcross"
RUN git checkout 1834bd41888ee518946d8e0ed661aad9ac88843d tools/faust2appls/faust2puredata 
RUN make &&  make install

# copy precompiled android libraries needed for OSC support (-osc option)
#COPY libs /usr/local/share/faust/osclib/android/libs

########################################################################
# Tune image by forcing Gradle upgrade
########################################################################
#ENV GRADLE_USER_HOME=/tmp/gradle

RUN echo "process=+;" > tmp.dsp; 
RUN faust2android tmp.dsp; 
RUN faust2smartkeyb -android tmp.dsp; 
RUN rm tmp.apk

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

# # install RUST (temp here should be moved)
# WORKDIR /rust
# RUN curl https://sh.rustup.rs -sSf > rustup
# RUN chmod a+x ./rustup
# RUN ./rustup -y
# RUN cp /root/.cargo/bin/* /usr/bin/
# ENV USER=faust

########################################################################
# Reinstall and starts Faustservice (a commit from server branch)
########################################################################

WORKDIR /faustservice
RUN git clone https://github.com/grame-cncm/faustservice.git /faustservice;
RUN git fetch && git checkout b777b41e4764bcca51ce880f1bd2002b0cf724df; \
    make


EXPOSE 80
WORKDIR /faustservice
RUN rm -rf makefiles/osx; \
    rm -rf makefiles/dockerosx; \
    mv makefiles/crossosx makefiles/osx; \
    rm -rf makefiles/ros makefiles/unity/Makefile.all makefiles/unity/Makefile.android makefiles/unity/Makefile.ios makefiles/unity/Makefile.osx \
    rm -rf makefiles/esp32/*Flash


# Update SuperCollider includes to latest
RUN rm -rf /usr/include/SuperCollider/*
COPY SuperCollider/include/ /usr/include/SuperCollider/
RUN rm -rf /osxcross/sdks/supercollider/*
COPY SuperCollider/include/ /osxcross/sdks/supercollider/


#########################################
# We don't have codesign for OSX targets
#########################################

RUN echo '#!/bin/bash' > /usr/local/bin/codesign
RUN echo 'echo "WARNING IGNORE:" $@' >> /usr/local/bin/codesign
RUN chmod a+x /usr/local/bin/codesign

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
