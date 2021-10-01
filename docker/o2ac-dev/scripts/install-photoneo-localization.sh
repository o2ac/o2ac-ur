#!/bin/bash

install_pkg()
{
    cd /tmp
    wget http://mirrors.kernel.org/ubuntu/pool/main/$1/$2
    dpkg -i $2
    rm $2
}

install_photoneo()
{
    cd /tmp
    wget https://photoneo.com/files/installer/$1/$2.tar
    tar xvf $2.tar
    # bash ./$2
    expect /root/o2ac-ur/docker/o2ac-dev/scripts/install-photoneo-localization.exp $2
    rm $2.tar $2
}

if [ `lsb_release -sc` == "bionic" ]; then
    install_pkg libp/libpng libpng12-0_1.2.54-1ubuntu1_amd64.deb
    install_pkg libr/libraw libraw15_0.17.1-1_amd64.deb
    install_pkg j/jasper libjasper1_1.900.1-14ubuntu3.5_amd64.deb
    install_pkg libw/libwebp libwebp5_0.4.4-1_amd64.deb
    install_pkg libw/libwebp libwebpmux1_0.4.4-1_amd64.deb

    cd /usr/lib/x86_64-linux-gnu
    ln -s libopencv_core.so libopencv_core.so.3.1
    ln -s libopencv_imgproc.so libopencv_imgproc.so.3.1
fi

# apt-get install -y --no-install-recommends libssh2-1

install_photoneo Localization/1.3.1 PhotoneoLocalizationSDKInstaller-1.3.1+ed614d8-Linux-gcc5.5.0.run
