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
    ./$2.run
    rm $2.tar $2.run
    cd /opt
    ln -s PhotoneoPhoXiControl-1.2.14 PhotoneoPhoXiControl 
}

if [ `lsb_release -sc` == "bionic" ]; then
  apt-get install -y libpcre16-3
  install_pkg i/icu libicu55_55.1-7_amd64.deb
  install_pkg libp/libpng libpng12-0_1.2.54-1ubuntu1_amd64.deb
fi

install_photoneo PhoXi/1.2.14 PhotoneoPhoXiControlInstaller-1.2.14-Ubuntu16-STABLE
