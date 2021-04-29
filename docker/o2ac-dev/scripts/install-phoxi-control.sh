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
    wget https://photoneo.com/files/installer/$1/$2.run.zip
    unzip $2.run
    sh ./$2.run
    rm $2.run.zip $2.run
}

install_photoneo PhoXi/1.2.14 PhotoneoPhoXiControlInstaller-1.2.14-Ubuntu18-STABLE
