#!/bin/bash

################################################################################

# Fix '.git' files of submodules so that they include only relative paths valid on both the container and the host.
# The command 'git submodule update --recursive' generates '.git' files with absolute paths for nested submodules.
# https://marc.info/?l=git&m=145932862024551&w=2

# Replaces full paths in '.git' files with the corresponding relative paths.
cd /root/o2ac-ur/
find . -type f -iname '.git' | \
  while read f ; do
    if grep -qE '^gitdir: /' $f ; then
      echo "Fix a full path in $f." ;
      RELATIVE=`echo ${f%/*/.git} | sed -ne 's/\([^/]*\)/../gp'`;
      sed -i -e 's@ \([^ ]*\)/\.git@ '${RELATIVE}'/.git@' $f;
    fi;
  done
cd /root/o2ac-ur/
