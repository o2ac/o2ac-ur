#!/bin/bash

# Launches Terminator with a convenience layout.
# To run with another layout, add an argument, e.g.:
#   ./LAUNCH-TERMINATOR-TERMINAL.sh rt

# Make sure that Terminator is not already running, so the config file can be read in correctly.

################################################################################

# Make a backup of the user's original Terminator configuration.
mkdir -p ~/.config/terminator/
if [ ! -f ~/.config/terminator/config.backup ]; then
  cp ~/.config/terminator/config ~/.config/terminator/config.backup
fi

# Update the user's current Terminator configuration with the project's one.
cp ./terminator/config ~/.config/terminator/config

################################################################################

# Run Terminator with the project's custom layout.
case "$1" in
  ( "" )
  terminator -m -l o2ac-autostart &  # The standard layout for a single machine
  ;;
  ( "rt" )
  terminator -m -l o2ac-rt-pc &  # The layout for the realtime kernel machine (no roscore, no vision...)
  ;;
  ( "vision" )
  terminator -m -l o2ac-vision-autostart &  # The layout for the vision PC
esac
