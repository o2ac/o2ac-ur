#!/bin/bash

################################################################################

RUN_ON_STARTUP="false"

case $1 in
    --run-immediately )     RUN_ON_STARTUP="true"
                            shift  # Moves remaining positional parameters down by one
esac

COMMAND=$1

if [ -z "${COMMAND}" ]; then
  echo "No command is specified."
  echo "Usage: $0 COMMAND"
  exit 1
fi

PROMPT_START=$'\e[4m\e[1m'
PROMPT_END=$'\e[0m'
KEY_START=$'\e[7m'
KEY_END=$'\e[0m'${PROMPT_START}
PROMPT="${PROMPT_START}Run '${COMMAND}'? Press:"$'\n'"'${KEY_START}r${KEY_END}' to run, "$'\n'"'${KEY_START}c${KEY_END}' to enter a child shell,"$'\n'"'${KEY_START}q${KEY_END}' to quit.${PROMPT_END}"$'\n'

if [ "${RUN_ON_STARTUP}" = "true" ]; then
  echo "Executing the command '${COMMAND}' once without waiting for confirmation"
  ${COMMAND};
  echo "" # Display an empty line.
fi

while true; do
  read -n 1 -s -p "${PROMPT}" input;
  if [ "${input}" = "r" ]; then
    echo "";
  elif [ "${input}" = "q" ]; then
    break;
  elif [ "${input}" = "c" ]; then
    cat <<EOF

Starting a new shell process.
You will return to the above prompt when you exit from this shell.

EOF
    bash -i
    continue;
  else
    continue;
  fi;
  echo "ROS_MASTER_URI: ${ROS_MASTER_URI}";
  ${COMMAND};
  echo "" # Display an empty line.
done

cat <<EOF

Starting a new shell process.

EOF

exec bash -i
