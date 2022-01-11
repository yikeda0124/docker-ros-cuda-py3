#!/bin/bash

################################################################################

PROMPT_START=$'\e[4m\e[1m'
PROMPT_END=$'\e[0m'
KEY_START=$'\e[7m'
CALIB_START=$'\e[5m'
KEY_END=$'\e[0m'${PROMPT_START}
KEY_CALIB=$'\e[1m'
PROMPT="${PROMPT_START}Run 'moveit'? Press:"$'\n'"'${KEY_START}r${KEY_END}' to run with the robot,"$'\n'"'${KEY_CALIB}s${KEY_END}' to run calibration,"$'\n'"'${KEY_START}s${KEY_END}' to run in the simulator,"$'\n'"'${KEY_START}c${KEY_END}' to enter a child shell,"$'\n'"'${KEY_START}q${KEY_END}' to quit.${PROMPT_END}"$'\n'

while true; do
  read -n 1 -s -p "${PROMPT}" input;
  if [ "${input}" = "r" ]; then
    export LAUNCH=weblab_real_default.launch
    sawyer_mode;
  elif [ "${input}" = "h" ]; then
    export LAUNCH=weblab_calibration.launch
    sawyer_mode;
  elif [ "${input}" = "s" ]; then
    export LAUNCH=weblab_gazebo_default.launch
    sim_mode;
  elif [ "${input}" = "q" ]; then
    break;
  elif [ "${input}" = "c" ]; then
    cat <<EOF
Starting a new shell process.
You will return to the above prompt when you exit from this shell.
Note: The new process does not inherit the mode ('sawyer_mode' or 'sim_mode') from the previously executed 'roslaunch' process.
EOF
    bash -i
    continue;
  else
    continue;
  fi;
done

cat <<EOF
Starting a new shell process.
Note: The new process does not inherit the mode ('sawyer_mode' or 'sim_mode') from the previously executed 'roslaunch' process.
EOF

exec bash -i