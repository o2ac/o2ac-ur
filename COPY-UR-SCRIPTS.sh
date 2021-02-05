# Copies the programs from the wrs2020 folders of the UR robots to the repository
# Used for backing up the robot contents.
# Requires sshpass to be installed.

rm -rf catkin_ws/src/o2ac_routines/urscripts/a_bot
rm -rf catkin_ws/src/o2ac_routines/urscripts/b_bot

sshpass -p "easybot" scp -r root@192.168.1.41:/programs/wrs2020 catkin_ws/src/o2ac_routines/urscripts/a_bot
sshpass -p "easybot" scp -r root@192.168.1.42:/programs/wrs2020 catkin_ws/src/o2ac_routines/urscripts/b_bot

echo "Copied UR scripts to o2ac_routines/urscript from wrs2020 folder on UR pendants"
