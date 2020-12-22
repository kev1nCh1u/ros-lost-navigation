#!/bin/bash
# Program:
#       This program ros clean
# History:
# 2020/12/20	kevin	First release

echo -e "\n\033[32m
##############################
# ros clean  #
# by Kevin Chiu 2020         #
##############################
\033[0m\n"

rosclean check
yes | rosclean purge
rosclean check
# gnome-terminal --tab -t "rosclean" -- bash -ic "rosclean check;yes | rosclean purge;rosclean check;exec bash"

# exec bash