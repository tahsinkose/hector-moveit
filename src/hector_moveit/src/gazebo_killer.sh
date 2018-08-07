#!/bin/bash

kill_gazebo(){
    kill -9 $(pidof gzserver | awk '{print $1}')
    kill -9 $(pidof gzclient | awk '{print $1}')
    sleep 5
}

PAST_TIME="$(ps -o etime= -p $(pidof gzserver))"
IFS=':' read -ra ADDR <<< "$PAST_TIME"
Length=${#ADDR[@]}
if [ $Length -gt 2 ] # Process is open for more than 1 hour.
then
     kill_gazebo
else
    if [ ${ADDR[0]} -gt 0 ] # Process is open for at least 1 minute
    then
        kill_gazebo
    elif [ ${ADDR[1]} -gt 30 ] # Process is open for at least 30 seconds.
    then
        kill_gazebo
    fi
fi