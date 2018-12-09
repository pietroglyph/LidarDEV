#!/bin/bash

./gradlew jar
if [ "$?" != 0 ]; then
    exit
fi
java -jar ./build/LidarDEV.jar $@ # Pass --display to open the debug display