#!/bin/bash
for f in *.avi
do 
    echo "Processing $f file.."; 
    ffmpeg -i "${f}" -y -codec:v libx264 "mp4/${f%.avi}.mp4"
done