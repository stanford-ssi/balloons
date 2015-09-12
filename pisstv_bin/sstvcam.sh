#!/bin/bash
# sstv camera, Gerrit Polder, PA3BYA
# shell script to test SSTV camera
# Modifications : Iskender Kushan, KK6MIR November 2014

#convert the image to a lower resolution
convert $1 -resize 320x256 -size 320x256 xc:white +swap -gravity center -composite /tmp/image.jpg

#add callsign watermark
composite -gravity south -geometry -75+235 ./pisstv_bin/ntp.png /tmp/image.jpg /tmp/image.jpg
./pisstv_bin/pisstv /tmp/image.jpg 22050

#generate a tone to trigger the squelch
#UBUNTU PORT CHANGE: mplayer instead of omxplayer
mplayer ./pisstv_bin/squelch_trigger.wav
mplayer /tmp/image.jpg.wav
rm /tmp/image.jpg*
