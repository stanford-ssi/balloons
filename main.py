#SSI Balloon Main Loop
#Iskender Kushan, November 2014

import subprocess
import time
import os
os.chdir("./")

import processimage
import take_images
import subprocess
import random

#Define global constants
nPics = 10 #number of pictures taken during a transmission cycle

#Begin function

#Create new folder to store data in the current session

currentTime = (time.strftime("%x") + time.strftime("%X")).translate(None,"/:") #get current date and time
#random prefix added because the RPi doesnt have an RTC
randomprefix = str(random.randint(1,1000000))
newDirectory = "./data/" + currentTime + "_" + randomprefix + "_pics"
subprocess.call(["mkdir",newDirectory]) #make a new directory for picturess

#Begin main cycle
currentPicture = ""
counter = 0

while True:
	currentPicture = take_images.TakeImage(nPics,newDirectory,counter)
	print "Best picture picked for transmission: "
	print currentPicture	
	#transmit the best picture with SSTV
	if currentPicture != "NONE":
		#SSTV
		subprocess.call(["sh", "./pisstv_bin/sstvcam.sh", currentPicture])
	counter = counter + nPics
