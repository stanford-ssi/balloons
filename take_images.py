#SSI Balloon Video Recording + SSTV transmission script
#Once initialized, loops forever to take a picture, transmit it over SSTV, shoot a 5 minute video, and then transmit
#an SSTV picture again.
#Iskender Kushan, July 2014

import time
import processimage
import subprocess

#image taking period in seconds
picPeriod = 1

def TakeImage(nPics, pathDirectory,counter):
	periodCounter = counter
	names = []
	for i in range(nPics): #TODO: Change this later!
		name = pathDirectory + "/photo" + str(periodCounter) + ".jpg"
		names.append(name)
		subprocess.call(["fswebcam","-r","1280x720","--no-banner",name])
		#uncomment this line for RPi operation		
		#subprocess.call(["raspistill","-n","-vf","-ex","antishake","-o",name])
		periodCounter += 1
		time.sleep(picPeriod)
	pic_name = processimage.pickBestImage(names)
	return pic_name	    
