#Stanford High Altitude Balloon
#Picks best images to transmit
#Uses a machine learning algorithm trained on a set of pictures obtained in Summer 2014
#Chuanqi Shen, Iskender Kushan November 2014
import os, sys
import Image
import math

def scoreImage(name):
  jpgfile = Image.open(name)
  pic = jpgfile.load()
  stepSize = 20
  if os.stat(name).st_size < 10000:
  #uncomment the following line for RPi operation	
  #if os.stat(name).st_size < 2200000:
    return -1.0
  (x, y) = jpgfile.size
  sky, white, not_sky = 0, 0, 0
  for i in range(0,x,stepSize):
    for j in range(0,y,stepSize):
      (r,g,b) = pic[i,j]
      r = r / 256.0
      g = g / 256.0
      b = b / 256.0
      is_sky = 23.0470 - 341.0218 * r  - 96.6866 * g + 309.2604 * b
      is_white = -23.4207 + 11.3401 * r + 12.2482 * g + 8.847 * b
      if is_sky > 0:
        sky += 1
      elif is_white > 0:
        white += 1
      else:
        not_sky += 1
  not_sky = not_sky * 1.0
  white = white * 1.0
  sky = sky * 1.0
  tot = (not_sky + white + sky) / 100.0
  #print "%.3lf %.3lf %.3lf" % (not_sky / tot, sky / tot, white / tot)
  return not_sky / (sky + white)

def pickBestImage(names):
  best_score = 0.0
  best_name = "NONE"
  for name in names:
    score = scoreImage(name)
    if score > best_score:
      best_score = score
      best_name = name
  if best_score < 0.8:
    return "NONE"
  else:
    return best_name
