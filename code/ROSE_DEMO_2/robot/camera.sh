#!/bin/bash
echo "Applying settings for optimal chilitag detection"
v4l2-ctl -d 1 -c auto_exposure=1
v4l2-ctl -d 1 -c exposure=60
v4l2-ctl -d 1 -c brightness=50
v4l2-ctl -d 1 -c contrast=32
v4l2-ctl -d 1 -c saturation=100
v4l2-ctl -d 1 -c hue=0
v4l2-ctl -d 1 -c sharpness=0
v4l2-ctl -d 1 -c gain_automatic=0
v4l2-ctl -d 1 -c gain=10

# High resolution
# Lowest exposure
# 1/6 to 1/5 of maximum gain
# 1/2 to 2/3 brightness
# Lowest contrast
# Highest color intensity
# Lowest white balance

# Defaults for ps3 eye:
# v4l2-ctl -d 1 -c brightness=0 				#(0-255)
# v4l2-ctl -d 1 -c contrast=32 					#(0-255)
# v4l2-ctl -d 1 -c saturation=64 				#(0-255)
# v4l2-ctl -d 1 -c hue=0 						#(-90 - 90)
# v4l2-ctl -d 1 -c white_balance_automatic=1 	#(0-1)
# v4l2-ctl -d 1 -c auto_exposure=0 				#(0-1)
# v4l2-ctl -d 1 -c exposure=120 				#(0-255)
# v4l2-ctl -d 1 -c gain_automatic=1 			#(0-1)
# v4l2-ctl -d 1 -c gain=20 						#(0-63)
# v4l2-ctl -d 1 -c horizontal_flip=0 			#(0-1)
# v4l2-ctl -d 1 -c vertical_flip=0 				#(0-1)
# v4l2-ctl -d 1 -c power_line_frequency=0 		#(0-1)
# v4l2-ctl -d 1 -c sharpness=0					#(0-63)