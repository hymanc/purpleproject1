	# A class for UVC camera interfacing via uvcdynctrl in Linux
	# Must have uvcdynctrl installed and on the system path

import os
import sys

class UVCInterface(object):

	BRIGHTNESS = 'Brightness'
	CONTRAST = 'Contrast'
	SATURATION = 'Saturation'
	WB_TEMP_AUTO = 'White\ Balance\ Temperature,\ Auto'
	GAIN = 'Gain'
	POWER_LINE_FREQUENCY = 'Power\ Line\ Frequency'
	SHARPNESS = 'Sharpness'
	BACKLIGHT_COMP = 'Backlight\ Compensation'
	FOCUS = 'Focus'
	FOCUS_ABS = 'Focus\ \(absolute\)'
	EXPOSURE_AUTO = 'Exposure\ Auto'
	EXPOSURE_AUTO_PRIORITY = 'Exposure,\ Auto\ Priority'
	EXPOSURE_ABS = 'Exposure\ \(absolute\)'

	# Sets a UVC parameter
	# @param device Video device to write to
	# @param param UVC parameter to write
	# @param value Value of UVC parameter
	@staticmethod
	def set(device, param, value):
		print 'Setting UVC parameter ' , str(param) , ' to ' , str(value)
		os.system(str('uvcdynctrl --device=video' + str(device) + ' --set=' + str(param) + ' ' + str(value)))

	# Gets a UVC parameter
	# @param device Video device to read from
	# @param param UVC parameter to read
	@staticmethod
	def get(device, param):
		print 'Getting UVC parameter ' , str(param)
		a = os.system(str('uvcdynctrl --device=video' + str(device) + ' --get=' + str(param)))
		print 'UVC parameter: ' , str(a)