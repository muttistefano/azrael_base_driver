from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
import time

def onPositionChange(self, positionChange, timeChange, indexTriggered):
	vel = (positionChange / 4.0) / (timeChange * 0.001)
	print(vel/100)
	# print("PositionChange [" + str(self.getChannel()) + "]: " + str(positionChange))
	# print("TimeChange [" + str(self.getChannel()) + "]: " + str(timeChange))
	# print("IndexTriggered [" + str(self.getChannel()) + "]: " + str(indexTriggered))
	# print("----------")
	

def main():
	encoder0 = Encoder()
	# encoder1 = Encoder()

	encoder0.setChannel(0)
	# encoder1.setChannel(1)

	encoder0.setOnPositionChangeHandler(onPositionChange)
	# encoder1.setOnPositionChangeHandler(onPositionChange)

	encoder0.openWaitForAttachment(5000)
	# encoder1.openWaitForAttachment(5000)

	encoder0.setDataInterval(8)
	# encoder1.setDataInterval(8)

	# minDataInterval = encoder0.getMinDataInterval()
	# print("MinDataInterval: " + str(minDataInterval))

	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	encoder0.close()
	# encoder1.close()

main()