import SwingProxy

#	Author:	Harry Withers
#   Date:   23/02/2017
#
#   This is a simple test of reading the position of the virtual swing.

swingProxy=SwingProxy.SwingProxy("127.0.0.1",5005)

while True:
	pos=swingProxy.get_angle()
	print pos
