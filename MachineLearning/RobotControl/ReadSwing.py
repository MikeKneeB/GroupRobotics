import SwingProxy

swingProxy=SwingProxy.SwingProxy("127.0.0.1",5005)

while True:
	pos=swingProxy.getUpdate()
	print pos
