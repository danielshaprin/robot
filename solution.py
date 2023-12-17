#!/usr/bin/python3

from robonav_client import RoboNavClient
from time import sleep

sess = RoboNavClient(42) # 42 is our seed. Test with 1000, it is a special constructed case
st = sess.status()
st.gpsLat, st.gpsLon
print("Session started with key = %d, target = %.9f, %.9f" % (sess.getKey(), sess.targetLat, sess.targetLon))

sess.setSpeeds(3, 3, 3, 3)
time = 0.0
while True:
	sess.wait(0.5)
	sleep(0.5)
	time += 0.5
	st = sess.status()
	print("At time %5.1f: my coordinates (%.9f, %.9f), speed %.1f" % (time, st.gpsLat, st.gpsLon, st.spdFL), end="")
	print(", distanceToTarget = %.3f" % st.distanceToTarget)
	if st.distanceToTarget <= 1.7:
		break
sess.finish()

