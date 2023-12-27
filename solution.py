#!/usr/bin/python3
import numpy as np

from robonav_client import RoboNavClient
from time import sleep

for i in range(1000):
	sess = RoboNavClient(i) # 42 is our seed. Test with 1000, it is a special constructed case
	st = sess.status()
	print("%d my coordinates (%.9f, %.9f)" % (i, st.gpsLat, st.gpsLon))
	sess.finish()

def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm != 0 else v

st = sess.status()
st.gpsLat, st.gpsLon
robot_pos = np.asarray((st.gpsLat, st.gpsLon))
dest_pos = np.asarray((sess.targetLat, sess.targetLon))
dest_dir = normalize(dest_pos - robot_pos)
north = np.asarray((1.0,0.0))


print("Session started with key = %d, target = %.9f, %.9f" % (sess.getKey(), sess.targetLat, sess.targetLon))

sess.setSpeeds(1, 0, 0, 0)
time = 0.
curr_dir = last_dir = st.hdg
avg_difs = []
i = 1
while True:
	sess.wait(1)
	sleep(1)
	time += 1
	st = sess.status()
	curr_dir = st.hdg
	dif = curr_dir - last_dir
	if dif < 0:
		dif += 360
	avg_difs.append(dif)
	if i % 10 == 0:
		avg_difs = avg_difs[1:] 
	print(sum(avg_difs)/len(avg_difs))
	last_dir = curr_dir
	#print("At time %5.1f: my coordinates (%.9f, %.9f), angle %.1f" % (time, st.gpsLat, st.gpsLon, st.hdg), end="")
	#print(", distanceToTarget = %.3f" % st.distanceToTarget)
	#print(sess.targetLat)
	if st.distanceToTarget <= 1.7:
		break
	i+=1
sess.finish()

