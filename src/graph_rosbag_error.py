import rosbag
import matplotlib.pyplot as plt
import math

BAG_FILE = '4working.bag'
bag = rosbag.Bag(BAG_FILE)
all_topics = ['traj_error']

dist = []
times = []
new_times = []

for topic, msg, time in bag.read_messages(topics=all_topics):
    dist.append(msg.data[0])
    times.append(time.to_sec())

for i in range(len(times)):
    new_times.append(times[i] - times[0])

print(sum(dist)/len(dist))

plt.plot(new_times, dist, 'b')
plt.axis([0,40,0,1])
plt.title("Distance of Car from Trajectory - Lookahead = 1.2m, Velocity = 2m/s")
plt.xlabel("Time")
plt.ylabel("Error Magnitude")
plt.legend()
plt.show()

bag.close()
