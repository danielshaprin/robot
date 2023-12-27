import numpy as np
import math
import matplotlib.pyplot as plt

from robonav_client import RoboNavClient
from time import sleep
from matplotlib.patches import Circle
from math_functions import get_turn, gnss_to_meters

STEP = 0.5
WHEEL_BASE = 0.8
TRACK = 1

def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm != 0 else v

def clockwise_angle_between_vectors(vector_a, vector_b):
    # Calculate angles using arctan2
    angle_a = np.arctan2(vector_a[1], vector_a[0])
    angle_b = np.arctan2(vector_b[1], vector_b[0])

    # Calculate the clockwise angle
    clockwise_angle = angle_b - angle_a

    # Adjust the angle to be in the range [0, 2*pi)
    clockwise_angle = (clockwise_angle + 2 * np.pi) % (2 * np.pi)

    return np.degrees(clockwise_angle)

def directional_angle_between_vectors(vector_a, vector_b):
    # Calculate angles using arctan2
    angle_a = np.arctan2(vector_a[1], vector_a[0])
    angle_b = np.arctan2(vector_b[1], vector_b[0])

    # Calculate the directional angle
    directional_angle = angle_b - angle_a
    return directional_angle

def distance_by_interpolated_speed(time, speed1, speed2):
    if abs(speed2) >=4.39:
        diff_speed = speed2 - speed1
        time_to_max = diff_speed / 0.5
        avg_speed = (time_to_max / time)*(speed1 + speed2)/2 + (1 - time_to_max / time)*speed2
    else:
        avg_speed = (speed1 + speed2)

    return avg_speed * time

def pairwise(iterable):
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)

class World:
    def __init__(self, sess: RoboNavClient):
        self.sess = sess
        self.status = sess.status()
        self.north = np.asarray((1.0, 0.0))
        self.time = 0
        self.roobot_positions = [self.get_robot_pos()]
        self.speeds = [0]
        self.update()

    def update(self, step = STEP):
        self.prev_status = self.sess.status()
        self.time += step
        self.sess.wait(step)
        sleep(step)
        self.status = self.sess.status()
        self.roobot_positions += [self.get_robot_pos()]
        self.speeds.append(self.status.spdFR)

    def report(self):
        st = self.status
        print(f"At time {self.time:5.1f}")#: my coordinates ({st.gpsLat:.9f}, {st.gpsLon:.9f})")
        # print("Speeds %.9f, %.9f, %.9f, %.9f" % (st.spdFL, st.spdFR, st.spdBL, st.spdBR))
        # print("steering angles (%.9f, %.9f)" % (math.degrees(st.steerFL), math.degrees(st.steerFR)), end="")
        # print(f"pointing to {self.status.hdg}", end="")
        print(", distanceToTarget = %.3f, angleToTarget = %.3f" % (st.distanceToTarget, self.get_dir_angle_to_target()))
        #print(f'Angle to target: {self.get_dir_angle_to_target()}')
        #print(self.status.spdFL, self.status.spdFR, self.status.spdBL, self.status.spdBR)
        #print('hgh', self.status.hdg)

    def get_robot_pos(self):
        pos =  (self.status.gpsLat - self.sess.targetLat, self.status.gpsLon - self.sess.targetLon)
        return gnss_to_meters(self.status.gpsLat, pos[0], pos[1])

    def get_dest_pos(self):
        #return np.asarray((self.sess.targetLat, self.sess.targetLon))
        return np.zeros((2))
                           
    def get_dest_dir(self):
        return normalize(self.get_dest_pos() - self.get_robot_pos())

    def get_dest_angle(self):
        return clockwise_angle_between_vectors(self.get_dest_dir(), self.north)

    def get_angle(self):
        return self.status.hdg

    def get_robot_dir(self):
        angle_radians = np.radians(self.status.hdg)

        unit_vector = np.array((np.cos(angle_radians), np.sin(angle_radians)))

        return unit_vector

    def get_robot_dir_from_travel(self):

        return normalize(self.get_robot_pos() - np.asarray((self.prev_status.gpsLat, self.prev_status.gpsLon)))

    def get_dir_angle_to_target(self):
        return directional_angle_between_vectors(self.get_robot_dir(), self.get_dest_dir())

    def get_dir_angle_to_target_from_travel(self):
        return directional_angle_between_vectors(self.get_robot_dir_from_travel(), self.get_dest_dir())

    def plot_turn(self, back_left_wheel, back_right_wheel, front_left_wheel, front_right_wheel, robot_pos, robot_dir, target, center, radius):
        fig, ax = plt.subplots()
        ax.scatter([back_left_wheel[0], back_right_wheel[0], front_left_wheel[0], front_right_wheel[0], (robot_pos + robot_dir)[0], target[0], center[0]],
        [back_left_wheel[1], back_right_wheel[1], front_left_wheel[1], front_right_wheel[1], (robot_pos + robot_dir)[1], target[1], center[1]] )
        plt.gca().add_patch(plt.Circle(center,radius, edgecolor='red', facecolor='none'))
        ax.annotate('back_left_wheel', (back_left_wheel[0], back_left_wheel[1]))
        ax.annotate('back_right_wheel', (back_right_wheel[0], back_right_wheel[1]))
        ax.annotate('front_left_wheel', (front_left_wheel[0], front_left_wheel[1]))
        ax.annotate('front_right_wheel', (front_right_wheel[0], front_right_wheel[1]))
        ax.annotate('dir', ((robot_pos + robot_dir)[0], (robot_pos + robot_dir)[1]))
        plt.show()

    def turn_to_point(self, target):
        if self.status.distanceToTarget < 5:
            return

        robot_dir = self.get_robot_dir()
        robot_pos = self.get_robot_pos()
        back_wheels_mid_point = robot_pos - robot_dir * TRACK/2
        front_wheels_mid_point = robot_pos + robot_dir * TRACK/2 
        back_left_wheel = back_wheels_mid_point + np.array((-robot_dir[1], robot_dir[0])) * (WHEEL_BASE / 2)
        back_right_wheel = back_wheels_mid_point + np.array((robot_dir[1], -robot_dir[0])) * (WHEEL_BASE / 2)
        front_left_wheel = front_wheels_mid_point + np.array((-robot_dir[1], robot_dir[0])) * (WHEEL_BASE / 2)
        front_right_wheel = front_wheels_mid_point + np.array((robot_dir[1], -robot_dir[0])) * (WHEEL_BASE / 2)

        center, radius = get_turn(back_left_wheel, back_right_wheel, front_wheels_mid_point, target)

        left_angle = np.dot(robot_dir, normalize(center - front_left_wheel))
        left_angle = math.degrees(left_angle)
        right_angle = np.dot(robot_dir, normalize(center - front_right_wheel ))
        right_angle = math.degrees(right_angle)

        dir_of_center = np.cross((center-robot_pos), robot_dir)
        # print(f'left_angle: {left_angle:2.2f} right_angle:{right_angle:2.2f}')
        if dir_of_center<0:
            left_angle *= -1
            right_angle *= -1


        # self.plot_turn(back_left_wheel, back_right_wheel, front_left_wheel, front_right_wheel, robot_pos, robot_dir, target, center, radius)
        
        self.sess.setSteering(left_angle, right_angle)

def plot(world):
    x,y = np.transpose(world.roobot_positions)
    plt.scatter(x, y, label='Data Points')
    
    # Draw confidence circles around each point
    confidence_radius = 0.30
    for xi, yi in zip(x, y):
       circle = Circle((xi, yi), confidence_radius, edgecolor='red', facecolor='none', linestyle='dotted')
       plt.gca().add_patch(circle)
    
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Scatter Plot with Confidence Circles')
    
    plt.scatter([world.get_dest_pos()[0]], [world.get_dest_pos()[1]], color='red')
    plt.legend()
    
    plt.show()


def run(seed):
    sess = RoboNavClient(seed)
    print("%d Session started with key = %d, target = %.9f, %.9f" % (seed, sess.getKey(), sess.targetLat, sess.targetLon))
    world = World(sess)
    angle_to_target = initial_angle_to_target = world.get_dir_angle_to_target()
    #print('Angle to target: ', angle_to_target)
    speed = 0.5
    speeds = (speed, -speed, speed, -speed) if angle_to_target > 0 else (-speed, speed, -speed, speed)
    rotation_travel = 0.4*0.4*abs(angle_to_target)
    
    rotation_traveled = 0
    radius = (WHEEL_BASE**2 + TRACK**2)**0.5
    time_to_rotate = abs(initial_angle_to_target)*radius
    sess.setSpeeds(*speeds)
    world.update(time_to_rotate/2)
    world.report()
    angle_to_target = world.get_dir_angle_to_target()
    sess.setSpeeds(0,0,0,0)
    world.update(time_to_rotate/2)
    world.report()

    sess.setSpeeds(4.39,4.39,4.39,4.39)
    i = 0
    min_dist = 9999
    errors = 0 
    while True:
        world.update()
        world.get_dir_angle_to_target_from_travel()
        world.turn_to_point(world.get_dest_pos())
        world.report()
        if world.status.distanceToTarget <= 1.7:
            sess.finish()
            return True
        if world.status.distanceToTarget > min_dist:
            if errors >= 3:
                sess.finish()
                return False
            else:
                errors += 1
        else:
            errors = 0
    


if __name__ == '__main__':
    print("Success!" if run(42) else "Fail!")

