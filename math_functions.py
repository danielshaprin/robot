import matplotlib.pyplot as plt
import numpy as np

def line_from_points(p1,p2):
    m = (p2[1] - p1[1])/(p2[0] - p1[0])
    c = p1[1] - m * p1[0]
    return (m,c)

def get_mid_point(p1,p2):
    return (np.array(p1)+np.array(p2))/2

def perpendicular_line_through_mid_point(p1, p2):
    mid_point = get_mid_point(p1,p2)
    m,_ = line_from_points(p1,p2)
    m = -1/m
    c = mid_point[1] - m*mid_point[0]
    return (m,c)

def find_intersection_between_two_lines(line1, line2):
    m1,c1 = line1
    m2,c2 = line2

    x = (c2-c1)/(m1-m2)
    y = m1*x + c1
    return x,y

def get_turn(back_left_wheel,back_righ_wheel,center,target):
    m,c = line_from_points(back_left_wheel,back_righ_wheel)
    target_line = perpendicular_line_through_mid_point(center,target)
    turn_center = find_intersection_between_two_lines(target_line, (m,c))
    turn_radius = np.linalg.norm(np.array(turn_center)-np.array(target))
    return turn_center, turn_radius

if __name__ == '__main__':
    W1 = [2,3]
    W2 = [4,5]

    T = [10,12]
    

    m,c = line_from_points(W1,W2)
    Pm, Pc = perpendicular_line_through_mid_point(W1,W2)
    C = [1]
    C.append(Pm*C[0] + Pc)
    target_line = perpendicular_line_through_mid_point(C,T)
    turn_center = find_intersection_between_two_lines(target_line, (m,c))
    turn_radius = np.linalg.norm(np.array(turn_center)-np.array(T))

    plt.scatter((W1[0], W2[0]),(W1[1], W2[1]))
    plt.scatter((C[0],),(C[1])) 
    plt.scatter((T[0],),(T[1]))
    X = [1,10]
    #fig = plt.figure()
    #ax = fig.add_subplot()
    plt.plot(X,[x*m + c for x in X])
    plt.plot(X,[x*target_line[0] + target_line[1] for x in X])
    plt.gca().add_patch(plt.Circle(turn_center,turn_radius, edgecolor='red', facecolor='none'))
    plt.show()

