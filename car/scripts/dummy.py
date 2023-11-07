from math import *

def angle_radians(x1,y1,x2,y2):
    d_th_x = x2 - x1
    d_th_y = y2 - y1
    angle = atan2(d_th_y,d_th_x)
    print(angle)

if __name__ == '__main__':
    angle_radians(0.0,0.0,10.0,10.0)