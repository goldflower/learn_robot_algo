import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
import random
import imageio, os
from sympy import N, Symbol

delta = 0.1
min_xy_boundary = -5.0
max_xy_boundary = 5.0

nb_contours = 50
alpha = 0.002
converge_condition = 1e-3
max_iterations = 500

def create_mesh_data(f):
    xs = np.arange(min_xy_boundary, max_xy_boundary, delta)
    ys = np.arange(min_xy_boundary, max_xy_boundary, delta)
    X, Y = np.meshgrid(xs, ys)
    Z = []
    for _xs, _ys in zip(X, Y):
        contour = [f.subs({x:_x, y:_y}) for _x, _y in zip(_xs, _ys)]
        Z.append(contour)
    return X, Y, Z

def compute_distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def gradient_descent(f, coord, converge_condition=converge_condition, max_iterations=max_iterations):
    current_coord = (coord[0], coord[1])
    path = [current_coord]
    for i in range(max_iterations):
        dx, dy = f.diff(x), f.diff(y)
        alpha_dx = alpha*dx.subs({x:current_coord[0], y:current_coord[1]})
        alpha_dy = alpha*dy.subs({x:current_coord[0], y:current_coord[1]})
        if sum([abs(alpha_dx), abs(alpha_dy)]) < converge_condition: 
            print("converged!!")
            break
        current_coord = (current_coord[0] - alpha_dx,
                         current_coord[1] - alpha_dy,)
        path.append(current_coord)
    else:
        print('exeeds max iterations, may not converged')

    return path

def update(i):
    label = 'timestep {0}'.format(i)
    

if __name__ == '__main__':
    x = Symbol('x')
    y = Symbol('y')
    f = (x**2+y-11)**2+(x+y**2-7)**2 # HimmelblauFunction
    start_point = (random.uniform(min_xy_boundary, max_xy_boundary),
                   random.uniform(min_xy_boundary, max_xy_boundary))
    path = gradient_descent(f, start_point)
    X, Y, Z = create_mesh_data(f)
    
    # visualization
    plt.clf()
    contours = plt.contour(X, Y, Z, nb_contours)
    plt.plot(start_point[0], start_point[1], 'xr')
    
    xs = [point[0] for point in path]
    ys = [point[1] for point in path]
    for i in range(0, len(xs)-1):
        plt.plot(xs[i: i+2], ys[i: i+2], '-r')
        plt.pause(0.1)
        
    plt.plot(xs[-1], ys[-1], '-*y')
    plt.show()
    plt.close()