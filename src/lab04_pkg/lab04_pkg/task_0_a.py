import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from math import cos, sin, degrees
import matplotlib as mpl
from sympy import symbols, Matrix
import sympy

from lab04_pkg.models.velocity_motion_model import sample_normal_distribution, sample_velocity_motion_model

arrow = u'$\u2191$'

def compute_jacobians(x, u, dt):
    # Use fresh sympy symbols (do not mutate input lists)
    x_s, y_s, theta_s, v_s, w_s, dt_s = symbols('x y theta v w dt')
    beta = theta_s + w_s * dt_s
    R = v_s / w_s

    # Define the motion model symbolically
    gux = Matrix([
        [x_s - R * sympy.sin(theta_s) + R * sympy.sin(beta)],
        [y_s + R * sympy.cos(theta_s) - R * sympy.cos(beta)],
        [beta],
    ]) # matrix making the motion model

    # lambdify evaluators (return numeric callable functions)
    eval_gux = sympy.lambdify((x_s, y_s, theta_s, v_s, w_s, dt_s), gux, 'numpy') # transform symbolic to function
    
    Gt = gux.jacobian(Matrix([x_s, y_s, theta_s])) # Jacobian w.r.t state
    eval_Gt = sympy.lambdify((x_s, y_s, theta_s, v_s, w_s, dt_s), Gt, 'numpy') # transform function to values

    Vt = gux.jacobian(Matrix([v_s, w_s]))
    eval_Vt = sympy.lambdify((x_s, y_s, theta_s, v_s, w_s, dt_s), Vt, 'numpy')

    print("Jacobian w.r.t state Gt:")
    sympy.pprint(Gt)
    print("\nJacobian w.r.t control Vt:")
    sympy.pprint(Vt)

    print("\nEvaluated Gt at x={}, u={}, dt={}:".format(x, u, dt))
    print(eval_Gt(x[0], x[1], x[2], u[0], u[1], dt))

    print("\nEvaluated Vt at x={}, u={}, dt={}:".format(x, u, dt))
    print(eval_Vt(x[0], x[1], x[2], u[0], u[1], dt))

    return (eval_gux, eval_Gt, eval_Vt, Gt, Vt) 

def main():
    n_samples = 500
    dt = 0.5

    x = [2, 4, 0]
    u = [0.8, 0.6]

    #a = [0, 0, 0, 0, 0, 0] # no noise -> ideal

    #a = [0, 0, 0.2, 0.2, 0, 0] # noise hilghitngs for only rotational 
    #a = [0.2, 0.2, 0, 0, 0, 0] # noise hilghitngs for only translational 

    #a = [0.01, 0.01, 0.15, 0.2, 0.02, 0.05] # noise hilghitngs for rotational and low translational
    #a = [0.2, 0.15, 0.01, 0.01, 0.05, 0.02] # noise hilghitngs for translational and low rotational
    
    #a = [0.01, 0.01, 0.01, 0.01, 0.2, 0.2] # drift

    #a = [0.3, 0.3, 0.3, 0.3, 0.1, 0.1] # extreme high noise

   # a = [0.05, 0.15, 0.15, 0.05, 0.03, 0.03] # high coupled noise -> velocity affects rotation and vice versa

    a = [0.05, 0.05, 0.05, 0.05, 0.03, 0.03] # balanced noise -> more realistic



    """ noise parameters 'a'
    a[1]: noise related to translational velocity variance due to translational velocity
    a[2]: noise related to translational velocity variance due to rotational velocity 
    a[3]: noise related to rotational velocity variance due to translational velocity
    a[4]: noise related to rotational velocity variance due to rotational velocity
    a[5]: noise related to drift (pose) variance due to translational velocity
    a[6]: noise related to drift variance due to rotational velocity """

    x_prime = np.zeros([n_samples, 3]) # It gives back a n*3 matrix, 3 because we want x, y, theta
    for i in range(n_samples):
        x_prime[i,:] = sample_velocity_motion_model(x, u, a, dt)
    
    print(x_prime)

    compute_jacobians(x, u, dt)

    #Sampling the velocity model 
    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x[2])-90)
    plt.scatter(x[0], x[1], marker=rotated_marker, s=100, facecolors='none', edgecolors='b')

    for x_ in x_prime[:500]:
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x_[2])-90)
        plt.scatter(x_[0], x_[1], marker=rotated_marker, s=40, facecolors='none', edgecolors='r')

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("velocity motion model sampling")
    plt.show()

if __name__ == "__main__":
    main()


"""OUTPUT TO PUT ON REPORT:

[[2.38861594 4.05873356 0.3       ]
 [2.40331724 4.06095544 0.3       ]
 [2.43387297 4.06557349 0.3       ]
 ...
 [2.36011701 4.05442636 0.3       ]
 [2.33664203 4.05087847 0.3       ]
 [2.41955889 4.06341012 0.3       ]]

Jacobian w.r.t state Gt:
⎡        v⋅cos(θ)   v⋅cos(dt⋅w + θ)⎤
⎢1  0  - ──────── + ───────────────⎥
⎢           w              w       ⎥
⎢                                  ⎥
⎢        v⋅sin(θ)   v⋅sin(dt⋅w + θ)⎥
⎢0  1  - ──────── + ───────────────⎥
⎢           w              w       ⎥
⎢                                  ⎥
⎣0  0               1              ⎦

Jacobian w.r.t control Vt:
⎡  sin(θ)   sin(dt⋅w + θ)  dt⋅v⋅cos(dt⋅w + θ)   v⋅sin(θ)   v⋅sin(dt⋅w + θ)⎤
⎢- ────── + ─────────────  ────────────────── + ──────── - ───────────────⎥
⎢    w            w                w                2              2      ⎥
⎢                                                  w              w       ⎥
⎢                                                                         ⎥
⎢ cos(θ)   cos(dt⋅w + θ)   dt⋅v⋅sin(dt⋅w + θ)   v⋅cos(θ)   v⋅cos(dt⋅w + θ)⎥
⎢ ────── - ─────────────   ────────────────── - ──────── + ───────────────⎥
⎢   w            w                 w                2              2      ⎥
⎢                                                  w              w       ⎥
⎢                                                                         ⎥
⎣           0                                    dt                       ⎦

Evaluated Gt at x=[2, 4, 0], u=[0.8, 0.6], dt=0.5:
[[ 1.          0.         -0.05955135]
 [ 0.          1.          0.39402694]
 [ 0.          0.          1.        ]]

Evaluated Vt at x=[2, 4, 0], u=[0.8, 0.6], dt=0.5:
[[ 0.49253368 -0.01982058]
 [ 0.07443918  0.09776122]
 [ 0.          0.5       ]]

"""