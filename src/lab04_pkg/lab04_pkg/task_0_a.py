import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from math import cos, sin, degrees
import matplotlib as mpl
from sympy import symbols, Matrix
import sympy

arrow = u'$\u2191$'

# The next function generates a single random number from a Gaussian distribution, mean is zero and variance is sigma_sqrd
def sample_normal_distribution(sigma_sqrd):  # The input is sigma_sqrd
    return 0.5 * np.sum(np.random.default_rng().uniform(-np.sqrt(sigma_sqrd), np.sqrt(sigma_sqrd), 12)) 


def sample_velocity_motion_model(x, u, a, dt):
    """ Sample velocity motion model.
    Arguments:
    x -- pose of the robot before moving [x, y, theta]
    u -- velocity reading obtained from the robot [v, w]
    a -- noise parameters of the motion model [a1, a2, a3, a4, a5, a6]
    dt -- time interval of prediction
    """
    v_hat = u[0] + np.random.normal(0, a[0]*u[0]**2 + a[1]*u[1]**2) # 
    w_hat = u[1] + np.random.normal(0, a[2]*u[0]**2 + a[3]*u[1]**2)
    gamma_hat = np.random.normal(0, a[4]*u[0]**2 + a[5]*u[1]**2)

    if abs(w_hat) < 1e-6:
        x_prime = x[0] + v_hat * dt * cos(x[2])
        y_prime = x[1] + v_hat * dt * sin(x[2])
        theta_prime = x[2] + gamma_hat * dt
    else:
        r = v_hat/w_hat
        x_prime = x[0] - r*sin(x[2]) + r*sin(x[2]+w_hat*dt)
        y_prime = x[1] + r*cos(x[2]) - r*cos(x[2]+w_hat*dt)
        theta_prime = x[2] + w_hat*dt + gamma_hat*dt

    return np.array([x_prime, y_prime, theta_prime])


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
    #a = [0.001, 0.01, 0.1, 0.2, 0.05, 0.05] # noise hilghitngs for rotational
    a = [0.12, 0, 0, 0, 0, 0] # noise hilghitngs for translational

    # a[1]: noise related to translational velocity variance due to translational velocity
    # a[2]: noise related to translational velocity variance due to rotational velocity 
    # a[3]: noise related to rotational velocity variance due to translational velocity
    # a[4]: noise related to rotational velocity variance due to rotational velocity
    # a[5]: noise related to drift (pose) variance due to translational velocity
    # a[6]: noise related to drift variance due to rotational velocity

    x_prime = np.zeros([n_samples, 3]) # It gives back a n*3 matrix, 3 because we want x, y, theta
    for i in range(n_samples):
        x_prime[i,:] = sample_velocity_motion_model(x, u, a, dt)
    
    print(x_prime)

    compute_jacobians(x, u, dt)


    ###################################
    ### Sampling the velocity model ###
    ###################################

    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x[2])-90)
    plt.scatter(x[0], x[1], marker=rotated_marker, s=100, facecolors='none', edgecolors='b')

    for x_ in x_prime[:200]:
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x_[2])-90)
        plt.scatter(x_[0], x_[1], marker=rotated_marker, s=40, facecolors='none', edgecolors='r')

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("velocity motion model sampling")
    plt.savefig("velocity_samples.pdf")
    plt.show()

    

if __name__ == "__main__":
    main()
