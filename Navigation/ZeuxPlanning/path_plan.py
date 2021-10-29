import math
import numpy as np
import matplotlib.pyplot as plt
np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)

show_animation = True

def B_calc (x1, x2, y1, y2, a1, a2, R):
    B = np.array((
        [x2 - x1 + 2*R*np.sin((a2-a1)/2)*np.sin((a1+a2)/2)],
        [y2 - y1 - 2*R*np.sin((a2-a1)/2)*np.cos((a1+a2)/2)]
        ))
    return B

def A_calc(a1, a2):
    A = np.array(([
    [-np.sin(a1),   -np.sin(a2)],
    [np.cos(a1),    np.cos(a2)]
    ]))
    return A

def points_calc(x1, x2, y1, y2, a1, a2, R):
    A = A_calc(a1, a2)
    B = B_calc (x1, x2, y1, y2, a1, a2, R)

    #L12 = np.divide(A, B)
    L12 = (np.linalg.lstsq(A, B, rcond=None))[0]
    L1 = L12[0]
    L2 = L12[1]

    P1 = np.array(([x1], [y1])) + L1 * np.array(([-np.sin(a1)], [np.cos(a1)]))
    P2 = np.array(([x2], [y2])) - L2 * np.array(([-np.sin(a2)], [np.cos(a2)]))

    return P1, P2

def path_calc(P1, P2, x1, x2, y1, y2, a1, a2, R):
    #P1, P2 = points_calc(x1, x2, y1, y2, a1, a2, R)
    tau = (np.linspace(0, 1, 101) * (a2 - a1)).reshape(1, -1)

    points1 = np.array(([x1, P1[0]], [y1, P1[1]]))

    points2_a = (2*R*np.sin(tau/2))
    points2_b = np.concatenate((-np.sin(tau/2+a1), np.cos(tau/2+a1)))
    points2 = P1 + np.multiply(points2_a, points2_b)

    points3 = np.array(([P2[0], x2], [P2[1], y2]))

    path = np.concatenate((points1, points2, points3), axis=1)

    return path


def plot_arrow(x, y, yaw, length=0.2, width=0.2, fc="r",
               ec="k"):  # pragma: no cover
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw), fc=fc,
                  ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def main():

    #Initial and final positions and headings
    start_x = 0.0
    start_y = 0.0
    start_ang = math.radians(30)

    end_x = -4.0
    end_y = 3.0
    end_ang = math.radians(120)

    rad = 1 # Turning radius

    P1, P2 = points_calc(start_x, end_x, start_y, end_y, start_ang, end_ang, rad) 
    path = path_calc(P1, P2, start_x, end_x, start_y, end_y, start_ang, end_ang, rad)

    plt.plot(path[0], path[1])
    plot_arrow(start_x, start_y, start_ang)
    plot_arrow(end_x, end_y, end_ang)
    plt.plot(P1[0], P1[1], marker='o', markersize=3, color="red")
    plt.plot(P2[0], P2[1], marker='o', markersize=3, color="red")
    plt.show()
    #print(path.shape)

    

if __name__ == '__main__':
    main()
