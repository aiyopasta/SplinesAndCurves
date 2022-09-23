from tkinter import *
import numpy as np
import time
from quat import *

# Window size
window_w = 1720
window_h = 1080
np.set_printoptions(suppress=True)

# Tkinter Setup
root = Tk()
root.title("Simulator")
root.attributes("-topmost", True)
root.geometry(str(window_w) + "x" + str(window_h))  # window size hardcoded
w = Canvas(root, width=window_w, height=window_h)
w.pack()

# Coordinate Shift
# def A(x, y):
#     return np.array([x + window_w/2, -y + window_h/2])

# Key handling function
def vanilla_key_pressed(event):
    global rho, theta, phi, focus, w

    m = 30
    drho, dphi, dtheta, dfocus = 10, np.pi/m, np.pi/m, 10
    if event.char == 'a':
        theta -= dtheta

    if event.char == 'd':
        theta += dtheta

    if event.char == 'w':
        phi -= dphi

    if event.char == 's':
        phi += dphi

    if event.char == 'p':
        rho -= drho

    if event.char == 'o':
        rho += drho

    if event.char == 'k':
        focus -= dfocus

    if event.char == 'l':
        focus += dfocus

    if event.char == 'm':
        w.bind("<KeyPress>", speedy_key_pressed)


def speedy_key_pressed(event):
    global v_rho, v_theta, v_phi, focus, w

    max_clicks = 10
    m = 800
    d2rho, d2phi, d2theta, dfocus = 3, np.pi / m, np.pi / m, 10
    if event.char == 'a':
        v_theta = max(v_theta - d2theta, -d2theta*max_clicks)

    if event.char == 'd':
        v_theta = min(v_theta + d2theta, d2theta*max_clicks)

    if event.char == 'w':
        v_phi = max(v_phi - d2phi, -d2phi*max_clicks)

    if event.char == 's':
        v_phi = min(v_phi + d2phi, d2phi*max_clicks)

    if event.char == 'p':
        v_rho = max(v_rho - d2rho, -d2rho*max_clicks/2)

    if event.char == 'o':
        v_rho = min(v_rho + d2rho, d2rho*max_clicks/2)

    if event.char == 'k':
        focus -= dfocus

    if event.char == 'l':
        focus += dfocus

    # Change mode
    if event.char == 'm':
        v_rho, v_theta, v_phi = 0, 0, 0
        w.bind("<KeyPress>", vanilla_key_pressed)


# Key binding
w.bind("<KeyPress>", speedy_key_pressed)
w.bind("<1>", lambda event: w.focus_set())
w.pack()

# ye parameters (rho = distance from origin, phi = angle from world's +z-axis, theta = angle from world's +x-axis)
rho, theta, phi = 700., np.pi/4, np.pi/4  # These provide location of the eye.
v_rho, v_theta, v_phi = 0, 0, 0
focus = 500.  # Distance from eye to near clipping plane, i.e. the screen.
far_clip = rho * 3  # Distance from eye to far clipping plane
assert far_clip > focus

# # Old world_to_plane() method from Winter 2019-2020
# def old_world_to_plane(v):
#     # Radial distance to eye from world's origin.
#     eye_rho = rho + focus
#
#     # Vector math from geometric computation (worked out on white board, check iCloud for possible picture)
#     # It is just converting from spherical to Cartesian coordinates, then flipping it because it is "eye to origin".
#     eye_to_origin = -np.array([eye_rho * np.sin(phi) * np.cos(theta),
#                                eye_rho * np.sin(phi) * np.sin(theta), eye_rho * np.cos(phi)])
#
#     # Vector from eye to the point in question.
#     eye_to_v = eye_to_origin + v
#
#     # Vector from origin to the center of the screen in front of eye.
#     origin_to_P = np.array([rho * np.sin(phi) * np.cos(theta), rho * np.sin(phi) * np.sin(theta), rho * np.cos(phi)])
#
#     # Formula for t corresponding to point on line from eye to v that intersects screen: t = (n•(a-b)) / (n•v)
#     # n•(r_t - origin_to_P)=0  <=>  n•(v + (t*eye_to_v) - origin_to_P)=0  <=>  the result!
#     t = np.dot(eye_to_origin, origin_to_P - v) / np.dot(eye_to_origin, eye_to_v)
#     r_t = v + (t * eye_to_v)  # The actual point of intersection
#
#     # Location of image coords in terms of world coordinates.
#     # Vector from the center of the screen to the intersection point.
#     tile_center_world = -origin_to_P + r_t
#
#     # Spherical basis vectors (coincidentally phi_hat is downwards but +'ve direction in image plane is also downwards)
#     # Convert it into the screen local coordinates of x pointing to the right and y pointing down.
#     theta_hat = np.array([-np.sin(theta), np.cos(theta), 0])
#     phi_hat = -np.array([np.cos(phi) * np.cos(theta), np.cos(phi) * np.sin(theta), -np.sin(phi)])
#     tile_center = np.array([np.dot(tile_center_world, theta_hat), np.dot(tile_center_world, phi_hat)])
#
#     # Re-adjust to make center of screen (0,0) and y pointing up.
#     tile_center = A(*tile_center)
#
#     return tile_center


# Input: A point in 3D world space. Output: Corresponding point in range [-width/2, width/2]x[-height/2, height/2].
def world_to_plane(v):
    global rho, theta, phi, focus
    # -1. Adjust focus based on
    # 0. Turn vector into a homogeneous one.
    v = np.append(v, 1)
    # 1. Convert vector from world into camera space
    # a) Get camera basis vectors in terms of world coordinates (x right, y up, z out of page).
    xhat = np.array([-np.sin(theta), np.cos(theta), 0, 0])
    yhat = -np.array([np.cos(phi) * np.cos(theta), np.cos(phi) * np.sin(theta), -np.sin(phi), 0])
    zhat = np.array([np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi), 0])
    # b) Build the 4th column of the matrix
    cam_pos = np.append((rho * zhat)[:3], 1)
    # c) Construct the matrix and do the conversion
    world_to_cam = np.linalg.inv(np.array([xhat, yhat, zhat, cam_pos]).T)
    v_cam = np.dot(world_to_cam, v)
    # 2. Convert from camera space to screen space (using similar triangles math)
    cam_to_screen = np.array([[-focus, 0, 0, 0],
                              [0, -focus, 0, 0],
                              [0, 0, -far_clip/(-far_clip+focus), (-far_clip*focus)/(-far_clip+focus)],
                              [0, 0, 1, 0]])
    v_screen = np.dot(cam_to_screen, v_cam)
    v_screen /= v_screen[3]  # division by z
    return (v_screen[:2] * np.array([1, -1])) + np.array([window_w/2, window_h/2])


# 1000 IQ Genius Function
def list_world_to_plane(l):
    new_l = []
    for v in l:
        new_l.extend(world_to_plane(v).tolist())
    return new_l


def draw_cube(sidelen=400, vertex_radius=5):
    # Vertices in World Cartesian Coordinates
    l = sidelen / 2
    v_0 = np.array([l, -l, l])
    v_1 = np.array([l, l, l])
    v_2 = np.array([-l, l, l])
    v_3 = np.array([-l, -l, l])
    v_4 = np.array([l, -l, -l])
    v_5 = np.array([l, l, -l])
    v_6 = np.array([-l, l, -l])
    v_7 = np.array([-l, -l, -l])
    vertices = [v_0, v_1, v_2, v_3, v_4, v_5, v_6, v_7]
    # Vertex radius
    vr = vertex_radius
    # Edges (each e_i is the set of vertices the ith vertex is connected to (unmarked))
    e_0 = [v_1, v_3, v_4]
    e_1 = [v_2, v_5]
    e_2 = [v_3, v_6]
    e_3 = [v_7]
    e_4 = [v_7, v_5]
    e_5 = [v_6]
    e_6 = [v_7]
    edges_from = [e_0, e_1, e_2, e_3, e_4, e_5, e_6, []]
    # Draw cube
    for i, v in enumerate(vertices):
        v = world_to_plane(v)
        edges_from_i = [world_to_plane(v_j) for v_j in edges_from[i]]
        if show_cube:
            w.create_oval(v[0] - (vr / 2), v[1] - (vr / 2), v[0] + (vr / 2), v[1] + (vr / 2), outline='white',
                          fill='white', tag='v_' + str(i))

            for j, v_j in enumerate(edges_from_i):
                w.create_line(v[0], v[1], v_j[0], v_j[1], fill='orange', tag='edge_' + str(i) + str(j))


# TODO: Update name with actual type of sphere
def draw_uvsphere(center, radius, subdivs, color, tag, w):
    dtheta, dphi = 2 * np.pi / subdivs, 2 * np.pi / subdivs

    # Vertex radius
    v_npole = world_to_plane(center + np.array([0, 0, radius]))
    v_spole = world_to_plane(center - np.array([0, 0, radius]))
    middle_verts = [[v_npole]]
    for p in np.arange(dphi, np.pi, dphi):
        ring_verts = []
        for t in np.arange(0, 2 * np.pi, dtheta):
            v_x = radius * np.sin(p) * np.cos(t)
            v_y = radius * np.sin(p) * np.sin(t)
            v_z = radius * np.cos(p)
            ring_verts.append(world_to_plane(np.array([v_x, v_y, v_z]) + center))

        middle_verts.append(ring_verts)

    middle_verts.append([v_spole])

    # Draw Sphere
    for i, ring in enumerate(middle_verts):
        for j, v_j in enumerate(ring):
            w.create_line(*v_j, *ring[(j + 1) % len(ring)], fill=color, tag=tag+'e' + str(i) + str(j))
            # w.create_oval(v_j[0] - (vr / 2), v_j[1] - (vr / 2), v_j[0] + (vr / 2), v_j[1] + (vr / 2),
            #               outline='white', fill='white', tag=tag+'v'+str(i)+str(j))

            if i == 0:
                for k in range(len(middle_verts[i + 1])):
                    w.create_line(*v_j, *middle_verts[i + 1][k], fill=color, tag=tag+'f' + str(i) + str(k))

            if i != len(middle_verts) - 1 and i != 0:
                w.create_line(*v_j, *middle_verts[i + 1][j % len(middle_verts[i + 1])],
                              fill=color, tag=tag+'f' + str(i) + str(j))


# Construct list of orientations (in the form of Euler Angle triples (theta_z, theta_y, theta_x), i.e. zyx order)
# To be absolutely clear: This means rotation about z-axis by theta_z, followed by rot about LOCAL y-axis by theta_y,
# followed finally by rot about LOCAL x-axis by theta_x.
euler_list = [(0, np.pi/4, 0), (0, 0, np.pi/2)]
quat_list = [euler_to_quat(*angles) for angles in euler_list]


# The actual method that produces the reoriented vector for a given t in [0,1], using an quaternion spline.
def slerp_using_quats(v, t=0):
    global quat_list
    assert len(quat_list) == 2
    q = Slerp(quat_list[0], quat_list[1], t)
    return q.rotate_vector(v)


# The actual method that produces the reoriented vector for a given t in [0,1], using an Euler angle spline.
def lerp_using_eulers(v, t=0):
    global euler_list
    assert len(euler_list) == 2
    o1, o2 = np.asarray(euler_list[0]), np.asarray(euler_list[1])  # o stands for "orientation"
    R = euler_to_rotmat(*(((1 - t) * o1) + (t * o2)))
    return np.dot(R, v)


# TODO: Implement Euler + Quat SPLINES to support > 2 points.

# Vector to reorient using quaternions
vector = np.array([0, 0, 300])
# Main controlling parameter
t = 0.
# Path points
euler_path, quat_path = [], []

# Show / hide  3D objects
show_axes = True
show_cube = False
show_sphere = True
show_curr_vectors = True
show_paths = True


# Main function
def run():
    global rho, phi, theta, v_rho, v_theta, v_phi, vector, t, show_axes, show_cube, show_sphere, show_curr_vectors, show_paths, euler_path, quat_path
    w.configure(background='black')

    while True:
        w.delete('all')
        # Camera Velocity Update
        rho += v_rho
        phi += v_phi
        theta += v_theta

        # t Parameter update
        t += 0.005
        if t > 1:
            t = 0
            euler_path.clear()
            quat_path.clear()

        # 3D Axes Drawing –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
        colors = ['red', 'blue', 'purple']
        mag = 200
        direction_vectors = [np.array([mag, 0, 0]), np.array([0, mag, 0]), np.array([0, 0, mag])]
        if show_axes:
            for i, v_i in enumerate(direction_vectors):
                tile_center = world_to_plane(v_i)
                w.create_line(window_w/2, window_h/2, tile_center[0], tile_center[1], fill=colors[i])

        # Cube Drawing –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
        sidelen, vertex_radius = 400, 5
        if show_cube:
            draw_cube(sidelen, vertex_radius)

        # Sphere Drawing –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
        center, radius, subdivs = np.array([0, 0, 0]), 300, 20
        if show_sphere:
            draw_uvsphere(center, radius, subdivs, _from_rgb((30, 30, 0)), 'potty', w)

        # Plot Current Point –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
        radius = 5
        if show_curr_vectors:
            point = lerp_using_eulers(vector, t)
            euler_path.append(point)
            point = world_to_plane(point)
            w.create_oval(point[0]-radius, point[1]-radius, point[0]+radius, point[1]+radius, fill='red')
            point = slerp_using_quats(vector, t)
            quat_path.append(point)
            point = world_to_plane(point)
            w.create_oval(point[0] - radius, point[1] - radius, point[0] + radius, point[1] + radius, fill='blue')

        if show_paths and len(euler_path) >= 2:
            w.create_line(*list_world_to_plane(euler_path), fill='red')
            w.create_line(*list_world_to_plane(quat_path), fill='blue')


        # End run
        w.update()
        time.sleep(0.001)


# From https://stackoverflow.com/questions/51591456/can-i-use-rgb-in-tkinter
def _from_rgb(rgb):
    """translates an rgb tuple of int to a tkinter friendly color code
    """
    return "#%02x%02x%02x" % rgb


# Main function
if __name__ == '__main__':
    run()

# Necessary line for Tkinter
mainloop()
