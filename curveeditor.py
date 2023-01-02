from tkinter import *
import math
import numpy as np

# Window size
window_w = 1720
window_h = 1000

# Tkinter Setup
root = Tk()
root.title("Curve Fun")
root.attributes("-topmost", True)
root.geometry(str(window_w) + "x" + str(window_h))  # window size hardcoded

w = Canvas(root, width=window_w, height=window_h)
w.configure(background='white')
w.focus_set()
w.pack()

# List of points. A point is of the form: np.array([x, y])
points = []

# Interpolation / Approximation scheme (-1 = none)
scheme = 0

# Scheme Names
scheme_names = {0: 'NONE', 1: 'LAGRANGE', 2: 'CUBIC BÉZIER / CUBIC HERMITE', 3: 'SINGLE BÉZIER', 4: 'B-SPLINE APPROX'}

# GUI Params
point_radius = 5
click_detection_radius = point_radius * 2
point_fill = 'yellow'
point_outline = 'black'
hover_fill = 'red'
hover_outline = 'red'

# Which point is the user currently dragging? (-1 if none)
point_dragged_index = -1

# Which point is the user currently hovering over? (-1 if none)
point_hovered_index = -1


# SCHEME IMPLEMENTATION —————————————————————————————————————
def get_curve_points(scheme):
    '''
        Takes in an interpolation scheme, and returns a set of sampled points from resulting curve's function.
        0 = Lagrange
    '''
    assert scheme != 0

    # Lagrange Interpolation
    if scheme == 1:
        return lagrange()

    if scheme == 2:
        return cardinal()

    if scheme == 3:
        return bézier()

    if scheme == 4:
        return bspline()


def lagrange():
    global points

    # Figure out where to put the t_i which make the Lagrange polynomials spike.
    # We do this by calculating the total length of a linear connection between all the points
    # and seeing how much each segment contributes to the total length. Note that the curve
    # computed using this method is the exact same as if we were to invert the Vandermonde
    # matrix. "The polynomial stays the same, the basis functions are different."
    ti_list = []

    total_dist = 0
    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i + 1]
        dist = np.linalg.norm(p2 - p1)
        total_dist += dist
        ti_list.append(total_dist)

    ti_list = np.array(ti_list) / total_dist
    ti_list = np.insert(ti_list, 0, 0)

    # At each point, there will be a corresponding Lagrange polynomial.
    # For example for the point at t_0 = 0, we need a polynomial which is 1 at t_0 but 0 for all other t_i.
    # After finding the aggregated polynomial, there is nothing else to do so we just construct and sample from the
    # polynomial simultaneously.
    curve_points = []
    dt = 0.005
    for idx in range(0, len(ti_list) - 1):
        t1, t2 = ti_list[idx], ti_list[idx + 1]
        for t in np.arange(t1, t2+dt, dt):
            summation = np.array([0., 0.])
            for i, ti in enumerate(ti_list):
                prod = 1
                for tj in ti_list:
                    if tj != ti:
                        prod *= (t - tj) / (ti - tj)

                summation += prod * points[i]

            curve_points.extend(summation)

    return curve_points


tension = 0.5
def cardinal():
    # This is a Cardinal Hermite (Cubic) Spline curve. This means we essentially break up the
    # curve into sections comprising of 2 points each, and provide each point with a single slope
    # so that each section joins smoothly. With each section having 2 points and 2 slopes, we can
    # fit a unique cubic polynomial through it. Note: An alternative approach would be using the
    # Bézier viewpoint, i.e. generating 2 additional Bézier control points per segment using the
    # 2 slopes and 2 points and doing the 1/3 thing, but this would result in the exact same spline.
    global points, tension
    assert len(points) >= 4

    # Starting with the second point and working our way to the second to last point, we directly
    # begin sampling from the curve, building it simultaneously.
    curve_points = []
    for idx in range(1, len(points)-2):
        # The points p1 and p2 are the interpolants, but p0 and p3 are needed to find the slopes there.
        p0, p1, p2, p3 = points[idx-1], points[idx], points[idx+1], points[idx+2]

        # Slope vector calculation
        m1, m2 = (p2 - p0) * tension, (p3 - p1) * tension

        # Hermite (cubic) interpolation
        dt = 0.01
        for t in np.arange(0, 1, dt):
            addend1 = ((2*np.power(t, 3)) - (3*np.power(t, 2)) + 1) * p1
            addend2 = ((np.power(t, 3)) - (2*np.power(t, 2)) + t) * m1
            addend3 = ((-2*np.power(t, 3)) + (3*np.power(t, 2))) * p2
            addend4 = (np.power(t, 3) - np.power(t, 2)) * m2
            curve_points.extend(addend1 + addend2 + addend3 + addend4)

    return curve_points


def bézier():
    # This is an implementation of the De-Casteljau Algorithm for computing the
    # Bernstein polynomial linear combination of the control points.
    global points

    curve_points = []
    dt = 0.005
    for t in np.arange(0, 1+dt, dt):
        curve_points.extend(casteljau(t, points))

    return curve_points


# Recursive helper function for the above.
def casteljau(t, points):
    '''
        Takes in a set of control points and a value of t.
        Returns the value of the Bézier curve formed by those control points at the point t.
    '''
    n = len(points)
    assert n >= 2

    # Base case
    if n == 2:
        return ((1-t)*points[0]) + (t*points[1])

    # Recursive case
    else:
        new_points = []
        for idx in range(len(points)-1):
            p0, p1 = points[idx], points[idx+1]
            p_mid = ((1-t)*p0) + (t*p1)
            new_points.append(p_mid)

        return casteljau(t, new_points)


# B-Spline vars
degree = 2
knot_vector_x, knot_vector_y = [], []
def bspline():
    # This is NOT an implementation of the De-Boor Algorithm. Instead,
    # we directly use the Cox-de-Boor recursion formula for the B-spline basis functions
    # to compute each dimension of the curve.
    global points, degree, knot_vector_x, knot_vector_y

    # Build the desired knot vector. Replace the functions with whatever is desired (e.g. padded_knots()).
    knot_vector_x = padded_uniform_knots()
    knot_vector_y = padded_uniform_knots()

    # Parametric domain is [0, b]
    b = degree + len(points)

    # Sample points from the domain.
    curve_points = []
    dt = 0.01
    for t in np.arange(0, b, dt):
        sum_x, sum_y = 0, 0
        for i in range(len(points)):
            sum_x += points[i][0] * B(i, degree, t, knot_vector_x)
            sum_y += points[i][1] * B(i, degree, t, knot_vector_y)

        curve_points.extend([sum_x, sum_y])

    return curve_points


# B-spline basis recursive function
def B(i, deg, t, knot_vec):
    '''
        Returns the value of the degree-'deg' B-spline basis function (that is supported on [knot[i], knot[i+deg+1]])
        at that point t in the parametric domain [0, b] where b = degree + number of control points.

        The 'deg' is the degree of the basis curve that is required at the time of the call (i.e. we might be in a
        recursive call where the required basis degree is 2, 1, etc), whereas the global degree variable is the actual
        degree of the final curve.
    '''
    global degree

    # Make sure basis function is well defined (i.e. not exiting outside scope of parametric domain).
    # Below, we assume that the final value in the knot vector is precisely b.
    b = knot_vec[-1]
    assert 0 <= i <= b - (deg + 1)

    # Base case
    if deg == 0:
        return 1 if knot_vec[i] <= t < knot_vec[i+1] else 0

    # Recursive case
    else:
        # Note that the support of each of the two degree 'deg'-1 basis functions is of length
        # 'deg', since the support of this current basis function has length 'deg'+1.
        # In the special case that knot_vec[i+deg] = knot_vec[i], or knot_vec[i+deg+1] = knot_vec[i+1],
        # meaning certain knots have been duplicated, we simply let the respective ramping = 0.
        ramp_up = (t - knot_vec[i]) / (knot_vec[i+deg] - knot_vec[i]) if knot_vec[i+deg] != knot_vec[i] else 0
        ramp_down = (knot_vec[i+deg+1] - t) / (knot_vec[i+deg+1] - knot_vec[i+1]) if knot_vec[i+deg+1] != knot_vec[i+1] else 0

        return (ramp_up * B(i, deg-1, t, knot_vec)) + (ramp_down * B(i+1, deg-1, t, knot_vec))


# Knot Vector initializers
def uniform_knots():
    '''
        Function that uses the points plotted and the degree to return the knot vectors
        for each dimension (x and y).

        NOTE: that the parametric domain will be [0, b] where b = degree + number of control points,
        and the total number of knots (including the one at zero) will be b+1 = (degree + 1) + number of control points.
    '''
    global points, degree
    return [i for i in range(0, degree + len(points) + 1)]


# Padded Knots
def padded_uniform_knots():
    global points, degree
    b = degree + len(points)

    knot_vec = [0 for _ in range(degree+1)]
    knot_vec.extend([i for i in range(degree+1, b - degree)])
    knot_vec.extend([b for _ in range(degree+1)])
    return knot_vec

# END SCHEME IMPLEMENTATION —————————————————————————————————


# KEY METHODS ———————————————————————————————————————————————
def key_pressed(event):
    global scheme, tension

    # Change scheme if valid key.
    try:
        scheme = int(event.char)  # Might throw ValueError
        w.itemconfig('scheme_text', text='Scheme: ' + scheme_names[scheme])  # Might throw KeyError
    except (KeyError, ValueError):
        if not (scheme == 2 and event.char in {'o', 'p'}):
            print('Not a valid key.')

    # Do the interpolation based on the current scheme.
    if scheme != 0:
        w.delete('line')
        w.delete('knot_line')
        w.delete('tension_text')
        w.create_line(get_curve_points(scheme), fill='red', width=2, tags='line')

        if scheme == 2:
            w.create_text(window_w/2, 60, text='Tension='+str(np.round(tension, decimals=2))+". Use O/P to change.", fill='blue', font='Avenir 30', tags='tension_text')

    else:
        w.delete('knot_line')
        w.delete('line')

    # If Cardinal spline and 'O' or 'P' pressed, change tension parameter.
    dtau = 0.1
    if scheme == 2:
        if event.char == 'o':
            tension = min(max(0, tension - dtau), 1)
        elif event.char == 'p':
            tension = min(max(0, tension + dtau), 1)


# END KEY METHODS ———————————————————————————————————————————


# MOUSE METHODS —————————————————————————————————————————————
def mouse_click(event):
    global points, point_radius, click_detection_radius, point_dragged_index, point_fill, point_outline

    # Clicked point
    clicked_pt = np.array([event.x, event.y])

    # Check if clicking on existing point
    for i, point in enumerate(points):
        if np.linalg.norm(point - clicked_pt) < click_detection_radius:
            point_dragged_index = i

    # If not clicked on existing point, create new one.
    # We'll tag each circle object as "pt_i" where 'i' is its index in the list of points.
    if point_dragged_index == -1:
        points.append(clicked_pt)
        w.create_oval(event.x - point_radius, event.y - point_radius, event.x + point_radius, event.y + point_radius,
                      fill=point_fill, outline=point_outline, tags='pt_'+str(len(points)-1))

        # Recalculate and redraw curve if scheme is selected
        if scheme != 0:
            w.delete('line')
            w.create_line(get_curve_points(scheme), fill='red', width=2, tags='line')


def mouse_release(event):
    global point_dragged_index
    point_dragged_index = -1


def left_drag(event):
    global point_dragged_index, point_radius, points

    # Make sure we're actually dragging on a legit point
    if point_dragged_index != -1:
        # Update circle on screen
        w.coords('pt_'+str(point_dragged_index), event.x-point_radius, event.y-point_radius,
                 event.x+point_radius, event.y+point_radius)

        # Update stored value of point
        points[point_dragged_index] = np.array([event.x, event.y])

        # Recalculate and redraw curve, if needed
        if scheme != 0:
            w.delete('line')
            w.create_line(get_curve_points(scheme), fill='red', width=2, tags='line')


def motion(event):
    global point_hovered_index, points, hover_fill, hover_outline, point_fill, point_outline

    # Point location
    mouse_point = np.array([event.x, event.y])

    # Check if mouse left range of point it is currently hovering, if any.
    # If it did, swap the colors back and set the index to -1
    if point_hovered_index != -1:
        if np.linalg.norm(points[point_hovered_index] - mouse_point) > click_detection_radius:
            w.itemconfig('pt_'+str(point_hovered_index), fill=point_fill, outline=point_outline)
            point_hovered_index = -1

    # Otherwise it actually was not hovering over anything to begin with, check if it now is.
    # If it now is, swap the colors to the new hover colors.
    else:
        for i, point in enumerate(points):
            if np.linalg.norm(point - mouse_point) < click_detection_radius:
                w.itemconfig('pt_' + str(i), fill=hover_fill, outline=hover_outline)
                point_hovered_index = i
# END MOUSE METHODS —————————————————————————————————————————

# Mouse bind
w.bind('<Motion>', motion)
w.bind("<Button-1>", mouse_click)
w.bind("<ButtonRelease-1>", mouse_release)
w.bind('<B1-Motion>', left_drag)

# Key bind
w.bind('<Key>', key_pressed)

# Display informational text
key_map_text = ''
for key, val in scheme_names.items():
    key_map_text += str(key).upper() + ': ' + val.upper() + '    '

w.create_text(window_w/2, window_h - 25, text=key_map_text, fill='red', font='Avenir 30')
w.create_text(window_w/2, 25, text='SCHEME: NONE', fill='black', font='Avenir 30', tags='scheme_text')

# Create the line object
w.create_line(-1, -1, -2, -2, tags='line')

# Necessary line for Tkinter
mainloop()