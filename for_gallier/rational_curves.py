import os
import numpy as np
import pyglet
from pyglet.gl import *
from pyglet import shapes

# Suppress sci-notation
np.set_printoptions(suppress=True)

# Save the animation? TODO: Make sure you're saving to correct destination!!
save_anim = False

# Animation saving setup
count = 0  # two vars that just match my usual animation setup
frame = -1
path_to_save = ''
if save_anim:
    for filename in os.listdir(path_to_save):
        # Check if the file name follows the required format
        b1 = filename.startswith("frame") and filename.endswith(".png")
        b2 = filename.startswith("output.mp4")
        if b1 or b2:
            os.remove(os.path.join(path_to_save, filename))
            print('Deleted frame ' + filename)

# Initialize the window
width, height = 1720, 1000
config = pyglet.gl.Config(double_buffer=True, major_version=3, minor_version=3)
window = pyglet.window.Window(width=width, height=height, caption="Polynomial Curve Control Points", config=config)

# The usual coordinate shift
def A(v: np.ndarray):
    global width, height
    return v + np.array([width / 2.0, height / 2.0])


# This function is called SEPARATELY every dt seconds.
def update(dt_):
    pass


# TODO: Replace this with sympy code that can just eat the rational fn itself and compute the Blossom automatically.
# BLOSSOM BANK ————————————————————————————————————————————————————————————————————————————————————————————————————————
# I. Circle in rational form (a CLOSED curve, defined on [-infty, +infty])
#     Formula: (x(t), y(t)) = (1 - t^2) / (1 + t^2), 2t / (1 + t^2)
#     After homogenization: (x(X, Z), y(X, Z), z(X, Z)) = Z^2 - X^2, 2XZ, Z^2 + X^2
#     Blossom: B((x1, x2), (z1, z2)) = (z1 * z2) - (x1 * x2), ((x1 * z2) + (x2 * z1)), (z1 * z2) + (x1 * x2)
def rational_circle_blossom(x1: float, z1: float, x2: float, z2: float):
    return np.array([(z1 * z2) - (x1 * x2), ((x1 * z2) + (x2 * z1)), (z1 * z2) + (x1 * x2)])

# TODO: Add more!

# ASSOCIATED DOMAIN BANK
# I. Rational circle
rational_circle_interval = ((0, 1), (1, 1))


# The chosen blossom function to use
blossom = rational_circle_blossom
poly_deg = 2

# Domain of the function (r, s)
(r1, s1), (r2, s2) = rational_circle_interval

# Number of times to subdivide de Casteljau algo (0 = NO subdivisions, outer-most points only)
sub_divs = 0

# Compute the list of control points
# (i) Compute the initial, outer-most ones using blossom, e.g. for deg=3: B(r, r, r), B(r, r, s), B(r, s, s), B(s, s, s)
control_pts = [blossom(*np.concatenate([np.tile([r1, s1], poly_deg - k), np.tile([r2, s2], k)])) for k in range(0, poly_deg+1)]

# Scaling factor (equal for both dimensions)
scale = 250.0

# Apple colors!
colors = {
    'red': np.array([255, 59, 48]),
    'orange': np.array([255, 149, 0]),
    'yellow': np.array([255, 204, 0]),
    'green': np.array([40, 205, 65]),
    'mint': np.array([0, 199, 190]),
    'teal': np.array([89, 173, 196]),
    'cyan': np.array([85, 190, 240]),
    'blue': np.array([0, 122, 255]),
    'indigo': np.array([88, 86, 214]),
    'purple': np.array([175, 82, 222]),
    'pink': np.array([255, 45, 85]),
    'brown': np.array([162, 132, 94]),
    'gray': np.array([142, 142, 147]),
    'white': np.array([255, 255, 255])
}


@window.event
def on_draw():
    global width, height, control_pts, scale
    window.clear()

    # Draw polyline from ctrl points (in a single Batch) + plot the points themselves too
    # TODO

    # Save frame, increment counts —————————————————————————————————————————————————————————————————————————————————
    global count, save_anim, path_to_save
    count += 1
    if save_anim:
        image = pyglet.image.get_buffer_manager().get_color_buffer().get_image_data()
        image.save(path_to_save+'/frame'+str(count)+'.png')
        print('Saved frame '+str(count))



@window.event
def on_resize(w, h):
    glViewport(0, 0, w, h)


@window.event
def on_close():
    global save_anim, path_to_save, window

    # Compile all images into video with ffmpeg
    if save_anim:
        input_files = path_to_save + '/frame%d.png'
        output_file = path_to_save + '/output.mp4'
        ffmpeg_path = "/opt/homebrew/bin/ffmpeg"
        os.system(f'{ffmpeg_path} -r 60 -i {input_files} -vcodec libx264 -crf 25 -pix_fmt yuv420p -vf "eq=brightness=0.00:saturation=1.0" {output_file} > /dev/null 2>&1')
        print('Saved video to ' + output_file)

    window.close()


# Schedule the update function to be called at 60Hz
pyglet.clock.schedule_interval(update, 1 / 60.0)

# Run the Pyglet event loop
pyglet.app.run()
