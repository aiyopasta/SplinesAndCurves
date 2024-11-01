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


# TODO: Replace this with sympy code that can just eat the poly itself and compute the Blossom automatically.
# BLOSSOM BANK ————————————————————————————————————————————————————————————————————————————————————————————————————————
# I. The nodal cubic (a POLYNOMIAL curve, i.e. NOT closed, defined on [-infty, +infty])
#    Formula: (x(t), y(t)) = (t^2 - 1, t^3 - t)  -->   B(u, v, w) = [(uv + uw + vw) / 3 - 1, uvw - ((u + v + w) / 3)]
def nodal_cubic_blossom(uvw: np.ndarray):
    u_, v_, w_ = uvw[0], uvw[1], uvw[2]
    return np.array([(((u_ * v_) + (u_ * w_) + (v_ * w_)) / 3.0) - 1.0, (u_ * v_ * w_) - ((u_ + v_ + w_) / 3.0)])


# TODO: Add more!

# ASSOCIATED DOMAIN BANK
# I. Nodal cubic
nodal_cubic_interval = (-1.4, +1.4)


# The chosen blossom function to use
blossom = nodal_cubic_blossom
poly_deg = 3

# Domain of the function (r, s)
r, s = nodal_cubic_interval

# Number of times to subdivide de Casteljau algo (0 = NO subdivisions, outer-most points only)
sub_divs = 6

# Compute the list of control points
# (i) Compute the initial, outer-most ones using blossom, e.g. for deg=3: B(r, r, r), B(r, r, s), B(r, s, s), B(s, s, s)
control_pts = [blossom(np.concatenate([np.full(poly_deg - k, r), np.full(k, s)])) for k in range(0, poly_deg+1)]
# (ii) Compute the intermediate ones, one recursive batch for each subdivision level
def subdivide(ctrl_pts: list, level=0) -> list:
    assert len(ctrl_pts) == 4
    if level == 0: return ctrl_pts
    b01, b11, b21 = (ctrl_pts[0] + ctrl_pts[1]) / 2.0, (ctrl_pts[1] + ctrl_pts[2]) / 2.0, (ctrl_pts[2] + ctrl_pts[3]) / 2.0
    b02, b12 = (b01 + b11) / 2.0, (b11 + b21) / 2.0
    b03 = (b02 + b12) / 2.0
    return list(np.concatenate([subdivide([ctrl_pts[0], b01, b02, b03], level=level-1), subdivide([b03, b12, b21, ctrl_pts[3]], level=level-1)[1:]]))


# Override with intermediate ones
control_pts = subdivide(control_pts, level=sub_divs)

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
    batch = pyglet.graphics.Batch()
    line_objects = []  # need to store pyglet line objects in memory till the time the batch is drawn i think
    for i in range(len(control_pts) - 1):
        line_objects.append(shapes.Line(*A(control_pts[i] * scale), *A(control_pts[i+1] * scale), width=2, color=colors['red'], batch=batch))
        shapes.Circle(*A(control_pts[i] * scale), radius=7, color=colors['blue']).draw()

    batch.draw()
    shapes.Circle(*A(control_pts[-1] * scale), radius=7, color=colors['blue']).draw()  # draw final pt


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
