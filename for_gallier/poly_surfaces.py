import numpy as np
import pyglet.graphics
from pyglet.gl import *
from pyglet import shapes
from pyglet.graphics.shader import Shader, ShaderProgram
import ctypes
import trimesh

# Window size
scale = 0.7  # 0.54 on laptop, 0.7 on main
width, height = int(1920 * scale), int(1920 * scale)
dims = np.array([width, height])


# The usual coordinate shift
def A(v: np.ndarray):
    global width, height
    return v + np.array([width / 2.0, height / 2.0])


# Initialize the window
config = pyglet.gl.Config(double_buffer=True, depth_size=24, major_version=3, minor_version=3)
window = pyglet.window.Window(width=width, height=height, caption="Polynomial Surface Control Points", config=config)


# REGULAR MATH FUNCTIONS ——————————————————————————————————————————————————————————————
def normalize(v: np.ndarray):
    assert np.linalg.norm(v) != 0.0, 'length is 0, cannot normalize!'
    return v / np.linalg.norm(v)


# 3D CAMERA ————————————————————————————————————————————————————————————————————————————————————————————————————————————
# Camera parameters
focus = 0.08  # also in [-1, 1]^3 coords
far_clip = 10.0  # optional, really.
cam_width = 0.1  # width of "fictitious camera sensor" DIVIDED BY 2
cam_height = cam_width * (height / width)  # Its height determined by aspect ratio of screen
cam_dims = np.array([cam_width, cam_height])
rho, theta, phi = 0.3, 0.0, np.pi / 2.0
cam_delta = 0.03  # how much camera rotates per frame WADS held


# View matrix
def view_mat():
    global rho, theta, phi, width, height, cam_width
    world_up = np.array([0.0, 0.0, 1.0])
    eye = rho * np.array([np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)])
    z_hat = normalize(eye)  # pointing OUT of screen!
    x_hat = normalize(np.cross(world_up, z_hat))  # TODO: "STAR"
    y_hat = normalize(np.cross(z_hat, x_hat))
    x_hat, y_hat, z_hat = [np.append(v, 0.0) for v in [x_hat, y_hat, z_hat]]
    d = np.append(eye, 1.0)
    return np.linalg.inv(np.column_stack([x_hat, y_hat, z_hat, d]))


# Projection matrix
def proj_mat():
    global focus, far_clip, width, height, cam_width, cam_height
    wby2, hby2 = cam_width / 2.0, cam_height / 2.0
    n, f = -focus, -(far_clip + focus)  # the z-coordinates of the near and far clipping planes. They're negative!!
    return np.array([[-n / wby2, 0.0, 0.0, 0.0],
                     [0.0, -n / hby2, 0.0, 0.0],
                     [0.0, 0.0, -(f + n) / (f - n), 2. * f * n / (f - n)],  # [0, 1] * 2 - 1
                     [0.0, 0.0, -1.0, 0.0]])  # Keep the w-coordinate positive for opengl. since the z-coordinate will be negative, the -1 ensures this'll happen. But then we have to negate the other coords too.


# Helper functions for manual projection (without opengl)
homo = lambda v: np.array([*v, 1.0])  # homogenize
dehomo = lambda v: v[:-1]  # dehomogenize

# CREATE SHADER PROGRAM ————————————————————————————————————————————————————————————————————————————————————————————————
vertex_src_einstein = open('pyglet_tests/vert.glsl').read()
frag_src_einstein = open('pyglet_tests/frag.glsl').read()
program_einstein = ShaderProgram(Shader(vertex_src_einstein, 'vertex'),
                                 Shader(frag_src_einstein, 'fragment'))


# CREATE VBO (slightly different way than others; i.e. using explicit vertex list face list)
# Load in data
mesh = trimesh.load_mesh('pyglet_tests/einstein.obj')
mesh.vertices -= mesh.center_mass
einstein_verts = mesh.vertices
einstein_faces = mesh.faces
einstein_nors = mesh.vertex_normals

# Load in control points to plot using pyglet commands
ctrl_pts = einstein_verts[:100, :]  # TODO: for now! change to something legit!

# Prepare mesh data for sending over to shader
einstein_verts = einstein_verts.flatten()
einstein_nors = einstein_nors.flatten()
einstein_faces = einstein_faces.flatten().astype(int)
# Create the buffer object
einstein_vlist = program_einstein.vertex_list_indexed(int(len(einstein_verts) / 3), GL_TRIANGLES, einstein_faces,
                                                      a_pos=('f', einstein_verts),
                                                      a_nor=('f', einstein_nors))

# SEND IN UNIFORMS (that won't need updating)
program_einstein['dims'] = np.array([width, height])

# APPLE COLORS! ————————————————————————————————————————————————————————————————————————————————————————————————————————
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


# This function is called SEPARATELY at 60Hz (set below). on_draw() is called as fast as possible, separately!
# Update is simply where game / simulation logic goes.
def update(dt_):
    # PLAY, PAUSE, REWIND TIME UPDATE ——————————————————————————————————————————————————————————————————————————————————
    global focus, keys_held, rho, theta, phi, cam_delta

    # ALBERT EINSTEIN SHADER UPDATE ————————————————————————————————————————————————————————————————————————————————————
    program_einstein['view_mat'] = view_mat().T.flatten()
    program_einstein['proj_mat'] = proj_mat().T.flatten()

    # KEYS HELD HANDLING ———————————————————————————————————————————————————————————————————————————————————————————————
    for symbol in keys_held:
        eps = 1e-3
        if symbol == pyglet.window.key.W: phi = max(phi - cam_delta, eps)
        if symbol == pyglet.window.key.S: phi = min(phi + cam_delta, np.pi - eps)
        if symbol == pyglet.window.key.A: theta -= cam_delta
        if symbol == pyglet.window.key.D: theta += cam_delta
        if symbol == pyglet.window.key.P: rho = min(rho + (cam_delta / 2.0), 0.4)
        if symbol == pyglet.window.key.O: rho = max(rho - (cam_delta / 2.0), 0.2)


# EVENT HANDLING ———————————————————————————————————————————————————————————————————————————————————————————————————————
# Set to track held-down keys
keys_held = set()


@window.event
def on_key_press(symbol, modifiers):
    # TODO: make control nicer by moving the below to "update" and keeping a variable called "key_held"
    global keys_held
    keys_held.add(symbol)


@window.event
def on_key_release(symbol, modifiers):
    global keys_held

    # Remove the key from the keys_held set
    if symbol in keys_held: keys_held.discard(symbol)


# DRAWING ——————————————————————————————————————————————————————————————————————————————————————————————————————————————
@window.event
def on_draw():
    global ctrl_pts, cam_dims, dims

    # Wipe it clean
    window.clear()

    # I guess these two lines really have to be called on every draw call
    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
    gl.glEnable(gl.GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    # SHADER PROGRAM DRAW CALLS ————————————————————————————————————————————————————————————————————————————————————————
    # Draw Einstein shader program elements
    program_einstein.use()
    einstein_vlist.draw(GL_TRIANGLES)
    program_einstein.stop()

    # SIMPLE PYGLET DRAW CALLS ————————————————————————————————————————————————————————————————————————————————————————
    # Accumulate line pts
    circles = []
    for raw_pt in ctrl_pts:
        xyz1 = proj_mat() @ view_mat() @ homo(raw_pt)
        xyz = dehomo(xyz1 / xyz1[-1])
        pt = A(xyz[:-1] * (dims / 2.0))
        if np.all((pt >= 0) & (pt <= dims)):
            circles.append(pt)

    # Draw them in a single Batch
    batch = pyglet.graphics.Batch()
    circ_objects = []  # need to store pyglet line objects in memory till the time the batch is drawn i think
    for circ in circles:
        circ_objects.append(shapes.Circle(*circ, radius=5, color=colors['red'], batch=batch))

    batch.draw()


@window.event
def on_resize(w, h):
    glViewport(0, 0, w, h)


# Schedule the update function to be called at 60Hz
pyglet.clock.schedule_interval(update, 1 / 60.0)

# Run the Pyglet event loop
pyglet.app.run()
