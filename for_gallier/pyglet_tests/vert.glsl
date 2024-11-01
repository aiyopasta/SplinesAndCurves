#version 410 core  // required to be first line

#define PI 3.141592653589793238464338327950288419716939937510582

// Ah okay so the "WindowBlock" thingy is required by Pyglet. It is a "uniform block" called "window" of type
// "WindowBlock" that's set automatically when the window is created. Just keep it the way it is.
// For more details: https://pyglet.readthedocs.io/en/latest/programming_guide/rendering.html
in vec3 a_pos;
in vec3 a_nor;

out vec4 v_pos;
out vec3 v_nor;
out vec3 v_viewdir;

// uniforms
uniform vec2 dims;       // width, height of screen
uniform mat4 view_mat;
uniform mat4 proj_mat;


void main()
{
    // good practice to use 'in's and 'uniform's at least once here (from personal experience)
    a_pos; a_nor; dims; view_mat; proj_mat;

    // Main code ------
    gl_Position = proj_mat * view_mat * vec4(a_pos.xyz, 1.0);

    // Send other attributes to interpolate
    v_pos = vec4(0.0); // TODO
    v_nor = mat3(view_mat) * a_nor;
    v_viewdir = vec3(0.0);

}