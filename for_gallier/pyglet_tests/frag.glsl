#version 410 core  // must be the first line

// ins-outs
in vec4 v_pos;
in vec3 v_nor;
in vec3 v_viewdir;
out vec4 f_col;

void main()
{
    // good practice to use 'in's and 'uniform's at least once here (from personal experience)
    v_pos; v_nor;

//    float shading = dot(v_nor, v_viewdir);
//    vec3 col = vec3(0.0, 0.3, 0.8) * (1.0 - shading);
//    f_col = vec4(col, 1.0);
    f_col = vec4(vec3(v_nor), 1.0);
}