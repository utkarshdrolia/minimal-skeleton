// BVH_Player.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <glad/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include "linmath.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "Skeleton.h"
#include "AnimRec.h"
#include "defs.h"


static const char* vertex_shader_text =
"#version 110\n"
"uniform mat4 MVP;\n"
"attribute vec3 vCol;\n"
"attribute vec3 vPos;\n"
"varying vec3 color;\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(vPos, 1.0);\n"
"    color = vCol;\n"
"}\n";

static const char* fragment_shader_text =
"#version 110\n"
"varying vec3 color;\n"
"void main()\n"
"{\n"
"    gl_FragColor = vec4(color, 1.0);\n"
"}\n";

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}



int main(void)
{
    GLFWwindow* window;
    GLuint vertex_buffer, vertex_shader, fragment_shader, program;
    GLint mvp_location, vpos_location, vcol_location;

    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
        exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(800, 600, "BVH Player", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwSetKeyCallback(window, key_callback);

    glfwMakeContextCurrent(window);
    gladLoadGL(glfwGetProcAddress);
    glfwSwapInterval(1);


    Skeleton skel;
    AnimRec record;
    //Hardcoded the bvh file here.  
    //Feel free to add an appropriate GUI file chooser if you like
    skel.CreateSkeletonFromBVH((char*)"ZooExcited.bvh", &record, false);
    //The files below can be helpful in debugging.  Just comment out the skeleton update in the main loop
//    skel.CreateSkeletonFromBVH((char*)"TestBlank.bvh", &record, false);
//    skel.CreateSkeletonFromBVH((char*)"TestZ45.bvh", &record, false);
//    skel.CreateSkeletonFromBVH((char*)"TestZ45Y90.bvh", &record, false);
    skel.AddGeometry();
    double state[500];
    record.GetFrame(0, state);
    skel.SetSkelState(state);
    //This calculates the transformations.  You need to write this.
    skel.UpdateLinks();

    //This is not an elegant way to code this!
    int maxEntries = 12 * 100;
    VERTEX* verts = new VERTEX[maxEntries];
    int vertCount = 0;
    skel.CalcVertexLocations(maxEntries, &vertCount, &verts);
    std::cout << "Total verts: " << vertCount;

    // NOTE: OpenGL error checks have been omitted for brevity

    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, maxEntries * 24, verts, GL_DYNAMIC_DRAW);

    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_text, NULL);
    glCompileShader(vertex_shader);

    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_text, NULL);
    glCompileShader(fragment_shader);

    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    mvp_location = glGetUniformLocation(program, "MVP");
    vpos_location = glGetAttribLocation(program, "vPos");
    vcol_location = glGetAttribLocation(program, "vCol");

    glEnableVertexAttribArray(vpos_location);
    glVertexAttribPointer(vpos_location, 3, GL_FLOAT, GL_FALSE,
        sizeof(verts[0]), (void*)0);
    glEnableVertexAttribArray(vcol_location);
    glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE,
        sizeof(verts[0]), (void*)(sizeof(float) * 3));
    float startTime = (float)glfwGetTime();
    float duration = record.GetEndTime();
    float frameTime = record.GetFrameTime();

    while (!glfwWindowShouldClose(window))
    {
        float ratio;
        int width, height;
        mat4x4 m, p, mvp;

        float curTime = (float)glfwGetTime();
        float animTime = curTime - startTime;
        if (animTime > duration)
        {
            startTime = curTime;
            animTime = 0;
        }
        //std::cout << animTime << std::endl;
        record.GetFrame(animTime/frameTime, state);
        skel.SetSkelState(state);


        skel.UpdateLinks();


        int vertCount = 0;
        skel.CalcVertexLocations(maxEntries, &vertCount, &verts);




//NOTE: HARDCODE SIZE
        glBufferData(GL_ARRAY_BUFFER, 24*vertCount, verts, GL_DYNAMIC_DRAW);

        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (float)height;

        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);

        mat4x4_identity(m);
        mat4x4_scale(m, m, 0.9);
        m[3][3] = 1.0;


        //TO DO: replace this with a proper camera model
        mat4x4_ortho(p, -1., 1., -1.f, 1.f, 1.f, -1.f);
        mat4x4_mul(mvp, p, m);

        glUseProgram(program);
        glUniformMatrix4fv(mvp_location, 1, GL_FALSE, (const GLfloat*)mvp);

//THIS SPECIFIES WHICH VERTICES WILL BE DRAWN
        //Don't draw the pyramid from the world frame to the root (so start at 12)
        glDrawArrays(GL_TRIANGLES, 12, vertCount);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    delete[] verts;
    glfwDestroyWindow(window);

    glfwTerminate();
    exit(EXIT_SUCCESS);
}



/*
#include <iostream>

int main()
{
    std::cout << "Hello World!\n";
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
*/