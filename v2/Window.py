# -*- coding: UTF-8 -*-
import cyglfw3 as glfw
from OpenGL.GL import *
from OpenGL.GL.NV.bindless_texture import *

class Window(object):

    def __init__(self, window_width, window_height, samples, window_title):
        self.window_title = window_title
        assert glfw.Init(), 'Glfw Init failed!'
        glfw.WindowHint(glfw.SAMPLES, samples)
        self.windowID = glfw.CreateWindow(window_width, window_height, self.window_title, None)
        assert self.windowID, 'Could not create Window!'
        glfw.MakeContextCurrent(self.windowID)
        assert glInitBindlessTextureNV(), 'Bindless Textures not supported!'
        self.framebuf_width, self.framebuf_height = glfw.GetFramebufferSize(self.windowID)
        def framebuffer_size_callback(window, w, h):
            self.framebuf_width, self.framebuf_height = w, h
        glfw.SetFramebufferSizeCallback(self.windowID, framebuffer_size_callback)
        def key_callback(window, key, scancode, action, mode):
            if action == glfw.PRESS:
                if key == glfw.KEY_ESCAPE:
                    glfw.SetWindowShouldClose(window, True)
        glfw.SetKeyCallback(self.windowID, key_callback)

        self.previous_second = glfw.GetTime()
        self.frame_count = 0.0

        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE)
        glDisable(GL_STENCIL_TEST)
        glDisable(GL_BLEND)
        glDisable(GL_CULL_FACE)
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glEnable( GL_LINE_SMOOTH );
        glEnable( GL_POLYGON_SMOOTH );

    def update_fps_counter(self):
        current_second = glfw.GetTime()
        elapsed_seconds = current_second - self.previous_second
        if elapsed_seconds > 1.0:
            self.previous_second = current_second
            fps = float(self.frame_count) / float(elapsed_seconds)
            glfw.SetWindowTitle(self.windowID, '%s @ FPS: %.2f' % (self.window_title, fps))
            self.frame_count = 0.0
        self.frame_count += 1.0
