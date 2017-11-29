# -*- coding: UTF-8 -*-
from OpenGL.GL import *
from OpenGL.GL.NV.bindless_texture import *
import numpy as np

class Framebuffer(object):

    def __init__(self, H, W):
        self.tex = np.empty(2, dtype=np.uint32)
        glCreateTextures(GL_TEXTURE_2D, len(self.tex), self.tex)
        glTextureStorage2D(self.tex[0], 1, GL_RGB8, W, H)
        glTextureParameteri(self.tex[0], GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTextureParameteri(self.tex[0], GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTextureStorage2D(self.tex[1], 1, GL_DEPTH_COMPONENT32F, W, H)
        glTextureParameteri(self.tex[1], GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTextureParameteri(self.tex[1], GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        
        self.fbo = np.empty(1, dtype=np.uint32)
        glCreateFramebuffers(len(self.fbo), self.fbo)
        glNamedFramebufferTexture(self.fbo, GL_COLOR_ATTACHMENT0, self.tex[0], 0)
        glNamedFramebufferTexture(self.fbo, GL_DEPTH_ATTACHMENT, self.tex[1], 0)

        self.color_handle = glGetTextureHandleNV(self.tex[0])
        glMakeTextureHandleResidentNV(self.color_handle)
        self.depth_handle = glGetTextureHandleNV(self.tex[1])
        glMakeTextureHandleResidentNV(self.depth_handle)

        assert glCheckNamedFramebufferStatus(self.fbo, GL_FRAMEBUFFER)\
                == GL_FRAMEBUFFER_COMPLETE, 'Framebuffer not complete'

    def bind(self):
        glBindFramebuffer(GL_FRAMEBUFFER, self.fbo)

