# -*- coding: UTF-8 -*-
import logging as log
from OpenGL.GL import *
from OpenGL.GL.NV.bindless_texture import *
import numpy as np
import pyassimp
from scipy import misc

class Texture(object):

    def __init__(self, filename=None):
        if isinstance(filename,str):
            self.handle, self.__tex, self.__H, self.__W = Texture.__loadTexture(filename)
        elif isinstance(filename, tuple):
            self.handle, self.__tex, self.__H, self.__W = Texture.__allocateTexture(filename[0],filename[1])

    @staticmethod
    def handlesToUBO(handles, binding_point=2):
        handles = np.array(handles,dtype=np.uint64)
        ubo = np.empty(1, dtype=np.uint32 )
        glCreateBuffers(len(ubo), ubo)
        glNamedBufferStorage( ubo[0], handles.nbytes, handles, 0)
        glBindBufferRange(GL_SHADER_STORAGE_BUFFER, binding_point, ubo[0], 0, handles.nbytes)

    @staticmethod
    def __loadTexture(filename):
        img = misc.imread(filename)
        H, W = img.shape[:2]
        tex = np.empty(1, dtype=np.uint32)
        handle = np.empty(1, dtype=np.uint64)
        glCreateTextures(GL_TEXTURE_2D, len(tex), tex)
        glTextureStorage2D(tex, 1, GL_RGB8, W, H)
        glTextureSubImage2D(tex, 0, 0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, img)
        glTextureParameteri(tex, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTextureParameteri(tex, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glGenerateTextureMipmap(tex)
        handle[:] = glGetTextureHandleNV(tex)
        glMakeTextureHandleResidentNV(handle)
        return handle, tex, H, W

    @staticmethod
    def __allocateTexture( H, W ):
        tex = np.empty(1, dtype=np.uint32)
        handle = np.empty(1, dtype=np.uint64)
        glCreateTextures(GL_TEXTURE_2D, len(tex), tex)
        glTextureStorage2D(tex, 1, GL_RGB8, W, H)
        glTextureParameteri(tex, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTextureParameteri(tex, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        handle[:] = glGetTextureHandleNV(tex)
        glMakeTextureHandleResidentNV(handle)
        return handle, tex, H, W

    @staticmethod
    def __loadTexture(filename):
        img = misc.imread(filename)
        H, W = img.shape[:2]
        tex = np.empty(1, dtype=np.uint32)
        handle = np.empty(1, dtype=np.uint64)
        glCreateTextures(GL_TEXTURE_2D, len(tex), tex)
        glTextureStorage2D(tex, 1, GL_RGB8, W, H)
        glTextureSubImage2D(tex, 0, 0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, img)
        glTextureParameteri(tex, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTextureParameteri(tex, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glGenerateTextureMipmap(tex)
        handle[:] = glGetTextureHandleNV(tex)
        glMakeTextureHandleResidentNV(handle)
        return handle, tex, H, W

    def getHandle(self):
        return self.handle

    def update(self, image):
        glTextureSubImage2D(self.__tex, 0, 0, 0, 
            self.__W, self.__H, GL_RGB, GL_UNSIGNED_BYTE, image)