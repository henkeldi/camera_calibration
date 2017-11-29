# -*- coding: UTF-8 -*-
import logging as log
from OpenGL.GL import *
import numpy as np

class Camera(object):


    def __init__(self):
        self.__T_world_view = np.eye(4, dtype=np.float32)
        self.__T_view_world = np.eye(4, dtype=np.float32)

        self.__T_view_proj = np.eye(4, dtype=np.float32)
        self.__T_proj_view = np.eye(4, dtype=np.float32)

        self.__T_proj_world = np.eye(4, dtype=np.float32)

        self.__viewport = (0,0,0,0)

        self.dirty = False


    def lookAt(self, pos, target, up):
        pos = np.array(pos,dtype=np.float32); target = np.array(target, dtype=np.float32); up = np.array(up, dtype=np.float32)
        z = pos - target; z *= (1.0 / np.linalg.norm(z))
        x = np.cross(up, z); x *= (1.0/np.linalg.norm(x))
        y = np.cross(z,x)
        rot = np.vstack((x,y,z))
        self.__T_view_world[:3,:3] = rot
        self.__T_view_world[:3,3] = -np.dot(rot,pos)
        self.__T_world_view[:3,:3] = rot.transpose()
        self.__T_world_view[:3,3] = pos
        self.__T_proj_world[:] = np.dot(self.__T_proj_view, self.__T_view_world)
        self.dirty = True


    def projection(self, fov, aspect, near, far):
        diff = near - far
        A = np.tan(fov/2.0)
        self.__T_proj_view[:] = np.array(   [   [A/aspect,0,0,0],
                                            [0,A,0,0],
                                            [0,0,(far+near)/diff,2*far*near/diff],
                                            [0,0,-1,0] ], dtype=np.float32)
        self.__T_view_proj[:] = np.linalg.inv(self.__T_proj_view)
        self.__T_proj_world[:] = np.dot(self.__T_proj_view, self.__T_view_world)
        self.dirty = True


    def ortho(self, left, right, bottom, top, nearVal, farVal):
        self.__T_proj_view[:] = Camera.__glOrtho__(left, right, bottom, top, nearVal, farVal)
        self.__T_view_proj[:] = np.linalg.inv(self.__T_proj_view)
        self.__T_proj_world[:] = np.dot(self.__T_proj_view, self.__T_view_world)
        self.dirty = True

    def realCameraIntrinsic(self, fx, fy, x0, y0, W, H, near, far):
        I = np.array([  [fx,   0.0,    x0],
                        [0.,    fy,     y0],
                        [0.,    0,      1.0]])
        self.setIntrinsic(I, W, H, near, far)

    def setIntrinsic(self, I, W, H, near, far, originIsInTopLeft=True):
        '''
        Args:
            I:                  3x3 intrinsic camera matrix from real camera (without any OpenGL stuff)
            W:                  Width of the camera image
            H:                  Height of the camera image
            near:               Near plane
            far:                Far plane
            originIsInTopLeft:  If True then the image origin is in top left
                                if False the image origin is in image center
        
            Source: http://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/
        '''
        Camera.__check_matrix__(I)

        A = near + far
        B = near * far
        persp = np.array( [ [ I[0,0], I[0,1], -I[0,2], 0 ],
                            [ 0     , I[1,1], -I[1,2], 0 ],
                            [ 0     , 0     , A      , B ],
                            [ 0     , 0     , -1     , 0 ] ] , dtype=np.float64)
        ortho = Camera.__glOrtho__(0, W, H, 0, near, far) if originIsInTopLeft else\
                    Camera.__glOrtho__(-W/2., W/2., -H/2., H/2., near, far)

        self.__T_proj_view[:] = np.dot( ortho, persp ).astype(np.float32)
        self.__T_view_proj[:] = np.linalg.inv(self.__T_proj_view)
        self.__T_proj_world[:] = np.dot(self.__T_proj_view, self.__T_view_world)
        self.dirty = True


    @staticmethod
    def __check_matrix__(I):
        if len(I.shape) != 2:
            log.error('Camera Matrix not 2D but %dD' % len(I.shape))
            exit(-1)
        elif I.shape != (3,3):
            log.error('Camera Matrix is not 3x3 but %dx%d' % I.shape)
            exit(-1)
        elif I[1,0] != 0.0:
            log.error('Camera Matrix Error: Expected Element @ 1,0 to be 0.0 but it\'s: %.f' % I[1,0])
            exit(-1)
        elif I[2,0] != 0.0:
            log.error('Camera Matrix Error: Expected Element @ 2,0 to be 0.0 but it\'s: %.f' % I[2,0])
            exit(-1)
        elif I[2,1] != 0.0:
            log.error('Camera Matrix Error: Expected Element @ 2,1 to be 0.0 but it\'s: %.f' % I[2,1])
            exit(-1)
        elif I[2,2] != 1.0:
            log.error('Camera Matrix Error: Expected Element @ 2,2 to be 1.0 but it\'s: %.f' % I[2,2])
            exit(-1)
        else:
            log.debug('Camera Matrix valid.')


    @staticmethod
    def __glOrtho__(left, right, bottom, top, nearVal, farVal):
        '''
            Source: https://www.opengl.org/sdk/docs/man2/xhtml/glOrtho.xhtml
        '''
        tx = - (right+left) / (right-left)
        ty = - (top+bottom) / (top-bottom)
        tz = - (farVal+nearVal) / (farVal-nearVal)
        return np.array([   [ 2./(right-left),      0.,                     0.,                         tx  ],
                            [ 0.,                   2. / (top - bottom),    0.,                         ty  ],
                            [ 0.,                   0.,                     -2. / (farVal - nearVal),   tz  ],
                            [ 0.,                   0.,                     0.,                         1.  ]] , dtype=np.float64)

    def setViewport(self, x0, y0, w, h):
        self.__viewport = (int(x0), int(y0), int(w), int(h))

    def T_world_view(self):
        return self.__T_world_view

    def T_view_world(self):
        return self.__T_view_world

    def T_view_proj(self):
        return self.__T_view_proj

    def T_proj_view(self):
        return self.__T_proj_view

    def T_proj_world(self):
        return self.__T_proj_world

    def viewport(self):
        return self.__viewport