# -*- coding: UTF-8 -*-

import logging as log
from OpenGL.GL import *
import os

class Shader(object):

    shaderFolder = ''
    activeShader = None

    def __init__(self, *shaderPaths):
        self.__shader = []
        endings = [ s[s.rindex('.')+1:] for s in shaderPaths]
        for end, shType in zip(['vs', 'tcs', 'tes', 'gs', 'frag', 'cs'],
                        [GL_VERTEX_SHADER, GL_TESS_CONTROL_SHADER, GL_TESS_EVALUATION_SHADER, GL_GEOMETRY_SHADER, GL_FRAGMENT_SHADER, GL_COMPUTE_SHADER]):
            try:
                shader = shaderPaths[endings.index(end)]
                self.__shader.append((shader,shType))
            except ValueError as e:
                pass

    def compile(self, varyings=None):
        log.info('Compiling shader.')
        this_dir = os.path.dirname(os.path.realpath(__file__))
        prefix = '' if Shader.shaderFolder == '' else this_dir + os.sep + Shader.shaderFolder + os.sep
        shaderIDs = []

        for shader in self.__shader:
            code = Shader.__readFile__(prefix + shader[0])
            shaderID = Shader.__createShader__(shader[0], shader[1], code)
            shaderIDs.append(shaderID)

        self.__program = glCreateProgram()
        for shaderID in shaderIDs:
            glAttachShader(self.__program, shaderID)

        if varyings is not None:            
            LP_c_char = ctypes.POINTER(ctypes.c_char)

            argv = (LP_c_char * len(varyings))()
            for i, arg in enumerate(varyings):
                enc_arg = arg.encode('utf-8')
                argv[i] = ctypes.create_string_buffer(enc_arg) 

            glTransformFeedbackVaryings(self.__program, 2, argv, GL_SEPARATE_ATTRIBS)

        glLinkProgram(self.__program)
        if not glGetProgramiv(self.__program, GL_LINK_STATUS):
            log.error(glGetProgramInfoLog(self.__program))
            log.critical('Shader linking failed!')
            exit(-1)
        else:
            log.info('Shader linked.')

        for shaderID in shaderIDs:
            glDeleteShader(shaderID)

    @staticmethod
    def __readFile__(path):
        f = None
        try:
            f = open(path, 'r')
            data = f.read()
            f.close()
            return data
        except IOError as e:
            log.critical("\"{2}\": I/O error({0}): {1}".format(e.errno, e.strerror, path))
            exit(-1)
        except:
            log.critical("Unexpected error: ", sys.exc_info()[0])
            exit(-1)


    @staticmethod
    def __createShader__(shaderPath, shaderType, shaderCode):
        shader = glCreateShader(shaderType)
        glShaderSource(shader, shaderCode)
        glCompileShader(shader)
        if not glGetShaderiv(shader, GL_COMPILE_STATUS):
            log.error(glGetShaderInfoLog(shader))
            log.critical('[%s]: Shader compilation failed!' % shaderPath)
            exit(-1)
        else:
            log.info('Shader compiled (%s).', shaderPath)
        return shader

    def printInfo(self):
        print glGetProgramInterfaceiv(self.__program, GL_PROGRAM_OUTPUT, GL_ACTIVE_RESOURCES)
        for i in range(glGetProgramInterfaceiv(self.__program, GL_PROGRAM_OUTPUT, GL_ACTIVE_RESOURCES)):
            name = glGetProgramResourceName(self.__program, GL_PROGRAM_OUTPUT, i, 0)
            params =  glGetProgramResourceiv(self.__program, GL_PROGRAM_OUTPUT, i, 2, [GL_TYPE, GL_LOCATION], 2, 0)
            print 'Index %d: %s %s @ location %s' % (i, params[0], name, params[1])

    def delete(self):
        glDeleteProgram(self.__program)

    def use(self):
        glUseProgram(self.__program)
        Shader.activeShader = self

    def getProgram(self):
        return self.__program