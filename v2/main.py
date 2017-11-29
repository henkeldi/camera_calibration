# -*- coding: UTF-8 -*-
import logging as log; import sys
log.basicConfig(stream=sys.stdout, format='[%(levelname)s]: %(message)s', level=log.INFO)
import cyglfw3 as glfw
from OpenGL.GL import *
from OpenGL.GL.NV.bindless_texture import *
import numpy as np
import thread
from threading import Lock
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pickle

from Shader import Shader
from Texture import Texture
from Camera import Camera
from Framebuffer import Framebuffer
from Window import Window
from Light import DirLight
from Material import Material
from cv2 import aruco


print aruco.drawDetectedMarkers
#cv2.aruco.drawMarker()
#dic = cv2.aruco.getPredefinedDictionary(cv2.acuro.DICT_6X6_250)
#print cv2.aruco
print help(cv2)

bridge = CvBridge()
with open('camera_calib.pickle','r') as f:
    ret, mtx, dist, rvecs, tvecs, objpoints, imgpoints = pickle.load(f)

window = Window(640, 480, 16, 'OpenCV+OpenGL')

webcam_image = np.zeros((480,640,3), dtype=np.uint8)

tex = Texture(webcam_image.shape[:2])
fbo = Framebuffer(*webcam_image.shape[:2])
Texture.handlesToUBO([tex.handle, fbo.color_handle])

def callback(data):
    webcam_image[:] = bridge.imgmsg_to_cv2(data, "passthrough")

rospy.init_node('listener', anonymous=True, disable_signals=True)
rospy.Subscriber("/rgb/image", Image, callback, queue_size=1)
thread.start_new_thread( rospy.spin, ())

screen_shader = Shader('shader/screen_shader.vs', 'shader/screen_shader.frag')
color_shader = Shader('shader/color_shader.vs', 'shader/color_shader.frag')
quad_shader = Shader('shader/quad_shader.vs', 'shader/quad_shader.frag')

glPointSize(10)

camera = Camera()
camera.realCameraIntrinsic( 566.0689547, 529.23541743, 
                            312.85730995, 234.45286398, 
                            window.framebuf_width, window.framebuf_height, 0.1, 1000)

T_world_board = np.eye(4, dtype=np.float32)
objp = np.zeros((9*7, 1, 3), np.float32) 
objp[:,:,:2] = np.mgrid[0:7,  0:9].T.reshape(-1,1,2)

def findBoard():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    while True:
        gray = cv2.cvtColor(webcam_image,cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,9), cv2.CALIB_CB_FAST_CHECK)
        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            ret, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
    
            T_world_board[:3,:3] = cv2.Rodrigues(rvecs)[0]
            T_world_board[:3,3] = tvecs[:,0]

thread.start_new_thread( findBoard, ())

T_world_prime = np.array([ [1,   0,  0,  0], [  0,   1,  0,  0], [  0,   0,  -1,  0], [  0,   0,  0,  1]], dtype=np.float32)
T = np.dot( np.dot( camera.T_proj_world(), T_world_prime), T_world_board)
p_World = np.array([1,0,0,1],dtype=np.float32)

def dumpVerticesToPickle(filenames, output_filename='all_obj.pickle'):
    import pyassimp
    vertices = np.zeros((0,), dtype= np.float32)
    indices = np.zeros((len(filenames), 4), dtype=np.uint32)
    
    offset = 0
    for i, file in enumerate(filenames):
        scene = pyassimp.load(file)
        mesh = scene.meshes[0]
        indices[i,:] = np.array( (mesh.vertices.shape[0], 1, offset, 0), dtype=np.uint32)
        vertices = np.hstack( (vertices, np.hstack( (mesh.vertices, mesh.normals) ).reshape(-1) ) ) 
        offset += mesh.vertices.shape[0]
        pyassimp.release(scene)
    with open(output_filename, 'w') as f:
        pickle.dump((vertices, indices), f)

#filenames = ['/home_local/henk_di/datasets/t-less_v2/models_cad/obj_%02d.ply' % i for i in range(1,31)]
#dumpVerticesToPickle(filenames)

def loadVerticesFromPickle(filename='all_obj.pickle'):
    with open(filename, 'r') as f:
        vertices, indices = pickle.load(f)
    vertices *= 0.05
    buf = np.empty(2, dtype=np.uint32)
    glCreateBuffers(len(buf), buf)
    glNamedBufferStorage(buf[0], indices.nbytes, indices, 0)
    glNamedBufferStorage(buf[1], vertices.nbytes, vertices, 0)

    vao = np.empty(1, dtype=np.uint32)
    glCreateVertexArrays(len(vao), vao)
    glVertexArrayAttribFormat(vao, 0, 3, GL_FLOAT, GL_FALSE, 0)
    glVertexArrayAttribFormat(vao, 1, 3, GL_FLOAT, GL_FALSE, 3*4)
    glVertexArrayAttribBinding(vao, 0, 0)
    glVertexArrayAttribBinding(vao, 1, 0)
    glEnableVertexArrayAttrib(vao, 0)
    glEnableVertexArrayAttrib(vao, 1)
    glVertexArrayVertexBuffer(vao, 0, buf[1], 0, 6*4)
    return vao, buf[0]

vao, ibo = loadVerticesFromPickle('all_obj.pickle')
glBindVertexArray(vao)
glBindBuffer(GL_DRAW_INDIRECT_BUFFER, ibo)

################# SCENE #################
dirlight = DirLight( (100, 100, 100), 0.5, 0.5, 1.0 )
material = Material.Pearl
def packSceneData():
    return np.hstack( (camera.T_view_world().transpose().reshape(-1),
                        camera.T_proj_view().transpose().reshape(-1),
                        camera.T_proj_world().transpose().reshape(-1),
                        dirlight.getLightPos(),
                        np.zeros(1, dtype=np.float32),
                        dirlight.getAmbient(),
                        np.zeros(1, dtype=np.float32),
                        dirlight.getDiffuse(),
                        np.zeros(1, dtype=np.float32),
                        dirlight.getSpecular(),
                        np.zeros(1, dtype=np.float32),
                        material.getAmbient(),
                        np.zeros(1, dtype=np.float32),
                        material.getDiffuse(),
                        np.zeros(1, dtype=np.float32),
                        material.getSpecular(),
                        material.getShininess()))

sceneData = packSceneData()
sceneUBO = np.empty(1, dtype=np.uint32)
glCreateBuffers(len(sceneUBO), sceneUBO)
glNamedBufferStorage( sceneUBO[0], sceneData.nbytes, sceneData, GL_DYNAMIC_STORAGE_BIT |
                                                        GL_MAP_WRITE_BIT|
                                                        GL_MAP_PERSISTENT_BIT)
glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sceneUBO[0], 0, sceneData.nbytes)

################# OBJECT #################
def packObjectData():
    mat = np.dot( np.dot(T_world_prime, T_world_board), np.array([[1,0,0,3.5],[0,1,0,4.5],[0,0,-1,0],[0,0,0,1]],dtype=np.float32) )
    return mat.transpose().reshape(-1)

objectData = packObjectData()
objectUBO = np.empty(1, dtype=np.uint32)
glCreateBuffers(len(objectUBO), objectUBO)
glNamedBufferStorage( objectUBO[0], objectData.nbytes, objectData, GL_DYNAMIC_STORAGE_BIT |
                                                        GL_MAP_WRITE_BIT|
                                                        GL_MAP_PERSISTENT_BIT)
glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 1, objectUBO[0], 0, objectData.nbytes)

while not glfw.WindowShouldClose(window.windowID):
    glfw.PollEvents()
    t = glfw.GetTime()

    sceneData = packSceneData()
    glNamedBufferSubData(sceneUBO, 0, sceneData.nbytes, sceneData)

    objectData = packObjectData()
    glNamedBufferSubData(objectUBO, 0, objectData.nbytes, objectData)

    window.update_fps_counter()
    tex.update(webcam_image)

    glViewport(0, 0, window.framebuf_width, window.framebuf_height)
    glClearColor(0.0, 0.0, 0.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    screen_shader.use()
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
    color_shader.use()

    quad_shader.use()
    T = np.dot( np.dot( camera.T_proj_world(), T_world_prime), T_world_board)
    glUniformMatrix4fv(0, 1, GL_TRUE, T)
    glEnable(GL_BLEND)
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUniform4fv(1,1,np.array([0.3,0.4,0.7,0.4], dtype=np.float32))
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4)
    glDisable(GL_BLEND)
    glLineWidth(5.0)
    glUniform4fv(1,1,np.array([0.3,0.4,0.7,1.0], dtype=np.float32))
    glDrawArrays(GL_LINE_LOOP, 0, 4)
    glLineWidth(1.0)

    color_shader.use()
    glEnable(GL_DEPTH_TEST)
    glDrawArraysIndirect(GL_TRIANGLES, ctypes.c_void_p(int(16)*16))
    glDisable(GL_DEPTH_TEST)
    glfw.SwapBuffers(window.windowID)


glfw.Terminate()
rospy.signal_shutdown('Terminated.')