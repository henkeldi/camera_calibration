# -*- coding: UTF-8 -*-
import logging as log; import sys
log.basicConfig(stream=sys.stdout, format='[%(levelname)s]: %(message)s', level=log.INFO)
import cyglfw3 as glfw
from OpenGL.GL import *
from OpenGL.GL.NV.bindless_texture import *
import numpy as np
from Shader import Shader
from Texture import Texture
from Camera import Camera
from Light import DirLight
import thread
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
bridge = CvBridge()

window_width, window_height = 640, 480
samples = 16
window_title = 'OpenCV+OpenGL'
assert glfw.Init(), 'Glfw Init failed!'
glfw.WindowHint(glfw.SAMPLES, samples)
window = glfw.CreateWindow(window_width, window_height, window_title, None)
assert window, 'Could not create Window!'
glfw.MakeContextCurrent(window)
assert glInitBindlessTextureNV(), 'Bindless Textures not supported!'
framebuf_width, framebuf_height = glfw.GetFramebufferSize(window)
def framebuffer_size_callback(window, w, h):
    global framebuf_width, framebuf_height
    framebuf_width, framebuf_height = w, h
glfw.SetFramebufferSizeCallback(window, framebuffer_size_callback)
def key_callback(window, key, scancode, action, mode):
    if action == glfw.PRESS:
        if key == glfw.KEY_ESCAPE:
            glfw.SetWindowShouldClose(window, True)
glfw.SetKeyCallback(window, key_callback)

previous_second = glfw.GetTime(); frame_count = 0.0
def update_fps_counter(window):
    global previous_second, frame_count
    current_second = glfw.GetTime()
    elapsed_seconds = current_second - previous_second
    if elapsed_seconds > 1.0:
        previous_second = current_second
        fps = float(frame_count) / float(elapsed_seconds)
        glfw.SetWindowTitle(window, '%s @ FPS: %.2f' % (window_title, fps))
        frame_count = 0.0
    frame_count += 1.0

glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE)
glDisable(GL_STENCIL_TEST)
glDisable(GL_BLEND)
glDisable(GL_CULL_FACE)
glClearColor(0.0, 0.0, 0.0, 0.0)

screen_shader = Shader('shader/screen_shader.vs', 'shader/screen_shader.frag')
screen_shader.compile(); screen_shader.use()

color_shader = Shader('shader/color_shader.vs', 'shader/color_shader.frag')
color_shader.compile(); color_shader.use()

raw_im = np.zeros((480,640,3), dtype=np.uint8)

def createFramebuffer(W, H, samples):
    tex = np.empty(3, dtype=np.uint32)
    if samples == 1:
        glCreateTextures(GL_TEXTURE_2D, len(tex), tex)
        glTextureStorage2D(tex[0], 1, GL_RGB8, W, H)
        glTextureParameteri(tex[0], GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTextureParameteri(tex[0], GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTextureStorage2D(tex[1], 1, GL_DEPTH_COMPONENT32F, W, H)
        glTextureParameteri(tex[1], GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTextureParameteri(tex[1], GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    else:
        glCreateTextures(GL_TEXTURE_2D_MULTISAMPLE, len(tex), tex)
        glTextureStorage2DMultisample(tex[0], samples, GL_RGB8, W, H, GL_TRUE)
        glTextureStorage2DMultisample(tex[1], samples, GL_DEPTH_COMPONENT32F, W, H, GL_TRUE)

        glTextureStorage2DMultisample(tex[2], samples, GL_R32F, W, H, GL_TRUE)

    fbo = np.empty(1, dtype=np.uint32)
    glCreateFramebuffers(len(fbo), fbo)
    
    glNamedFramebufferTexture(fbo, GL_COLOR_ATTACHMENT0, tex[0], 0)
    glNamedFramebufferTexture(fbo, GL_DEPTH_ATTACHMENT, tex[1], 0)
    
    color_handle = glGetTextureHandleNV(tex[0])
    glMakeTextureHandleResidentNV(color_handle)
    depth_handle = glGetTextureHandleNV(tex[1])
    glMakeTextureHandleResidentNV(depth_handle)

    assert glCheckNamedFramebufferStatus(fbo, GL_FRAMEBUFFER)\
                    == GL_FRAMEBUFFER_COMPLETE, 'Framebuffer not complete'
    return fbo, color_handle

fbo, color_handle  = createFramebuffer(640, 480, 1)

tex = Texture(raw_im.shape[:2])

handles = np.empty(2, dtype=np.uint64)
handles[0] = tex.getHandle()
handles[1] = color_handle

ubo = np.empty(1, dtype=np.uint32)
glCreateBuffers(len(ubo), ubo)
glNamedBufferStorage( ubo[0], handles.nbytes, handles, 0)
glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 2, ubo[0], 0, handles.nbytes)

def callback(data):
    try:
        #lock.acquire()
        dat = bridge.imgmsg_to_cv2(data, "passthrough")
        raw_im[:] = dat
        #lock.release()
    except CvBridgeError as e:
      print(e)

def ros_thread():
    rospy.spin()    

rospy.init_node('listener', anonymous=True, disable_signals=True)
rospy.Subscriber("/rgb/image", Image, callback, queue_size=1)
thread.start_new_thread( ros_thread, ())

camera = Camera()
camera.realCameraIntrinsic( 566.0689547, 529.23541743, 
                            312.85730995, 234.45286398, 
                            raw_im.shape[1], raw_im.shape[0], 0.1, 1000)

dirlight = DirLight( (100, 100, 100), 0.5, 0.5, 1.0 )

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
                        np.zeros(1, dtype=np.float32)))

sceneData = packSceneData()
sceneUBO = np.empty(1, dtype=np.uint32)
glCreateBuffers(len(sceneUBO), sceneUBO)
glNamedBufferStorage( sceneUBO[0], sceneData.nbytes, sceneData, GL_DYNAMIC_STORAGE_BIT |
                                                        GL_MAP_WRITE_BIT|
                                                        GL_MAP_PERSISTENT_BIT)
glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sceneUBO[0], 0, sceneData.nbytes)

mat = [  [  6.78451508e-02,   9.97672439e-01,   6.83312537e-03,  -3.45446110e+00],
         [ -8.30655158e-01,   5.26910387e-02,   5.54288447e-01,   2.10924363e+00],
         [  5.52638292e-01,  -4.32817526e-02,   8.32296610e-01,   1.10088863e+01],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]
mat = np.array(mat, dtype=np.float32)
def packObjectData():
    return np.linalg.inv(mat).transpose().reshape(-1)

objectData = packObjectData()
objectUBO = np.empty(1, dtype=np.uint32)
glCreateBuffers(len(objectUBO), objectUBO)
glNamedBufferStorage( objectUBO[0], objectData.nbytes, objectData, GL_DYNAMIC_STORAGE_BIT |
                                                        GL_MAP_WRITE_BIT|
                                                        GL_MAP_PERSISTENT_BIT)
glBindBufferRange(GL_SHADER_STORAGE_BUFFER, 1, objectUBO[0], 0, objectData.nbytes)

glPointSize(10)


print np.linalg.inv(mat)[:3,3]

buf = np.empty(16 , np.float32)
glGetNamedBufferSubData(objectUBO, 0, objectData.nbytes, buf)
#print buf

print np.dot( np.dot(camera.T_proj_view(), camera.T_view_world()) , np.array([[0],[0],[-1],[1]]) )

A = np.empty(16*3, np.float32)
glGetNamedBufferSubData(sceneUBO, 0, A.nbytes, A)
for i in range(3):
    print A[16*i:16*i+16].reshape((4,4)).transpose()

while not glfw.WindowShouldClose(window):
    glfw.PollEvents()
    
    t = glfw.GetTime()
    update_fps_counter(window)
    
    tex.update(raw_im)
    #sceneData = packSceneData()
    #glNamedBufferSubData(sceneUBO, 0, sceneData.nbytes, sceneData)

    objectData = packObjectData()
    glNamedBufferSubData(objectUBO, 0, objectData.nbytes, objectData)

    glBindFramebuffer(GL_FRAMEBUFFER, fbo)
    glViewport(0,0,framebuf_width,framebuf_height)
    glClearColor(0.0,0.0,0.0,0.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    color_shader.use()
    glDrawArrays(GL_POINTS, 0, 1)
    glBindFramebuffer(GL_FRAMEBUFFER, 0)

    glBindFramebuffer(GL_FRAMEBUFFER, 0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glViewport(0,0,framebuf_width,framebuf_height)
    glDisable(GL_DEPTH_TEST)
    screen_shader.use()
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 6)
    glUniform1i(0, 1)
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 6)
    glUniform1i(0, 0)
    glfw.SwapBuffers(window)

glfw.Terminate()
rospy.signal_shutdown('Terminated.')