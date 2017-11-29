# -*- coding: UTF-8 -*-
import numpy as np

class DirLight(object):

	def __init__(self, light_pos, ambient, diffuse, specular):
		self.__light_pos = np.array(light_pos, dtype=np.float32)
		if isinstance(ambient, tuple):
			self.__ambient = np.array(ambient, dtype=np.float32)
		else:
			self.__ambient = np.array(3*[ambient], dtype=np.float32)
		if isinstance(diffuse, tuple):
			self.__diffuse = np.array(diffuse, dtype=np.float32)
		else:
			self.__diffuse = np.array(3*[diffuse], dtype=np.float32)
		if isinstance(specular, tuple):
			self.__specular = np.array(specular, dtype=np.float32)
		else:
			self.__specular = np.array(3*[specular], dtype=np.float32)

	def getLightPos(self):
		return self.__light_pos

	def getAmbient(self):
		return self.__ambient

	def getDiffuse(self):
		return self.__diffuse

	def getSpecular(self):
		return self.__specular