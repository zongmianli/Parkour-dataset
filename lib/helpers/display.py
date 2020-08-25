from pinocchio.utils import *
from pinocchio.explog import exp,log
import pinocchio as se3
import gepetto.corbaserver
import numpy as np
import numpy.linalg as LA


class Viewer:
	'''
	A class implementing a client for the Gepetto-viewer server. The main
    method of the class is 'place', that sets the position/rotation of a
    3D visual object in a scene.
	'''

	def __init__(self, windowName="pinocchio"):
		'''
		This function connects with the Gepetto-viewer server and opens a
        window with the given name. If the window already exists, it is
        kept in the current state. Otherwise, the newly-created
		window is set up with a scene named 'world'.
		'''

		# create the client and connect it with the display server.
		try:
			self.gui = gepetto.corbaserver.Client().gui
		except:
			print("Error while starting the viewer client. ")
			print("Check whether Gepetto-viewer is properly started")

		# Open a window for displaying the model.
		try:
			# If the window already exists, do not do anything.
			window_id = self.gui.getWindowID(windowName)
			print("Warning: window '"+windowName+"' already created.")
			print("The previously created objects will not be destroyed and do not have to be created again.")
		except:
			# Otherwise, create the empty window.
			window_id = self.gui.createWindow(windowName)
			# Start a new "scene" in this window, named "world",
            # with just a floor.
			self.gui.createSceneWithFloor("world")
			self.gui.addSceneToWindow("world", window_id)

		# refresh the layout
		self.ShowFloor(False)
		self.ShowXYZAxis(True)
		self.window_name = windowName
		self.window_id = window_id


	def ShowFloor(self, option=True):
		if option:
			self.gui.setVisibility('world/floor', 'ON')
		else:
			self.gui.setVisibility('world/floor', 'OFF')
		self.gui.refresh()


	def ShowXYZAxis(self, option=True, radius=0.06, size=0.66, color=[0.6, 0.6, 0.6, 0.8]):
		if option:
			self.gui.addXYZaxis('world/xyzAxis', color, radius, size)
		else:
			self.gui.deleteNode('world/xyzAxis', False)
		self.gui.refresh()


	def Place(self, objName, M, refresh=True):
		'''
		This function places (ie changes both translation and rotation)
        of the object names "objName" in place given by the SE3 object "M".
        By default, immediately refresh the layout. If multiple objects
        have to be placed at the same time, do the refresh only at the
        end of the list.
		'''
		self.gui.applyConfiguration(objName, se3ToXYZQUAT(M))
		if refresh:
			self.gui.refresh()


class Visual:
	'''
	Class representing the 3D mesh of body parts to be attached to a joint.
    The class contains:
	- the name of the 3D objects inside Gepetto viewer.
	- the ID of the joint in the kinematic tree to which the body is attached.
	- the placement of the body with respect to the joint frame.
	This class is only used in the list Limb.visuals (see below).
	'''
	def __init__(self, name, placement, parent_id=None, parent_type=None):
		self.name = name
		self.placement = placement # placement of the body wrt joint, i.e. jointMbody
		if parent_id is not None:
			self.parent_id = parent_id # ID of the parent joint or frame
		if parent_type is not None:
			self.parent_type = parent_type # 'j' (joint) or 'f' (frame)

	def Place(self, viewer, oMi):
		oMbody = oMi*self.placement
		viewer.Place(self.name, oMbody, False)


class PointCloud:
	'''
	This class helps to display a point cloud (either in 2D or in 3D).
	'''

	def __init__(self, cloud_name,
              gui,
              names,
              colors=None,
			  opacity=0.8,
              size=0.03):
		self.cloud_name = cloud_name
		self.gui = gui
		self.num_points = len(names)
		self.names = [cloud_name+'_'+names[i] for i in range(self.num_points)]
		if colors is None or len(colors) != self.num_points:
			self.colors = [[1., 1., 1., opacity] for i in range(self.num_points)]
		else:
			if max(colors) > 1:
				colors = [[i/255.0 for i in c] for c in colors]
			self.colors = [colors[i]+[opacity] for i in range(self.num_points)]
		self.size = size
		self.CreateVisuals()


	def CreateVisuals(self):
		size = self.size
		for i in range(self.num_points):
			name = self.names[i]
			color = self.colors[i]
			self.gui.addSphere('world/'+name, size, color)


	def DeleteVisuals(self):
		for name in self.names:
			self.gui.deleteNode('world/'+name, False)


	def Display(self, q, refresh_gui=False):
		for i in range(self.num_points):
			name = self.names[i]
			placement = se3.SE3(eye(3), q[(3*i):(3*i+3),0])
			self.gui.applyConfiguration("world/"+name, se3ToXYZQUAT(placement))
		if refresh_gui:
			self.gui.refresh()


	def Display2d(self, q, refresh_gui=False):
		translation = np.matrix(np.zeros(3)).T
		for i in range(self.num_points):
			name = self.names[i]
			translation[:2, 0] = q[(3*i):(3*i+2),0]
			placement = se3.SE3(eye(3), translation)
			self.gui.applyConfiguration("world/"+name, se3ToXYZQUAT(placement))
		if refresh_gui:
			self.gui.refresh()
