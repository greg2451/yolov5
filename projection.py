from math import prod
from re import A
from matplotlib.pyplot import plot

from scipy.misc import central_diff_weights
from yolo_ros.msg import ContactsList
from scipy.spatial.transform import Rotation as R
from numpy import sqrt, degrees, arctan, array, cos, sin, tan, radians


def new_point_offset(lat, lon, azimuth, distance):
    """
    Renvoie les coordonnées d'un nouveau point sachant la distance de celui-ci à un point aux coordonnées connues, ainsi que son relèvement.

    Latitude et longitude sont en degrés, azimuth en radians et distance en mètres.
    Il s'agit d'une formule approchée fonctionnant bien pour les distances petites devant 111 km (distance parcourue par 1 degré de latitude)
    """
    dLat = distance * cos(azimuth) / 111111
    dLon = distance * sin(azimuth) * cos(radians(lat)) / 111111
    return (lat + dLat, lon + dLon)


class Projecteur:
	"""
	Classe qui permet la projection d'un point sur un écran d'une caméra, aux angles de relèvement depuis le bateau (Yaw & Pitch).
	
	Elle effectue les actions suivantes :
		- Rotation de l'écran pour compenser le roulis relatif à la caméra (stabilisation).
		- Association du pixel à un couple d'angle (yaw,pitch) relatif à la caméra.
		- Ajout de l'offset (yaw,pitch) afin d'obtenir les angles de relèvement dans le référentiel de la CI.
		- Projection en lattitude, longitude après mise en parralèle avec la position du bateau
	""" 
	
	def __init__(
		self,
        f = 1020,
        b = -0.00022355714835994704,
        c = -9.927438097191326e-11,
        screen_dim = [1280,720],
        camera_orientation = [[60*i, -22.4, 0] for i in range(6)],
        camera_height = 1.8,
    ):
		"""
		f : Focale de la lentille (= distance lentille-écran dans le pin-hole model, à régler par l'expérience) 
		b : Terme de correction quadratique (calcul via le logiciel de correction grâce à l'horizon)
		c : Terme de correction quartique (calcul via le logiciel de correction grâce à l'horizon)
		screen_dim : dimensions de l'image produite par la caméra (en pixels), le premier axe correspond à la dimension verticale (convention matricielle)
		camera_orientation : la liste des attitudes des caméras, en degrés et convention NED
		"""
		self.f = f 
		self.b = b 
		self.c = c 
		self.screen_dim = screen_dim
		self.camera_height = camera_height
		self.camera_orientation_initial = R.from_euler(
			'zyx',
			camera_orientation,
			degrees=True,
			# NED conventions
		)

		# Initialization.
		self.boat_orientation = R.from_euler(
			'zyx',
			[0, 0, 0],
			degrees=True,
		)
		
	def update(
		self,
        QuatMsg,
        NavMsg,
    ):
		"""Met à jour la position du bateau et celles des caméras dans le référentiel de la CI.

		Args:
			boat_quaternion (array): Le quaternion représentant les coordonnées angulaires du bateau.
		"""
		
		self.boat_orientation = R.from_quat([getattr(QuatMsg.quaternion,attr) for attr in ['x','y','z','w']])
		self.camera_orientation = self.camera_orientation_initial * self.boat_orientation 
		self.boat_position = NavMsg

	def __call__(
		self,
		xywh,
		camera_id,
		obj_id,
		rosmsg = False
    ):
		"""
		Etant donné les coordonnées d'un point en convention OpenCV, renvoie le gisement de ce point (en degrés) et la position de ce point. 
		""" 

		# Get coordinates of center in pixel.
		x, y, w, h = xywh
		x = (x - 0.5) * self.screen_dim[0]
		y = (0.5 - y) * self.screen_dim[1] # Incohérence de signe entre convention OpenCV et NED sinon.
		w = w * self.screen_dim[0]
		h = h * self.screen_dim[1]
		

		# Récupération du roulis relatif à la caméra.
		# Rotation associée à l'angle -roll pour stabiliser la caméra (d'où le signe moins en bas).
		yaw, pitch, roll = self.camera_orientation[camera_id].as_euler('zyx')
		roll_rotation_matrix = array(
			( (cos(roll), sin(roll)),
			(-sin(roll),  cos(roll)) )
		)

		# Application de la rotation de l'écran.
		(x,y),(w,h) = (roll_rotation_matrix @ array(((x,y),(w,h))).T).T

	
		# Inversion du DL f(r) = r + br**2 + cr**4 
		# donne : f-1(r) = r -br**2 + 2b**2 * r**3 + (-c -5b**3)r**4 
		ri2 = x**2 + y**2 
		ri = sqrt(ri2)
		rf = ri - self.b*ri2 + 2*(self.b**2)*ri*ri2 + (-self.c - 5*self.b**3)*ri2**2 

		# Calcul de conversion pixel vers angle.
		def pixel2angle(pixel):
			return arctan(pixel*(rf/(ri + 10e-9))/self.f)

		# Relèvement du point le plus bas de la détection
		elevation = pixel2angle(y-h/2) + pitch # Eventuellement rajouter un - par orientation.
		azimuth = pixel2angle(x) + yaw

		# Calcul de la largeur et hauteur angulaire de la cible.
		angular_width = pixel2angle(x+w/2) - pixel2angle(x-w/2)
		angular_height = pixel2angle(y+h/2) - pixel2angle(y-h/2) # Convention OpenCV, (0,0) est le coin supérieur gauche.

		# Calcul de la distance en considérant son altitude nulle (dépendance extrêmement sensible en l'élévation).
		distance = (self.camera_height - self.boat_position.altitude) / tan(elevation)

		# Déduction de la hauteur et largeur du plus petit rectangle couvrant la cible.
		target_height = 2 * distance * tan(angular_height/2)
		target_width = 2 * distance * tan(angular_width/2)
		
		# Déduction des coordonnées GPS de la cible.
		lat_target, lon_target = new_point_offset(
			self.boat_position.latitude,
			self.boat_position.longitude,
			azimuth,
			distance,
		)
		
		if rosmsg :
			message = ContactsList(
				id = int(obj_id),
				status = 1,
				latitude = lat_target,
				longitude = lon_target,
				speed = 0,
				size = target_height * target_width
			)

			return message
		
		else :
			return lat_target, lon_target, (target_width, target_height), distance, elevation, azimuth

def plot_projection_results(boat_rotation = [0,0,15]):
    
	from sbg_driver.msg import SbgEkfQuat, SbgEkfNav
	import seaborn as sns
	from tqdm import tqdm
	import numpy as np
	import matplotlib.pyplot as plt

	projecteur = Projecteur()
	quaternion = R.from_euler('zyx', boat_rotation, degrees=True)
	quat_dict = dict(zip([*'xyzw'],quaternion.as_quat()))

	QuatMsg = SbgEkfQuat()
	_ = [setattr(QuatMsg.quaternion, key, value) for key,value in quat_dict.items()]
	NavMsg = SbgEkfNav()

	projecteur.update(QuatMsg, NavMsg)

	XYWH = np.array([[((x-640), (y-360), 0, 0) for x in range(1,1281)] for y in range(1,721)])

	X = XYWH[:,:,0]
	Y = XYWH[:,:,1]

	yaw, pitch, roll = projecteur.camera_orientation[0].as_euler('zyx')
	print(projecteur.camera_orientation[0].as_euler('zyx',degrees=True))

	roll_rotation_matrix = array(
		((cos(roll), sin(roll)),
		(-sin(roll),  cos(roll)) )
	)

	# Application de la rotation de l'écran.

	azimuth = np.zeros((720,1280))
	elevation = np.zeros((720,1280))
	for xywh1 in tqdm(XYWH):
		for xywh in xywh1:
			x_i,y_i,w,h = xywh
			x,y =roll_rotation_matrix @ array(((x_i,y_i)))
			ri2 = x**2 + y**2 
			ri = sqrt(ri2)
			rf = ri - projecteur.b*ri2 + 2*(projecteur.b**2)*ri*ri2 + (-projecteur.c - 5*projecteur.b**3)*ri2**2 	
			azimuth[y_i+359,x_i+639] = np.degrees(np.arctan(x*(rf/(ri + 10e-6))/projecteur.f) + yaw)
			elevation[y_i+359,x_i+639] = np.degrees(np.arctan(-y*(rf/(ri + 10e-6))/projecteur.f) + pitch)

	fig = plt.figure(figsize=plt.figaspect(9/16))
	fig.suptitle(f"Projection espace des pixels vers angles de relèvement avec l'angle de rotation (zyx) : {projecteur.camera_orientation[0].as_euler('zyx',degrees=True)}")

	ax = fig.add_subplot(2, 2, 1, projection='3d')
	# ax = plt.axes(projection='3d')
	az = ax.plot_surface(X, Y, azimuth, rstride=20, cstride=20, cmap='viridis')
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('Azimuth (degrés)')
	ax.set_title('Azimuth vs coordonnées des pixels')
	ax.axis()
	fig.colorbar(az, shrink=0.5, aspect=10)

	ax = fig.add_subplot(2, 2, 3, projection='3d')
	el = ax.plot_surface(X, Y, elevation, rstride=20, cstride=20, cmap='plasma')
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('Élévation (degrés)')
	ax.set_title('Élévation vs coordonnées des pixels')
	fig.colorbar(el, shrink=0.5, aspect=10)

	ax = fig.add_subplot(2,2, 2)
	sns.heatmap(azimuth, ax=ax, cmap='viridis')
	ax.set_xlabel('x')
	ax.set_ylabel('y')

	ax = fig.add_subplot(2, 2, 4)
	sns.heatmap(elevation, ax=ax, cmap='plasma')
	ax.set_xlabel('x')
	ax.set_ylabel('y')


	plt.show()

if __name__ == '__main__':
	plot_projection_results([90,0.2,2])