#!/usr/bin/env python

import cv2
import cv2.aruco as aruco

# Tamano deseado del marcador en metros
desired_marker_size = 0.40

# Densidad de la imagen generada en pixeles por metro
image_density = 100

# Calcula el tamano en pixeles del marcador
marker_size = int(desired_marker_size * image_density)

# ID del marcador ArUco
marker_id = 12

# Crea un diccionario ArUco
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Crea un marcador ArUco con el tamano deseado
marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)

# Guarda el marcador como una imagen
cv2.imwrite('marker12.png', marker_image)
