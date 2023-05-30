import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt

def load_map_from_yaml(file_path):
    with open(file_path, 'r') as file:
        map_data = yaml.safe_load(file)
    return map_data


    # Aquí puedes implementar tu algoritmo A* para planificar la ruta en el mapa
    # Utiliza los datos del mapa (por ejemplo, la matriz de ocupación) para generar la ruta deseada
    # El parámetro 'start' representa la posición de inicio y 'goal' la posición de destino
    
    # Ejemplo: Ruta directa desde el inicio hasta el objetivo
    path = [start, goal]
    return path

def create_occupancy_grid(map_data):
    # Obtener la información necesaria del mapa YAML
    resolution = map_data['resolution']
    origin_x = map_data['origin'][0]
    origin_y = map_data['origin'][1]
    occupied_thresh = map_data['occupied_thresh']
    free_thresh = map_data['free_thresh']

    # Cargar la imagen del mapa PGM
    map_image = cv2.imread('mapa/my_map.pgm', cv2.IMREAD_GRAYSCALE)

    # Calcular el tamaño de la matriz de ocupación
    grid_width = int(map_image.shape[1] * resolution)
    grid_height = int(map_image.shape[0] * resolution)

    # Crear una matriz vacía de ocupación
    occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # Llenar la matriz de ocupación con los datos del mapa
    # Llenar la matriz de ocupación con los datos del mapa
    for y in range(grid_height):
        for x in range(grid_width):
            map_x = int(x * resolution + origin_x)
            map_y = int(y * resolution + origin_y)
            if map_x >= 0 and map_x < map_image.shape[1] and map_y >= 0 and map_y < map_image.shape[0]:
                if map_image[map_y, map_x] > occupied_thresh:
                    occupancy_grid[y, x] = 255
                elif map_image[map_y, map_x] < free_thresh:
                    occupancy_grid[y, x] = 0

    return occupancy_grid

# Cargar el mapa desde el archivo YAML
map_data = {
    'image': 'my_map.pgm',
    'resolution': 0.050000,
    'origin': [-51.224998, -51.224998, 0.000000],
    'occupied_thresh': 0.65,
    'free_thresh': 0.196
}



# Ruta al archivo YAML del mapa
map_file = 'mapa/my_map.yaml'

# Cargar el mapa desde el archivo YAML
map_data = load_map_from_yaml(map_file)

# Crear la matriz de ocupación
occupancy_grid = create_occupancy_grid(map_data)
# Visualización de la matriz de ocupación como una imagen binaria
plt.imshow(occupancy_grid, cmap='gray')
plt.colorbar()
plt.title('Matriz de ocupación')
plt.show()
# Imprimir la matriz de ocupación
# for row in occupancy_grid:
#     print(row)