import rasterio
import matplotlib.pyplot as plt
import numpy as np

# Ruta del archivo TIF
tif_file = 'mapa/hector_slam_image.tif'

# Abrir el archivo TIF
# Abrir el archivo TIF
with rasterio.open(tif_file) as src:
    # Leer la banda de la imagen
    band = src.read(1)

    # Definir los umbrales de ocupación y libre
    occupied_thresh = 0.65
    free_thresh = 0.196

    # Crear la matriz de ocupación
    occupancy_grid = np.where(band > occupied_thresh, 1, np.where(band < free_thresh, 0, -1))

    print(occupancy_grid)

    # Visualizar la matriz de ocupación
    plt.imshow(occupancy_grid, cmap='gray')
    plt.colorbar()
    plt.title('Matriz de ocupación')
    plt.show()
