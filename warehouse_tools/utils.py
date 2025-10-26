import numpy as np
from PIL import Image
import os


def value_to_pixel(val):
    """
    Helper function to generate map => colour to matrix values
    """ 

    if val in [0,9]: # trav space
        return 255
    elif val in [-1,1,2]: # non trav space
        return 0
    elif val == 3: # grey
        return 128

    


def generate_map(parent_dir,warehouse_name,pickup_scenario=None,cell_size = 1.0,pixels_per_cell = 10):
    """
    Function is used to generate a .pgm and .yaml file from the 
    provided .txt matrix representation of an empty warehouse

    cell_size: a float value, representing the length of each square cell. Surface area is therefore cell_size * cell_size
    pixels_per_cell: an int value, representing the amount of pixels per cell. Cell then consists of pixels_per_cell*pixels_per_cell pixels

    """

    try:
        # Empty warehouse
        if pickup_scenario == None:
            file_path = os.path.join(parent_dir,warehouse_name,f"{warehouse_name}_empty",f"{warehouse_name}_empty.txt")
            output_path = os.path.join(parent_dir,warehouse_name,f"{warehouse_name}_empty")
        # pickup scenario
        else:
            file_path = os.path.join(parent_dir,warehouse_name,f"scenarios",f"{pickup_scenario}",f"{pickup_scenario}.txt")
            output_path = os.path.join(parent_dir,warehouse_name,f"scenarios",f"{pickup_scenario}")
        warehouse_matrix = np.loadtxt(file_path, dtype=int)
    except Exception as e:
        print(f"Unable to load warehouse from: {file_path}   Error: {e}")
        return 0
    
    # Convert to pixel matrix (color coded)
    pixel_matrix = np.array([[value_to_pixel(v) for v in row] for row in warehouse_matrix])

    # compute the neccesary resolution to keep the desired cell_size
    resolution = cell_size / pixels_per_cell

    scaled_pixels = np.kron(pixel_matrix, np.ones((pixels_per_cell, pixels_per_cell), dtype=np.uint8))

    # save .pgm
    pgm_path = os.path.join(output_path,f"{warehouse_name}.pgm")
    img = Image.fromarray(scaled_pixels.astype(np.uint8), mode="L")
    img.save(pgm_path)

    # save .yaml
    yaml_content = f"""
    image: warehouse_map.pgm
    resolution: {resolution}
    origin: [0.0, 0.0, 0.0]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196
    mode: trinary
    """

    yaml_path = os.path.join(output_path, f"{warehouse_name}.yaml")
    with open(yaml_path, "w") as f:
        f.write(yaml_content)

    return file_path

    
