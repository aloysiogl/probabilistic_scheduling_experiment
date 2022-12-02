import yaml

import numpy as np
import cv2

def pixel_coordnates_to_metric_coordinates(x, y, map_resolution, map_origin):
    """
    Convert pixel coordinates to metric coordinates
    :param x: x pixel coordinate
    :param y: y pixel coordinate
    :param map_resolution: map resolution
    :param map_origin: map origin
    :return: metric coordinates
    """
    return (x * map_resolution) + map_origin[0], (y * map_resolution) + map_origin[1]

def main():
    width = 4
    inner_radius = 20
    margin = 2
    resolution = 5/40 # meters per pixel
    half_side = inner_radius + margin + width
    map_origin = (-half_side, -half_side)
    # map_origin = (0, 0)

    image_size = int(half_side * 2 / resolution)

    image = np.zeros((image_size, image_size, 3), np.uint8)

    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            x, y = pixel_coordnates_to_metric_coordinates(i, j, resolution, (-half_side, -half_side))
            if x**2 + y**2 < (inner_radius + width)**2 and x**2 + y**2 > (inner_radius)**2:
                image[i, j] = (255, 255, 255)
    img_bw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('map.pgm', img_bw)
    cv2.imwrite('map.png', img_bw)
    yaml = open(f'map' + '.yaml', 'w')
    yaml.write('image: map' + '.pgm\n')
    yaml.write(f'resolution: {resolution}\n')
    yaml.write('origin: [' + str(map_origin[0]) + ',' + str(map_origin[1]) + ', 0.000000]\n')
    yaml.write('negate: 0\noccupied_thresh: 0.45\nfree_thresh: 0.196')
    yaml.close()

if __name__ == '__main__':
    main()