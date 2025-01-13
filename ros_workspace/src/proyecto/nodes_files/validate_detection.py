#!/usr/bin/python3

from collections import Counter
import numpy as np

def detection_validation(all_data):
    observations = []
    for data in all_data:
        piece_types = [piece_data[2] for piece_data in data]
        observations.append(piece_types)

    conteo = Counter(tuple(sublista) for sublista in observations)
    combinacion_mas_comun, _ = conteo.most_common(1)[0]

    # Filtrar detecciones que coinciden con la combinación más común
    posiciones = [i for i, sublista in enumerate(observations) if tuple(sublista) == combinacion_mas_comun]
    selected_data = [all_data[i] for i in posiciones]

    print(f'\nNúmero de detecciones que coinciden: {len(selected_data)}/{len(all_data)}')

    # Verificar número de piezas y calcular promedios
    num_piezas = len(combinacion_mas_comun)
    print(f'Número de piezas en cada detección seleccionada para promediar: {num_piezas}')
    print(f'Las piezas detectadas son de tipo: {selected_data[0][0][2]}')

    counter = 0
    final_data = []

    while counter < num_piezas:
    
        x_coords = []
        y_coords = []
        angles = []
    
        for i in range(len(selected_data)):
            x_coords.append(selected_data[i][counter][0][0])
            y_coords.append(selected_data[i][counter][0][1])
            angles.append(selected_data[i][counter][1])
            piece_type = selected_data[i][counter][2]
    
        if piece_type == 'triangulos' or piece_type == 'pentagonos':
            x_avg = x_coords[0]
            y_avg = y_coords[0]
            coords_avg = (x_avg, y_avg)
            angle_avg = angles[0]
        else:
            x_avg = np.mean(x_coords)
            y_avg = np.mean(y_coords)
            coords_avg = (x_avg, y_avg)
            angle_avg = np.mean(angles)
    
        final_data.append([coords_avg, angle_avg, piece_type])
        counter += 1
    
    print(f'Datos promediados: {final_data}\n')
    return final_data
