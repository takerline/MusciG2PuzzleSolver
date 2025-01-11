import cv2
import cv2.aruco as aruco
import numpy as np
import pickle
from numpy import arctan, argmin
from math import pi

class PieceDetection:
    def __init__(self, calibration_file='new_calibration.pkl'):
        self.calibration_file = calibration_file

    def correct_image(self, img):
        """
        Elimina la distorsión de una imagen dada utilizando los parámetros de calibración.
        """
        with open(self.calibration_file, 'rb') as f:
            cameraMatrix, dist = pickle.load(f)

        h, w = img.shape[:2]
        newCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w, h), 1, (w, h))
        calibrated_img = cv2.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

        return calibrated_img

    def pixel_to_distance(self, img, dist_cm=5.0):
        """
        Obtiene el factor de conversión de distancia en píxeles a cm basado en un marcador ArUco.

        Args:
            img: Imagen ya calibrada donde se buscará el marcador ArUco.
            dist_cm: Distancia real conocida en cm del marcador.

        Returns:
            bottom_left: Coordenadas de la esquina inferior izquierda del marcador,
            utilizada como origen de coordenadas.
            pix_to_cm_x: Factor de conversión de píxeles a cm en el eje X.
            pix_to_cm_y: Factor de conversión de píxeles a cm en el eje Y.
        """
        # Crear el diccionario y los parámetros para detectar marcadores ArUco
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        # Detectar marcadores en la imagen
        markerCorners, markerIds, _ = detector.detectMarkers(img)

        if markerIds is not None:
            for _, corners in zip(markerIds.flatten(), markerCorners):
                # Extraer las esquinas del marcador
                top_left = tuple(corners[0][0].astype(int))
                top_right = tuple(corners[0][1].astype(int))
                bottom_right = tuple(corners[0][2].astype(int))
                bottom_left = tuple(corners[0][3].astype(int))

                dist_pix_x = bottom_right[0] - bottom_left[0]
                dist_pix_y = bottom_left[1] - top_left[1]

                pix_to_cm_x = dist_cm / dist_pix_x
                pix_to_cm_y = dist_cm / dist_pix_y

                self.bottom_left = bottom_left

                print('### Resultados de conversión de píxeles a cm con ArUco:')
                print(f'A {dist_pix_x} píxeles en X les corresponden {dist_cm} cm')
                print(f'A {dist_pix_y} píxeles en Y les corresponden {dist_cm} cm\n')

                return pix_to_cm_x, pix_to_cm_y

        else:
            print("No se detectaron marcadores ArUco.")
            return None, None
        
    def get_piece_contours(self, img):
        """
        Obtiene los contornos de las piezas detectadas en la imagen calibrada.

        Args:
            img: Imagen menconada.

        Returns:
            pieces_contours: Lista de contornos detectados.
        """

        # Pasar de formato BGR a Lab.
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)

        # Definir umbrales para el espacio Lab.
        # Los valores se han ajustado para detectar piezas y reducir sombras.
        bajo = np.array([0, 60, 110]) 
        alto = np.array([170, 145, 145]) 

        # Crear máscara basada en los umbrales.
        mascara_prueba = cv2.inRange(img_lab, bajo, alto)

        # Convertir la máscara a escala de grises.
        img_gray = cv2.cvtColor(cv2.merge([mascara_prueba] * 3), cv2.COLOR_BGR2GRAY)

        # Aplicar suavizado para eliminar ruido.
        blur = cv2.GaussianBlur(img_gray, (11, 11), 0) 

        # Detectar bordes con Canny.
        mask_canny = cv2.Canny(blur, 50, 140)

        # Dilatación + erosión para cerrar pequeños huecos en los bordes detectados.
        kernel = np.ones((9, 9), np.uint8)
        bordes_cerrados = cv2.morphologyEx(mask_canny, cv2.MORPH_CLOSE, kernel)

        # Búsqueda de contornos.
        contours, hierarchy = cv2.findContours(bordes_cerrados, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Vamos a filtrar por tamaño de contornos para no considerar el ArUco ni
        # ninguna otra cosa que no sean piezas.
        pieces_contours = []

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            min_area_piece = 300
            max_area = 20000

            if (area > min_area_piece) and (area < max_area):
                # Tenemos en cuenta el hierarchy para quedarnos con el contorno externo
                # de cada pieza, que es el que nos interesa para que el robot la recoja.
                if hierarchy[0][i][3] == -1:
                    pieces_contours.append(contours[i])

        print('### Detección de piezas:')
        print(f'Se han detectado {len(pieces_contours)} piezas.\n')

        return pieces_contours
    
    def PolygonDetection(self, pieces_contours) -> dict:
        """
        Se realiza la clasificación por forma de la piezas anteriores.

        Args:
            pieces_contours: Lista de contornos detectados.

        Returns:
            dict_pieces: Diccionario en el que se organizan contornos por forma.
            dict_vertices: Diccionario en el que por cada forma se crea una lista 
            con los vértices de cada figura en la imagen.
        """

        # Listas para clasificar por forma.
        triangulos = []
        cuadrados = []
        rectangulos = []
        pentagonos = []
        circulos = []

        # Listas para guardar vértices de cada pieza según forma.
        vertices_triangulos = []
        vertices_cuadrados = []
        vertices_rectangulos = []
        vertices_pentagonos = []
        vertices_circulos = []

        print('### Clasificación de piezas:')

        for contorno in pieces_contours:

            # Calcular el perímetro y aproximar la forma.
            perimetro = cv2.arcLength(contorno, True)
            # Obtener lista con los vértices de cada contorno.
            aproximacion = cv2.approxPolyDP(contorno, 0.064 * perimetro, True) # 0.066 antes y 0.072
            # Calculamos también el área.
            area = cv2.contourArea(contorno)
            # Calculamos circularidad para la clasificación.
            circularidad = (4 * 3.1416 * area) / (perimetro * perimetro)

            # Si la circularidad es alta se trata de un círculo.
            if circularidad > 0.85: 
                circulos.append(contorno)
                vertices_circulos.append(aproximacion)
                print('Círculo')

            # Si no clasificar según cantidad de lados.
            else:
                if len(aproximacion) == 5:
                    pentagonos.append(contorno)
                    vertices_pentagonos.append(aproximacion)
                    print('Pentágono')

                elif len(aproximacion) == 3:
                    triangulos.append(contorno)
                    vertices_triangulos.append(aproximacion)
                    print('Triángulo')

                # Si hay 4 lados puede tratarse de un cuadrado o un rectángulo.
                elif len(aproximacion) == 4:
                    rect = cv2.minAreaRect(contorno)
                    (centro, dimensiones, angulo) = rect
                    ancho, alto = dimensiones

                    # Prevenir errores de orientación (asegurar ancho < alto).
                    if ancho < alto:
                        ancho, alto = alto, ancho

                    # Nos fijamos en relación altura vs anchura para hacer clasificación.
                    relacion = alto / ancho

                    if 0.90 <= relacion <= 1.1:
                        cuadrados.append(contorno)
                        vertices_cuadrados.append(aproximacion)
                        print('Cuadrado')
                    else:
                        rectangulos.append(contorno)
                        vertices_rectangulos.append(aproximacion)
                        print('Rectángulo')

        dict_pieces = {
            'triangulos': triangulos,
            'cuadrados': cuadrados,
            'rectangulos': rectangulos,
            'pentagonos': pentagonos,
            'circulos': circulos
        }

        dict_vertices = {
            'triangulos': vertices_triangulos,
            'cuadrados': vertices_cuadrados,
            'rectangulos': vertices_rectangulos,
            'pentagonos': vertices_pentagonos,
            'circulos': vertices_circulos      
        }
        
        print('')

        return dict_pieces, dict_vertices
    
    def get_order(self, pieces):
        """
        Se obtiene orden necesario para organizar piezas por tamaño.

        Args:
            pieces: Todas las piezas de un tipo determinado.
        """

        self.indices_ordenados = sorted(range(len(pieces)), key=lambda i: cv2.contourArea(pieces[i]), reverse=True)
        
    def get_data(self, pieces):
        """
        Para piezas de un tipo determinado obtiene información relevante.

        Args:
            pieces: Todas las piezas de un tipo determinado.

        Returns:
            piezas_data: Información sobre coordenadas, rotación y forma de cada pieza.
            piezas_rect: Rectangulo que contiene a cada pieza que se utilizará para
            dibujar el resultado de la detección.
        """

        # Para este tipo de piezas, se obtiene el orden correspondiente.
        self.get_order(pieces)
        # Y se ordena.
        pieces = [pieces[i] for i in self.indices_ordenados]

        piezas_data = []
        piezas_rect = []

        for pieza in pieces:
            # Calculamos el rectángulo de area mínima que contiene a cada pieza. Obtenemos su centro, 
            # dimensiones (alto y ancho) y como de girado está respecto al eje X (en grados).
            rect = cv2.minAreaRect(pieza)

            (centro, _, ang) = rect

            # También podemos convertir la info de los rectángulos en un conjunto de puntos para 
            # dibujar el rectángulo.
            rect_points = cv2.boxPoints(rect)
            rect_points = np.round(rect_points).astype(int)

            piezas_data.append([centro, ang, self.pieces_type])
            piezas_rect.append(rect_points)

        return piezas_data, piezas_rect
    
    def get_context(self, pieces_dict: dict):
        """
        Aplica el metodo anterior para el tipo de pieza presente en la imagen.

        Args:
            pieces_dict: Diccionario que contiene contornos agrupados por forma.

        Returns:
            piezas_data: Información sobre coordenadas, rotación y forma de cada pieza.
            piezas_rect: Rectangulo que contiene a cada pieza que se utilizará para
            dibujar el resultado de la detección.
        """

        for piece_type in pieces_dict.keys():
            if len(pieces_dict[piece_type]) > 0:
                self.pieces_type = piece_type
                piezas_data, piezas_rect =  self.get_data(pieces_dict[piece_type])
                return piezas_data, piezas_rect

        return [], []
    
    def get_piece_borders(self, dict_vertices: dict):
        """
        Obtiene los vértices de las piezas presentes en la imagen.

        Args:
            dict_vertices: Diccionario que contiene los vértices de cada pieza agrupados
            por forma.

        Returns:
            vertices_ordenados: Lista de vértices mencionados, ordenados de pieza mayor a menor.
        """

        for piece_type in dict_vertices.keys():
            if len(dict_vertices[piece_type]) > 0:
                vertices = dict_vertices[piece_type]
                vertices_ordenados = [vertices[i] for i in self.indices_ordenados]
                return vertices_ordenados
            
    def plot_results(self, img, pieces, pieces_data, pieces_rect, vertices_ordenados):
        """
        Visualización de resultados.
        """

        # Dar distinto color a los contornos de las piezas en función de su tipo.
        if self.pieces_type == 'triangulos':
            cv2.drawContours(img, pieces, -1, (255, 0, 0), 2)
        elif self.pieces_type == 'cuadrados':
            cv2.drawContours(img, pieces, -1, (0, 0, 255), 2)
        elif self.pieces_type == 'rectangulos':
            cv2.drawContours(img, pieces, -1, (0, 255, 0), 2)
        elif self.pieces_type == 'pentagonos':
            cv2.drawContours(img, pieces, -1, (255, 255, 0), 2)
        elif self.pieces_type == 'circulos':
            cv2.drawContours(img, pieces, -1, (0, 255, 255), 2)    

        # Dibujar el rectángulo que contiene a cada pieza.
        cv2.drawContours(img, pieces_rect, -1, (255, 255, 255), 1)

        # Texto en función de tipo de piezas.
        cv2.putText(img, self.pieces_type, [30, 30], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Mostrar vértices detectados de cada pieza.
        for vertices_pieza in vertices_ordenados:
            for vertice_pieza in vertices_pieza:
                cv2.circle(img, vertice_pieza[0], 2, (0, 0, 255), 1)

        # Mostrar centro de cada rectángulo.
        for i in range(len(pieces_data)):
            point = list(round(x) for x in pieces_data[i][0])
            cv2.circle(img, point, 2, (0, 255, 0), 1)

        # Mostrar esquina inferior derecha del ArUco detectado, que representa
        # nuestro origen de coordenadas.
        cv2.circle(img, self.bottom_left, 2, (0, 255, 255), 2)

        cv2.imwrite('/home/laboratorio/ros_workspace/src/proyecto/final/Unai.png', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def correct_angles(self, pieces_data: list, vertices_ordenados: list):
        """
        Permite corregir ángulos de rotación de triangulos y rectángulos para medirlos
        con respecto al eje X

        Args:
            piezas_data: Información sobre coordenadas, rotación y forma de cada pieza.
            vertices_ordenados: Lista de vértices de cada pieza, tras ordenar por tamaño
            de las mismas
            
        Returns:
            corrected_angles: Ángulos corregidos.
        """

        corrected_angles = []

        print(f'### Corrección de ángulos dado que se tienen {self.pieces_type}:')

        # Hallamos centro de rectángulo y vértices de cada pieza.
        for i in range(len(pieces_data)):
            point = list(round(x) for x in pieces_data[i][0])
            vertices_pieza = vertices_ordenados[i]

            # Distancias de centro de caja a vertices.
            distancias = []

            # Mediremos ángulo con respecto a vértice más cercano al centro de la caja,
            # tal y como hace minAreaRect.
            for j in range(len(vertices_ordenados[0])):
                vertice = vertices_pieza[j][0]

                cuadrante = 0

                distancia = abs(vertice[0] - point[0]) + abs(vertice[1] - point[1])
                distancias.append(distancia)

                # Ver si hay piezas alineadas con ejes. No haría falta lo de después.
                if vertice[0] == point[0]:
                    corrected_angles.append(0.0) # Pieza alineada con eje X.
                elif vertice[1] == point[1]:
                    corrected_angles.append(90.0) # Pieza alineada con eje Y.

            # Extraer posición del vértice de referencia (el más cercano).
            arg_vertice = argmin(distancias)

            # Tomar borde, ver en qué cuadrante está y calcular ángulo.
            vertice = vertices_pieza[arg_vertice][0] 

            # 1: abajo izq. 2: abajo dcha., 3: arriba dcha., 4: arriba izq.
            if vertice[0] < point[0] and vertice[1] > point[1]:
                cuadrante = 1 # Vertice en cuadrante 1
            elif vertice[0] > point[0] and vertice[1] > point[1]:
                cuadrante = 2 # Vertice en cuadrante 2
            elif vertice[0] > point[0] and vertice[1] < point[1]:
                cuadrante = 3 # Vertice en cuadrante 3
            elif vertice[0] < point[0] and vertice[1] < point[1]:
                cuadrante = 4 # Vertice en cuadrante 4

            # Calcular ángulo respecto al eje Y, sin tener en cuenta sentido.
            alpha = arctan(abs((point[0] - vertice[0]) / (vertice[1] - point[1])))
            alpha = alpha * 180.0 / pi

            print(f'Cuadrante de vértice de referencia para pieza {i+1}:' , cuadrante)

            # Corregir según cuadrante.
            if cuadrante == 1:
                corrected_angles.append(-90.0 + alpha)
            elif cuadrante == 2:
                corrected_angles.append(-90.0 - alpha)
            elif cuadrante == 3:
                corrected_angles.append(90.0 + alpha)
            elif cuadrante == 4:
                corrected_angles.append(90.0 - alpha)

            print(f'Valor de ángulo tras corrección: {round(corrected_angles[i], 2)}º\n')

        return corrected_angles
    
    def correct_angles_rect(self, pieces_data: list, vertices_piezas: list):
        """
        Permite corregir ángulos de rotación de rectángulos para medirlos
        con respecto al eje X

        Args:
            piezas_data: Información sobre coordenadas, rotación y forma de cada pieza.
            vertices_ordenados: Lista de vértices de cada pieza, tras ordenar por tamaño
            de las mismas
            
        Returns:
            corrected_angles: Ángulos corregidos.
        """
        
        corrected_angles = []
        for i in range(len(pieces_data)):
            point = list(round(x) for x in pieces_data[i][0])
            vertices_pieza = vertices_piezas[i]

            vertices_pieza_copy = vertices_pieza
            vertices_pieza_copy = vertices_pieza_copy.reshape(-1, 2)

            indices_ordenados = np.argsort(vertices_pieza_copy[:, 1])
            
            # Nos quedamos con los 3 vértices con menor Y ya que en las imagen son los más altos.
            vertice1 = vertices_pieza_copy[indices_ordenados[0]]
            vertice2 = vertices_pieza_copy[indices_ordenados[1]]
            vertice3 = vertices_pieza_copy[indices_ordenados[2]]

            dist1 = abs(vertice1[0] - vertice2[0]) + abs(vertice1[1] - vertice2[1])
            dist2 = abs(vertice1[0] - vertice3[0]) + abs(vertice1[1] - vertice3[1])

            if dist1 > dist2:
                vertice = (vertice1 + vertice3) / 2.0
            else:
                vertice = (vertice1 + vertice2) / 2.0
            print('vertice')
            print(vertice)

            for j in range(len(vertices_piezas[0])):

                cuadrante = 0

                # Ver si hay piezas alineadas con ejes.
                if vertice[0] == point[0]:
                    corrected_angles.append(0.0) # Pieza alineada con eje X.
                elif vertice[1] == point[1]:
                    corrected_angles.append(90.0) # Pieza alineada con eje Y.

            # 1: abajo izq. 2: abajo dcha., 3: arriba dcha., 4: arriba izq.
            if vertice[0] < point[0] and vertice[1] > point[1]:
                cuadrante = 1 # Borde en cuadrante 1
            elif vertice[0] > point[0] and vertice[1] > point[1]:
                cuadrante = 2 # Borde en cuadrante 2
            elif vertice[0] > point[0] and vertice[1] < point[1]:
                cuadrante = 3 # Borde en cuadrante 3
            elif vertice[0] < point[0] and vertice[1] < point[1]:
                cuadrante = 4 # Borde en cuadrante 4

            alpha = arctan(abs((point[0] - vertice[0]) / (vertice[1] - point[1])))
            alpha = alpha * 180.0 / pi

            # Corregir según cuadrante.
            if cuadrante == 1:
                corrected_angles.append(-90.0 + alpha)
            elif cuadrante == 2:
                corrected_angles.append(-90.0 - alpha)
            elif cuadrante == 3:
                corrected_angles.append(90.0 + alpha)
            elif cuadrante == 4:
                corrected_angles.append(90.0 - alpha)

        return corrected_angles
    
    def get_final_message(self, pieces_data, vertices_ordenados, pix_to_dist_x, pix_to_dist_y):
        """
        Permite obtener el mensaje final que debemos enviar al nodo de control del robot.
        Para ello, se hace la conversión de píxeles a distancias para las posiciones de las
        piezas y se corrigen los ángulos en caso necesario.

        Args:
            piezas_data: Información sobre coordenadas, rotación y forma de cada pieza.
            vertices_ordenados: Lista de vértices de cada pieza, tras ordenar por tamaño
            de las mismas.
            pix_to_dist_x: Conversión de px a cm en el eje X.
            pix_to_dist_y: Conversión de px a cm en el eje Y.
            
        Returns:
            final_data: Mensaje que se enviará al nodo de control de robot para ir a por
            las piezas, tomarlas y dejarlas en su lugar correspondiente.
        """
        
        final_positions = []
        pix_to_dist = [pix_to_dist_x, pix_to_dist_y]
        # Origen de coordenadas.
        origin = self.bottom_left

        # Obtener distancias reales.
        for data in pieces_data:
            # Redondear ya que se trata de píxeles.
            point = tuple(round(x) for x in data[0])

            # Pasar sistema de referencias al del origen en el ArUco.
            point_prime = tuple((point[i] - origin[i]) * (-1)**(i) for i in range(2))

            # Realizar conversión.
            point_final = tuple(point_prime[i] * pix_to_dist[i] for i in range(2))

            final_positions.append(point_final)

        # Calcular ángulos de triángulos y pentágonos.
        num_lados = len(vertices_ordenados[0])

        if num_lados == 3 or num_lados == 5:
            corrected_angles = self.correct_angles(pieces_data, vertices_ordenados)

        if self.pieces_type == "rectangulos":
            corrected_angles = self.correct_angles_rect(pieces_data, vertices_ordenados)

        final_data = []
        for i in range(len(pieces_data)):
            current_data = []
            current_data.append(final_positions[i])

            if num_lados == 3 or num_lados == 5:
                current_data.append(corrected_angles[i])
            else:
                current_data.append(pieces_data[i][1])

            current_data.append(self.pieces_type)

            final_data.append(current_data)

        print('### Información final:')

        counter = 1
        for data in final_data:
            print(f'Pieza {counter}:')

            x, y = round(data[0][0], 2), round(data[0][1], 2)

            print(f'Coordenadas: {(float(x), float(y))}cm')
            print(f'Ángulo de rotación: {round(data[1], 2)}º')
            print(f'Tipo de pieza: {data[2]}\n')
            counter += 1

        return final_data
    
    def process_image(self, img):
        """
        Se realiza la detección, clasificación y ploteo de resultados. Por último, se
        devuelve el mensaje final para el robot. Se trata de un método que gestiona todos
        los anteriores para darnos los resultados deseados.

        Args:
            img: Imagen en la que se quieren detectar las piezas.
            
        Returns:
            final_data: Mensaje que se enviará al nodo de control de robot para ir a por
            las piezas, tomarlas y dejarlas en su lugar correspondiente.
        """

        # Quitar distorsión de la imagen original.
        img_corrected = self.correct_image(img)
        # Obtener conversión de px a cm.
        pix_to_dist_x, pix_to_dist_y = self.pixel_to_distance(img_corrected)
        # Hallar contornos.
        pieces_contours = self.get_piece_contours(img_corrected)
        # Realizar clasificación.
        dict_pieces, dict_vertices = self.PolygonDetection(pieces_contours)
        # Tomar info del tipo de pieza que se tiene y ordenar por tamaño.
        pieces_data, pieces_rect = self.get_context(dict_pieces)

        if len(pieces_data) > 0:
            # Obtener vértices de cada figura y ordenar por tamaño.
            vertices_ordenados = self.get_piece_borders(dict_vertices)
            # Realizar conversión de px a cm y corregir ángulos si corresponde para obtener
            # el mensaje final.
            final_data = self.get_final_message(pieces_data, vertices_ordenados, pix_to_dist_x, pix_to_dist_y)
            # Mostrar resultados.

            self.plot_results(img_corrected, dict_pieces[self.pieces_type], pieces_data, pieces_rect, vertices_ordenados)

            print('')
            print(final_data)

            return final_data
        
        else:
            return []