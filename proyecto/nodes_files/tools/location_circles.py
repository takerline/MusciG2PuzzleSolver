import cv2 
import numpy as np 
import matplotlib.pyplot as plt
import random
import time
import os
import uuid
from typing import Union
from threading import Thread

class DetectHanoiTower:
    def __init__(self, factor_resize=1):
        """
        Inicializa la clase DetectHanoiTower.

        Este método configura una lista de colores para ser utilizados en la visualización
        y establece el factor de redimensionamiento para las imágenes.

        Args:
            factor_resize (float, opcional): Factor de redimensionamiento para las imágenes.
                                             Por defecto es 1 (sin redimensionamiento).

        Atributos:
            factor_resize (float): Factor de redimensionamiento para las imágenes.
            colors (list): Lista de colores para visualización, cada uno representado como una tupla RGB.
                           Los colores incluidos son:
                           - Rojo indio
                           - Verde primavera medio
                           - Azul acero
                           - Orquídea
                           - Naranja oscuro
                           - Azul pizarra
                           - Verde azulado
                           - Marrón rosáceo

        """
        self.factor_resize = factor_resize
        self.colors = [
                (205, 92, 92),   # Rojo indio
                (60, 179, 113),  # Verde primavera medio
                (70, 130, 180),  # Azul acero
                (218, 112, 214), # Orquídea
                (255, 140, 0),   # Naranja oscuro
                (106, 90, 205),  # Azul pizarra
                (0, 128, 128),   # Verde azulado
                (188, 143, 143)  # Marrón rosáceo
        ]
    def read_image_path(self, path) -> np.ndarray:
        """
        Lee una imagen desde la ruta dada y opcionalmente la redimensiona en caso de que se tenga imagenes de 
        diferentes dispositivos.

        Args:
            path (str): La ruta del archivo de la imagen a leer.
            factor_resize (float, opcional): El factor por el cual redimensionar la imagen. 
                                         Si es 1, no se realiza ninguna redimensión. Por defecto es 1.

        Returns:
            np.array: lectura de imagen
        """

        # Permite determinar si lo que se esta enviando es un frame o una ruta de archivo
        if type(path) == str:
            self.img = cv2.imread(path, cv2.IMREAD_COLOR)
        else:
            self.img = path

        # Imagen reescalada a apartir del factor de escalado
        if self.factor_resize == 1:
            self.img = self.img.copy()
        else:
            new_w = int(self.img.shape[1]/self.factor_resize)
            new_h = int(self.img.shape[0]/self.factor_resize)
            self.img = cv2.resize(self.img, (new_w,new_h), cv2.INTER_CUBIC)

        return self.img

    def convert_bgr_gray(self, img: np.ndarray) -> np.ndarray:
        """
        Convierte una imagen BGR a escala de grises.

        Args:
            img (np.ndarray): Imagen de entrada en formato BGR.

        Returns:
            np.ndarray: Imagen en escala de grises.

        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        return gray

    def blurfilter(self, img: np.ndarray, kernel_size: int = 3) -> np.ndarray:
        """
        Aplica un filtro de desenfoque mediano a la imagen.

        Args:
            img (np.ndarray): Imagen de entrada.
            kernel_size (int, opcional): Tamaño del kernel para el filtro mediano. Por defecto es 3.

        Returns:
            np.ndarray: Imagen desenfocada.

        """
        blurred = cv2.medianBlur(img, kernel_size) 
        return blurred

    def cannyfilter(self, img: np.ndarray, l1: int, l2: int) -> np.ndarray:
        """
        Aplica el filtro de detección de bordes Canny a la imagen.

        Args:
            img (np.ndarray): Imagen de entrada.
            l1 (int): Umbral inferior para la detección de bordes.
            l2 (int): Umbral superior para la detección de bordes.

        Returns:
            np.ndarray: Imagen con los bordes detectados.

        """
        edged = cv2.Canny(img, l1, l2)
        return edged

    def morphology_dilation(self, img: np.ndarray, kernel_size = (3,3), iterations: int = 1) -> np.ndarray:
        """
        Aplica una operación de dilatación morfológica a la imagen.

        Args:
            img (np.ndarray): Imagen de entrada.
            kernel_size (tuple, opcional): Tamaño del kernel para la dilatación. Por defecto es (3,3).
            iterations (int, opcional): Número de iteraciones para la dilatación. Por defecto es 1.

        Returns:
            np.ndarray: Imagen dilatada.

        """
        kernel = np.ones(kernel_size, np.uint8)
        dilatada = cv2.dilate(img, kernel, iterations=iterations)
        return dilatada

    def morphology_erosion(self, img: np.ndarray, kernel_size = (3, 3)) -> np.ndarray:
        """
        Aplica una operación de erosión morfológica a la imagen de entrada.

        Args:
            img (np.ndarray): Imagen de entrada sobre la que se aplicará la erosión.
                            Debe ser una imagen en escala de grises o binaria.
            kernel_size (Tuple[int, int], opcional): Tamaño del kernel para la erosión.
                                                    Por defecto es (3, 3).

        Returns:
            np.ndarray: Imagen erosionada.

        """
        # Crear un kernel rectangular lleno de unos
        kernel = np.ones(kernel_size, dtype=np.uint8)

        # Aplicar la operación de erosión
        eroded = cv2.erode(img, kernel, iterations=1)

        return eroded

    def show_image(self, img: np.ndarray, name: str) -> None:
        """
        Muestra una imagen en una ventana.

        Args:
            img (np.ndarray): Imagen a mostrar.
            name (str): Nombre de la ventana.

        Returns:
            None

        """
        cv2.imshow(name, img)
        #cv2.waitKey(0)

    def group_columns(self, columns: np.ndarray):
        """
        Agrupa las columnas cercanas basándose en sus coordenadas y sobre el radio más grande entre todos 
        los circulos detectados.

        Args:
            columns (np.ndarray): Array con las coordenadas y radios de los círculos detectados.

        Returns:
            List[np.ndarray]: Lista de columnas agrupadas.
        """

        # Ordenar las columnas por su coordenada x
        columns = columns[columns[:, 0].argsort()]

        # Encontrar el radio máximo
        max_r = np.max(columns[:, 2])

        # Agrupar columnas cercanas
        grouped_columns = []
        for column in columns:
            if not grouped_columns or abs(column[0] - grouped_columns[-1][0]) > max_r:
                grouped_columns.append(column)
            else:
                # Si está cerca, actualizar el grupo existente con el círculo más grande
                if column[2] > grouped_columns[-1][2]:
                    grouped_columns[-1] = column

        return grouped_columns

    def cut_columns(self, columns: np.ndarray, img: np.ndarray, delta: int):
        """
        Corta las columnas de la imagen basándose en las coordenadas de los círculos detectados.

        Args:
            columns (np.ndarray): Array con las coordenadas y radios de los círculos detectados.
            img (np.ndarray): Imagen en dos dimensiones.
            delta (int): Margen adicional para el recorte.

        Returns:
            dict: Diccionario con los recortes de las columnas y sus coordenadas.
        """
        cuts = {}

        # Agrupar las columnas 
        grouped_columns = self.group_columns(columns)

        # Realizar los cortes
        for l, pt in enumerate(grouped_columns):
            a, b, r = pt.astype(np.int32)

            # Nos permite determinar si se trata de una columna no detectada que debia ser detectda
            if r != -1:
                # Calcular las coordenadas de recorte
                x1 = max(0, a - r + delta)
                y1 = max(0, b - r + delta)
                x2 = min(img.shape[1], a + r - delta)
                y2 = min(img.shape[0], b + r - delta)

                # Realizar el recorte
                cut = img[y1:y2, x1:x2]
                #self.show_image(cut, f'Columna {l+1}')
                cuts[l] = [cut, [x1, y1]]

        return cuts

    def adaptative_filter(self, img: np.ndarray, output_thresh: int = 255, neighborhood_size: int = 9, c: int = 3) -> np.ndarray:
        """
        Aplica un filtro adaptativo a la imagen.

        Args:
            img (np.ndarray): Imagen de entrada en escala de grises.
            output_thresh (int): Valor máximo para la imagen binarizada. Por defecto es 255.
            neighborhood_size (int): Tamaño del vecindario para el cálculo del umbral. Por defecto es 9.
            c (int): Constante sustraída de la media. Por defecto es 3.

        Returns:
            np.ndarray: Imagen binarizada después de aplicar el filtro adaptativo.
        """
        thresh = cv2.adaptiveThreshold(
            img, 
            output_thresh, 
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV,
            neighborhood_size,
            c
        )
        return thresh

    def verificar_circulo(self, pt: np.ndarray, img_size):
        """
        Verifica si un círculo está completamente dentro de los límites de la imagen.

        Args:
            pt (np.ndarray): Array con las coordenadas del centro y el radio del círculo [x, y, r].
            img_size (tuple): Tupla con las dimensiones de la imagen (altura, ancho).

        Returns:
            bool: True si el círculo está fuera de los límites de la imagen, False en caso contrario.
        """
        x, y, r = pt[0], pt[1], pt[2]
        fuera_x = (x - r < 0) or (x + r > img_size[1])
        fuera_y = (y - r < 0) or (y + r > img_size[0])

        return fuera_x or fuera_y
    
    def detect_columns(self, edge_image: np.ndarray) -> np.ndarray:
        """
        Detecta las columnas (torres) en la imagen del juego de Hanoi.

        Este método busca contornos en la imagen con bordes detectados, identifica
        formas circulares que podrían representar las columnas del juego de Hanoi,
        y las procesa para obtener sus coordenadas.

        Args:
            edge_image (np.ndarray): Imagen con bordes detectados.

        Returns:
            np.ndarray: Si se detectan columnas, devuelve un array de coordenadas de columnas [x, y, r].

        Nota:
            - Las columnas se ordenan de izquierda a derecha.
            - Se detectan y marcan las columnas "perdidas" (sin discos).
        """
        contours, hierarchy = cv2.findContours(edge_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        potential_columns = []
        
        for contour, contour_hierarchy in zip(contours, hierarchy[0]):
            contour_perimeter = cv2.arcLength(contour, False)

            # Solo se analizan contornos con un perimetro mayor a 50 pixels, nos permite eliminar ruido
            if contour_perimeter > 50:

                # Verifica si el contorno es un contorno padre
                if contour_hierarchy[3] == -1:
                    contour_mask = np.zeros_like(edge_image)
                    cv2.drawContours(contour_mask, [contour], -1, 255, 1)

                    #self.show_image(contour_mask, "Contornos internos en columns")
                    
                    circle_coordinates = self.detect_circular_shapes(contour_mask, 100, 300, 18, 1)
                    if circle_coordinates is not None:  
                        
                        x, y, r = map(int, circle_coordinates)

                        contour_mask_c = np.copy(contour_mask)
                        contour_mask_c = cv2.cvtColor(contour_mask_c, cv2.COLOR_GRAY2BGR)
                        cv2.circle(contour_mask_c, (x, y), r, (0, 255, 0), 2)
                        cv2.circle(contour_mask_c, (x, y), 2, (0, 0, 255), 3)
                        #self.show_image(contour_mask_c, "Contornos detectados")

                        #self.show_image(contour_mask_c, "Contornos detectados (marcados)")
                        #cv2.waitKey(0)
                        potential_columns.insert(0, circle_coordinates)
        if len(potential_columns) > 0:
            
            potential_columns = np.array(potential_columns) 
            unique_columns = self.delete_nested_circles(potential_columns)

            # Ordenar columnas de izquierda a derecha
            sorted_columns = unique_columns[unique_columns[:,0].argsort()]
            columns_with_lost = self.detect_losed_columns(sorted_columns, edge_image.shape[1])
            print("Todos: ",columns_with_lost)
            return columns_with_lost

        return False
    
    def detect_circular_shapes(self, thresh, param0=50, param1=10, param2=40, tu=1) -> np.array:
        """
        Detecta formas circulares y retorna el círculo más grande encontrado o False si no se detecta ninguno.

        Args:
            thresh (np.ndarray): Imagen umbralizada en la que se buscarán los círculos.
            param0 (int, optional): Distancia mínima entre los centros de los círculos detectados. Default is 50.
            param1 (int, optional): Parámetro del método Canny para la detección de bordes. Default is 10.
            param2 (int, optional): Umbral para la detección de centros. Default is 40.
            tu (float, optional): Inverso del ratio de la resolución de la imagen. Default is 1.

        Returns:
            np.array or bool: Array con las coordenadas [x, y, r] del círculo más grande detectado,
                              o False si no se detecta ningún círculo.
        """
        # Aplicar la transformada de Hough para detectar círculos
        detected_circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, tu, param0,
        param1=param1, param2=param2, 
        minRadius=0, maxRadius=int(thresh.shape[0]))

        if detected_circles is not None: 
            # Convertir las coordenadas y radios a enteros
            detected_circles = np.uint16(np.around(detected_circles)) 

            # Inicializar variable para almacenar el círculo más grande
            big_circle = [0,0,0] 

            # Iterar sobre todos los círculos detectados
            for pt in detected_circles[0, :]: 
                # Actualizar big_circle si se encuentra un círculo más grande
                if big_circle[2] < pt[2]:
                    big_circle = pt

            # Verificar si se detectó al menos un círculo
            if len(big_circle) > 1:
                return big_circle

    def detect_nested_circles(self, edged: np.ndarray, gray: np.ndarray, delta: int):
        """
        Detecta los circulos anidados del proceso iterativo de cada columna.

        Args:
            edged (np.ndarray): Imagen con bordes detectados.
            gray (np.ndarray): Imagen en escala de grises.
            delta (int): Valor de ajuste para el recorte del ROI.

        Returns:
            Tuple[Optional[np.ndarray], Optional[Tuple[int, int, int]], Optional[Tuple[int, int]]]: 
                - ROI del círculo más grande detectado.
                - Coordenadas (x, y, r) del círculo más grande.
                - Coordenadas (x1, y1) del punto superior izquierdo del ROI.
            Retorna (None, None, None) si no se detectan círculos.
        """
        contours, _ = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)

        inside = []
        
        for cnt in contours:
            pe = cv2.arcLength(cnt, False)
            if pe > 5:
                black = np.zeros_like(edged)
                cv2.drawContours(black, [cnt], -1, 255, 1)
                
                coordinates = self.detect_circular_shapes(black, 100, 300, 20, 1.4)
                
                if coordinates is not None:
                    if pe > (2 * np.pi * coordinates[2]) * 0.5:
                        inside.append(coordinates)

        if not inside:
            self.flag = False
            return None, None, None
        
        inside = np.array(inside)
        largest_circle = inside[inside[:, 2].argsort()][-1]
        


        x, y, r = map(int, largest_circle)

        # Calcular las coordenadas del ROI
        x1 = max(0, x - r + delta)
        x2 = min(gray.shape[1], x + r - delta)
        y1 = max(0, y - r + delta)
        y2 = min(gray.shape[0], y + r - delta)

        roi = gray[y1:y2, x1:x2]
        
        return roi, (x, y, r), (x1, y1)

    def reverse_split(self, circle: np.ndarray, x: int, y: int) -> None:
        """

        Este método ajusta las coordenadas de un círculo detectado en una subimagen
        para que correspondan a su posición en la imagen original completa.

        Args:
            circle (np.ndarray): Array con las coordenadas [x, y, r] del círculo.
            x (int): Desplazamiento en x de la subimagen respecto a la imagen original.
            y (int): Desplazamiento en y de la subimagen respecto a la imagen original.

        Returns:
            None
        """
        # Guarda las coordenadas originales del círculo
        orginal_x = int(circle[0])
        orginal_y = int(circle[1])

        # Ajusta las coordenadas del círculo a la posición en la imagen original
        circle[0] += x
        circle[1] += y

        # La clave es un valor único basado en las coordenadas originales
        self.show_circles[orginal_x*orginal_y] = circle


    def draw_detection(self, gray: np.ndarray) -> np.ndarray:
        """
        Grafica los círculos detectados en la imagen con etiquetas.

        Este método dibuja círculos y etiquetas sobre una copia de la imagen de entrada
        para visualizar los discos detectados.
        Args:
            gray (np.ndarray): Imagen en escala de grises sobre la que se dibujarán los círculos y etiquetas.

        Returns:
            np.ndarray: Imagen en color con los círculos y etiquetas dibujados.
        """
        # Convierte la imagen de escala de grises a color
        color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Itera sobre cada disco detectado
        for disk in self.show_circles_filter:
            x, y, r = [int(v) for v in self.show_circles_filter[disk]]

            # Dibuja el círculo
            cv2.circle(color, (x, y), r, self.colors[disk], 3)

            # Configura el texto de la etiqueta
            font = cv2.FONT_HERSHEY_SIMPLEX
            text = str(disk)
            font_scale = 0.4
            font_thickness = 1
            text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)

            # Calcula la posición de la etiqueta (en la parte superior del círculo)
            text_x = x - text_size[0] // 2
            text_y = y - r - 2 

            # Dibuja un rectángulo de fondo para la etiqueta
            cv2.rectangle(color, 
                          (text_x - 2, text_y - text_size[1] - 2),
                          (text_x + text_size[0] + 2, text_y + 2),
                          self.colors[disk], 
                          -1)

            # Dibuja el texto de la etiqueta en blanco
            cv2.putText(color, text, (text_x, text_y), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

        return color

    def detect_square(self, edged: np.ndarray, blur: np.ndarray, original: np.ndarray, delta: int):
        """
        Detecta un cuadrado en la imagen que contiene círculos (las torres de Hanoi).

        Este método busca contornos cuadrados en la imagen y verifica si contienen círculos,
        que representarían las torres del juego de Hanoi.

        Args:
            edged (np.ndarray): Imagen con bordes detectados.
            blur (np.ndarray): Imagen suavizada.
            delta (int): Margen para ajustar el recorte de la región de interés para no volver a detectar el circulo ya detectado.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray]: 
                - Array de columnas detectadas.
                - Región de interés recortada.
                - Coordenadas [x, y] del recorte en la imagen original para redimensionar los circulos.
            Si no se detecta un cuadrado válido, se lanza un error.
        """
        contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        matches = {}

        for cnt, h in zip(contours, hierarchy[0]):
            # Verifica si el contorno es cerrado
            if cv2.contourArea(cnt) > cv2.arcLength(cnt, True):
                # Verifica si el contorno es un contorno padre
                if h[3] == -1 and h[2] != -1:
                    epsilon = 0.01 * cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, epsilon, True)

                    # Verifica si el contorno es un rectangulo
                    if len(approx) == 4:
                        # Recorta la región de interés
                        x, y, w, h = cv2.boundingRect(approx)

                        # Se agrega un delta para recortar el perimetro del rectangulo
                        x1,y1  = x + delta, y + delta
                        x2, y2 = x + w - delta, y + h - delta
                        
                        rblur = blur[y1:y2, x1:x2]
                        redged = original[y1:y2, x1:x2]

                        if rblur.shape[0] > 0 and rblur.shape[1] > 0:
                            #self.show_image(rblur, 'blur cuadrado')

                            # Aplica filtros adicionales a la región recortada
                            canny_square_img = self.cannyfilter(redged, l1=10, l2=60)
                            #self.show_image(canny_square_img, 'recortada')
                            dilated_canny_squeare = self.morphology_dilation(canny_square_img, (2,2))

                            # Detecta columnas en la región recortada
                            columns = self.detect_columns(dilated_canny_squeare)

                            if columns is not False:
                                matches[columns.shape[1]] = [columns, rblur, [x1, y1]]

        keys = np.array(list(matches.keys()))

        # Se determina si se detecta al menos una columna
        if len(keys) > 0:
            # Selecciona el mejor match (el que tiene más columnas)
            best = max(keys)

            # Ajusta las coordenadas de los círculos detectados a la imagen original
            [self.reverse_split(circle.copy(), matches[best][2][0], matches[best][2][1]) for circle in matches[best][0]]

            return matches[best][0], matches[best][1], matches[best][2]

        # No se detecta un cuadrado válido
        raise ValueError("No se detecto la base")

    def delete_nested_circles(self, coordinates: np.ndarray) -> np.ndarray:
        """
        Elimina círculos anidados dentro de otros círculos más grandes.

        Args:
            coordinates (np.ndarray): Array de coordenadas de círculos [x, y, r].

        Returns:
            np.ndarray: Array de coordenadas de círculos únicos (no anidados).
        """
        # Ordena los círculos por radio de mayor a menor
        sorted_coordinates_by_radius = coordinates[coordinates[:,2].argsort()][::-1]

        delete_index = []

        # Compara cada círculo con los siguientes más pequeños
        for j, p1 in enumerate(sorted_coordinates_by_radius):
            x1, y1, r1 = p1
            secondary = sorted_coordinates_by_radius[j+1:]

            for k, p2 in enumerate(secondary):
                x2, y2, _ = p2
                # Si el centro del círculo más pequeño está dentro del más grande, se marca para eliminar
                if np.sqrt((x2 - x1)**2 + (y2 - y1)**2) < r1:
                    delete_index.append(k+j+1)

        # Elimina los círculos anidados
        unique_columns = np.delete(sorted_coordinates_by_radius, delete_index, axis=0)
        return unique_columns

    def detect_losed_columns(self, columns: np.ndarray, weight: int) -> np.ndarray:
        """
        Detecta si no se detectaron aquellas columnas sin circulos y las marca con [distancia media, -1, -1].

        Args:
            columns (np.ndarray): Array de coordenadas de columnas [x, y, r].
            weight (int): Ancho de la imagen para calcular la distancia entre columnas.

        Returns:
            np.ndarray: Array de coordenadas de columnas con las perdidas marcadas.
        """
        # Se agrega la primera columna al resultado
        columns_with_losed_columns = [columns[0]]
        weight = int(weight/2)

        # Compara la distancia entre columnas consecutivas
        for k in range(1, len(columns)):
            x1 = int(columns[k-1][0])
            x2 = int(columns[k][0])

            diff = abs(x1 - x2) 
            # Si la distancia es mayor que la mitad del ancho, se considera una columna perdida
            if diff > weight:
                mean_distance_x = x1 + abs(x2-x1)/2
                columns_with_losed_columns.append([mean_distance_x, -1, -1])  # Marca la columna perdida
            columns_with_losed_columns.append(columns[k])

        return np.array(columns_with_losed_columns)
    
    def delete_column_detection(self, high_rect: float, circles):
        """
        Elimina los círculos detectados que son demasiado pequeños en relación con la altura del rectángulo.

        Args:
            high_rect (float): Altura del rectángulo de referencia.
            circles (Dict[int, List[Tuple[int, int, int]]]): Diccionario de círculos detectados por columna.

        Returns:
            Dict[int, List[Tuple[int, int, int]]]: Diccionario actualizado de círculos detectados.
        """
        for key in circles:
            if circles[key] and circles[key][-1][2] / high_rect < 0.13:
                circles[key].pop()
        
        return circles


    def filter_small_circles(self, circles, rectangle_height: float):
        """
        Filtra círculos pequeños basándose en el tamaño relativo al rectángulo.

        Args:
            circles (Dict[int, List[List[int]]]): Diccionario donde las claves son índices de columnas
                y los valores son listas de círculos. Cada círculo es una lista [x, y, r].
            rectangle_height (float): Altura del rectángulo que contiene las torres.

        Returns:
            Dict[int, List[List[int]]]: Diccionario filtrado con los círculos que cumplen el criterio de tamaño.
        """
        filtered = {}
        for key, column_circles in circles.items():
            filtered[key] = [circle for circle in column_circles if circle[2] / rectangle_height >= 0.15]
        return filtered

    def organize_circles(self, circles):
        """
        Organiza los círculos en una estructura de lista 3x5.

        Args:
            circles (Dict[int, List[List[int]]]): Diccionario de círculos filtrados por columna.

        Returns:
            List[List[List[int]]]: Lista de 3 columnas, cada una con 5 elementos (círculos o [0,0,0]).
        """
        organized = []
        for i in range(3):
            column = circles.get(i, [])
            print(column)
            organized.append(column + [[0,0,0]] * (5 - len(column)))
        return organized

    def number_disks(self, circles) -> np.ndarray:
        """
        Asigna números a los discos basándose en su tamaño y actualiza show_circles_filter.

        Args:
            circles (List[List[List[int]]]): Lista organizada de círculos (3x5).

        Returns:
            np.ndarray: Matriz 3x5 que representa la configuración numerada de las torres de Hanoi.
        """
        circles_array = np.array(circles)
        output = np.zeros((3, 5), dtype=int)
        disk_number = 5

        while np.sum(circles_array) > 0:
            radius_array = circles_array[:,:,2]
            max_radius = np.max(radius_array)
            x, y = np.unravel_index(np.argmax(radius_array), radius_array.shape)

            output[x, y] = disk_number
            circle_key = int(circles_array[x, y][0] * circles_array[x, y][1])
            self.show_circles_filter[disk_number] = self.show_circles.get(circle_key, [0,0,0])

            circles_array[x, y] = [0,0,0]
            disk_number -= 1

        return np.flip(output, axis=1)
    def format_output(self, detected_circles, rectangle_height: float) -> np.ndarray:
        """
        Formatea la salida de los círculos detectados en una matriz que representa
        la configuración de las torres de Hanoi.

        Este método realiza las siguientes operaciones:
        1. Filtra los círculos detectados basándose en su tamaño relativo a la base.
        2. Organiza los círculos en una matriz 3x5 (3 torres, máximo 5 discos).
        3. Asigna números a los discos basándose en su tamaño (5 para el más grande, 1 para el más pequeño).
        4. Actualiza el diccionario self.show_circles_filter para la visualización.

        Args:
            detected_circles (Dict[int, List[List[int]]]): Diccionario de círculos detectados por columna.
            rectangle_height (float): Altura del rectángulo que contiene las torres.

        Returns:
            np.ndarray: Matriz 3x5 que representa la configuración de las torres de Hanoi.
        """
        print("Circulos detectados: ",detected_circles)
        filtered_circles = self.filter_small_circles(detected_circles, rectangle_height)
        print("Organizados: ",filtered_circles)
        organized_circles = self.organize_circles(filtered_circles)
        numbered_disks = self.number_disks(organized_circles)
        return numbered_disks

    def inicializate(self):
        """
        Inicializa los diccionarios para almacenar información sobre los círculos detectados.
        
        - show_circles: para almacenar todos los círculos detectados.
        - show_circles_filter: para almacenar los círculos filtrados.
        """
        self.show_circles = {}
        self.show_circles_filter = {}

    def preprocess_image(self) -> None:
        """
        Realiza el preprocesamiento de la imagen.
        
        Este método aplica una serie de operaciones de procesamiento de imagen:
        1. Convierte la imagen a escala de grises.
        2. Convierte la imagen al espacio de color LAB y normaliza el canal 'a'.
        3. Aplica un filtro de desenfoque a la imagen en escala de grises y al canal 'a' normalizado.
        4. Aplica el filtro Canny para detección de bordes.
        5. Realiza una dilatación morfológica en la imagen de bordes.
        """
        self.gray_img = self.convert_bgr_gray(self.img)

        lab = cv2.cvtColor(self.img, cv2.COLOR_BGR2LAB)
        l, self.a, b = cv2.split(lab)
        

        self.a_norm = cv2.normalize(self.a, None, 0, 255, cv2.NORM_MINMAX)
        self.b_norm = cv2.normalize(b, None, 0, 255, cv2.NORM_MINMAX)

        #self.show_image(self.a_norm, 'Hola')

        self.blur_img = self.blurfilter(self.gray_img, kernel_size=3)
        #self.show_image(self.gray_img, 'Grises')
        self.blur_lab_img = self.blurfilter(self.a_norm, kernel_size=3)

        canny_img = self.cannyfilter(self.blur_img, l1=30, l2=40)
        #self.show_image(canny_img, 'Canny')
        self.dilated_img = self.morphology_dilation(canny_img, kernel_size=(1,1))

    def main(self, path) -> None:
        """
        Método principal para procesar una imagen y detectar la torre de Hanoi.

        Args:
            path (str): Ruta de la imagen a procesar.

        Returns:
            tuple: (output, detection, self.a_norm, self.img)
                output (np.ndarray): Matriz que representa la configuración de las torres de Hanoi.
                detection (np.ndarray): Imagen con las detecciones dibujadas.
                self.a_norm (np.ndarray): Canal 'a' normalizado del espacio de color LAB.
                self.img (np.ndarray): Imagen original.

        Este método realiza los siguientes pasos:
        1. Lee la imagen de entrada.
        2. Inicializa los diccionarios necesarios.
        3. Preprocesa la imagen.
        4. Detecta la base y las columnas de la torre de Hanoi.
        5. Recorta las bases de las columnas.
        6. Detecta los círculos (discos) en cada columna.
        7. Formatea la salida.
        8. Dibuja las detecciones en la imagen.
        """

        self.read_image_path(path)
        self.inicializate()
        self.preprocess_image()

        columns, square_blur, zoom = self.detect_square(self.dilated_img, self.blur_lab_img,self.blur_img, delta=5)

        columns = columns[columns[:, 2].argsort()][::-1][:3]
        print("COLUMNAS: ",columns)
        columns = columns[columns[:, 0].argsort()]

        self.high_rect = min(square_blur.shape)
        delta = int(square_blur.shape[0] * 0.05)
        cuts = self.cut_columns(columns, square_blur, delta)

        locations = {}

        count = 5
        for k, m in enumerate(cuts):
            cut = cuts[m][0]
            zoom_copy = zoom.copy()
            zoom_copy[0] += cuts[m][1][0]
            zoom_copy[1] += cuts[m][1][1]
            delta = int(cut.shape[0] * 0.05)
            locations[m] = [list(columns[m])]
            self.flag = True

            while self.flag:
                can = self.cannyfilter(cut, l1=20, l2=70)
                cut, circle, other = self.detect_nested_circles(can, cut, delta=delta)
                #self.show_image(cut, f'Columna {count}')
                delta = int(delta / 1.1)

                if circle is not None:
                    self.reverse_split(list(circle), zoom_copy[0], zoom_copy[1])
                    zoom_copy[0] += other[0]
                    zoom_copy[1] += other[1]
                    locations[m].append(list(circle))

                    #self.show_image(cut, f'Columna {count}')
                    count += 1

        output = self.format_output(locations, self.high_rect)
        detection = self.draw_detection(self.gray_img)
        return output, detection, self.a_norm, self.img


class RTSPVideoStream:
    def __init__(self, url):
        self.stream = cv2.VideoCapture(url)
        self.stopped = False
        self.frame = None

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while not self.stopped:
            if not self.stream.isOpened():
                self.stop()
                return
            (grabbed, frame) = self.stream.read()
            if not grabbed:
                self.stop()
                return
            self.frame = frame

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

if __name__ == '__main__':    
    #RTSP URL - replace with your camera's RTSP stream URL
    #Initialize the video capture
    cap = cv2.VideoCapture(6)
    if not cap.isOpened():
        print("Error: Could not open RTSP stream.")
        exit()

    detect_hanoi = DetectHanoiTower(factor_resize=1)
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        frame_count += 1

        # Process every 10th frame
        if frame_count % 10 == 0:
            # Process the frame
            begin = time.time()
            try:
                output, detection, _, _ = detect_hanoi.main(frame)
                end = time.time()

                print("Salida:", output)
                print(f"Tiempo de procesamiento: {end - begin} segundos")

                # Display the results
                #cv2.imshow('Imagen original', frame)
                #cv2.moveWindow('Imagen original', 1000, 0)
                cv2.imshow('Detecciones', detection)
                #cv2.waitKey(0)
            except:
                continue

        # Always show the original frame
        cv2.imshow('Imagen original', frame)

        # Break the loop if 'q' is pressedq
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    cap.release()
    cv2.destroyAllWindows()



    # Directorio que contiene las imágenes
    # image_directory = 'imagenes_12_ene_lab_a'
    # image_directory = 'cam'

    # Obtener una lista de todos los archivos de imagen en el dmirectorio
    # image_files = [f for f in os.listdir(image_directory) if f.endswith('.png') or f.endswith('.jpeg')]
    
    # detect_hanoi = DetectHanoiTower()
    # flag = False
    # for image_file in image_files:
    #     image_file = 'img_gorka_17.png'
    #     path_img = os.path.join(image_directory, image_file)
    #     print(f"Procesando: {path_img}")
        
    #     begin = time.time()
    #     output, detenction, _, img= detect_hanoi.main(path_img)
    #     end = time.time()
    #     print("Salida: ",output)
    #     print(f"Tiempo de procesamiento: {end - begin} segundos")


    #     cv2.imshow('Imagen original', img)
    #     cv2.moveWindow('Imagen original', 1000, 0)
    #     cv2.imshow('Detecciones', detenction)

    #     print("Presiona 'm' para continuar con la siguiente imagen...")
        
    #     while True:
    #         key = cv2.waitKey(0) & 0xFF
    #         if key == ord('m'):
    #             break
    #         elif key == ord('r'):
    #             flag = True
    #             cv2.destroyAllWindows()
    #             break
        
    #     if flag:
    #         break
    #     cv2.destroyAllWindows()
    #     break
