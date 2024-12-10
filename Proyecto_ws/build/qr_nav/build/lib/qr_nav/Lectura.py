# Importamos librerías para lecturas
import cv2
import pyzbar.pyzbar as pyzbar
import numpy as np

# Creamos la videocaptura
cap = cv2.VideoCapture(0)

# Diccionario de mensajes de acción según la letra del QR
actions = {
    'A': 'Der90',
    'B': 'Izq90',
    'C': 'Der45',
    'D': 'Izq45',
    'E': 'Avanzar',
    'F': 'Retroceder',
    'G': 'Alto',
    'H': 'Vuelta180'
}

# Empezamos
while True:
    # Leemos los frames
    ret, frame = cap.read()

    # Leemos los códigos QR
    for codes in pyzbar.decode(frame):
        # Extraemos y decodificamos la información del QR
        info = codes.data.decode('utf-8')

        # Tipo de acción LETRA
        letra = info[0]  # Extrae el primer carácter
        accion = actions.get(letra, "Acción no reconocida")

        # Extraemos coordenadas para dibujar el recuadro alrededor del QR
        pts = np.array([codes.polygon], np.int32)
        xi, yi = codes.rect.left, codes.rect.top

        # Redimensionamos
        pts = pts.reshape((-1, 1, 2))

        # Dibujamos el polígono y el texto en la imagen
        cv2.polylines(frame, [pts], True, (255, 255, 0), 5)
        cv2.putText(frame, f'{letra} - {accion}', (xi - 15, yi - 15), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 55, 0), 2)

        # Imprime el mensaje de acción correspondiente
        print(f"Código: {info}, Acción: {accion}")

    # Mostramos FPS
    cv2.imshow("LECTOR DE QR", frame)

    # Salir con 'ESC'
    if cv2.waitKey(5) == 27:
        break

cv2.destroyAllWindows()
cap.release()

