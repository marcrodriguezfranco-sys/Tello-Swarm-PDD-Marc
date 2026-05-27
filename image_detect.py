"""
Detección de objetos en una imagen subida desde disco.
Usa el mismo modelo YOLOv8 que detection.py.
"""
import random
import cv2
from tkinter import filedialog, messagebox
from detection import model


def detect_in_image(parent_window=None):
    """
    Abre un diálogo para escoger una imagen, ejecuta YOLOv8 sobre ella,
    muestra el resultado en una ventana de OpenCV y devuelve la lista de
    clases detectadas con confianza > 0.4.

    Devuelve: list[dict] con keys class_id, class_name, confidence, bbox
    """
    path = filedialog.askopenfilename(
        parent=parent_window,
        title="Selecciona una imagen",
        filetypes=[
            ("Imágenes", "*.jpg *.jpeg *.png *.bmp *.webp"),
            ("Todos los archivos", "*.*"),
        ],
    )
    if not path:
        return []

    img = cv2.imread(path)
    if img is None:
        messagebox.showerror("Error", f"No se pudo cargar la imagen:\n{path}")
        return []

    # Redimensionar si es muy grande (mantiene la velocidad razonable)
    h, w = img.shape[:2]
    max_side = 1280
    if max(h, w) > max_side:
        scale = max_side / max(h, w)
        img = cv2.resize(img, (int(w * scale), int(h * scale)))

    results = model(img, verbose=False)

    detections = []
    colors = {}
    CONFIDENCE = 0.4

    for box in results[0].boxes:
        conf = float(box.conf[0])
        if conf < CONFIDENCE:
            continue
        class_id = int(box.cls[0])
        class_name = model.names[class_id]
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        if class_id not in colors:
            random.seed(class_id)
            colors[class_id] = (
                random.randint(50, 255),
                random.randint(50, 255),
                random.randint(50, 255),
            )
        color = colors[class_id]

        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f"{class_name} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(img, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        detections.append({
            "class_id":   class_id,
            "class_name": class_name,
            "confidence": conf,
            "bbox":       (x1, y1, x2, y2),
        })

    # Mostrar resultado
    window_name = f"Detección - {path.split('/')[-1]}"
    try:
        cv2.imshow(window_name, img)
        cv2.waitKey(0)
    except cv2.error:
        pass
    # Si el usuario cierra la ventana con la X (en vez de pulsar tecla),
    # la ventana ya no existe y destroyWindow lanza error. Lo ignoramos.
    try:
        cv2.destroyWindow(window_name)
    except cv2.error:
        pass

    return detections


if __name__ == "__main__":
    # Prueba rápida sin GUI principal
    import tkinter as tk
    root = tk.Tk()
    root.withdraw()
    dets = detect_in_image()
    print(f"Detectados {len(dets)} objetos:")
    for d in dets:
        print(f"  - {d['class_name']} ({d['confidence']:.2f}) en {d['bbox']}")