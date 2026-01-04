import numpy as np
import yaml

#   0: Wand (Schwarz)
# 254: Frei (Weiß)
# 205: Unbekannt (Grau)

class PGMMap:
    def __init__(self, width, height, resolution=0.05, default_value=205):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.data = np.full((height, width), default_value, dtype=np.uint8)

    def draw_rectangle(self, x, y, w, h, value=0):
        """Zeichnet ein gefülltes Rechteck."""
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(self.width, x + w), min(self.height, y + h)
        self.data[y1:y2, x1:x2] = value

    def draw_line(self, x0, y0, x1, y1, value=0):
        """Bresenham-Linien-Algorithmus."""
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        err = dx + dy
        while True:
            if 0 <= x0 < self.width and 0 <= y0 < self.height:
                self.data[y0, x0] = value
            if x0 == x1 and y0 == y1: break
            e2 = 2 * err
            if e2 >= dy: err += dy; x0 += sx
            if e2 <= dx: err += dx; y0 += sy

    def draw_circle(self, x_center, y_center, radius, value=0, fill=False):
        """
        Zeichnet einen Kreis.
        x_center, y_center: Mittelpunkt in Pixeln
        radius: Radius in Pixeln
        fill: Wenn True, wird der Kreis ausgefüllt
        """
        x = radius
        y = 0
        err = 1 - x

        while x >= y:
            if fill:
                # Zeichne horizontale Linien zwischen den symmetrischen Punkten, um den Kreis zu füllen
                self.draw_line(x_center - x, y_center + y, x_center + x, y_center + y, value)
                self.draw_line(x_center - x, y_center - y, x_center + x, y_center - y, value)
                self.draw_line(x_center - y, y_center + x, x_center + y, y_center + x, value)
                self.draw_line(x_center - y, y_center - x, x_center + y, y_center - x, value)
            else:
                # Nur die 8 symmetrischen Randpunkte setzen
                points = [
                    (x_center + x, y_center + y), (x_center + y, y_center + x),
                    (x_center - y, y_center + x), (x_center - x, y_center + y),
                    (x_center - x, y_center - y), (x_center - y, y_center - x),
                    (x_center + y, y_center - x), (x_center + x, y_center - y)
                ]
                for px, py in points:
                    if 0 <= px < self.width and 0 <= py < self.height:
                        self.data[py, px] = value

            y += 1
            if err < 0:
                err += 2 * y + 1
            else:
                x -= 1
                err += 2 * (y - x) + 1

    def save_map(self, base_filename):
        """Speichert .pgm (P5 binär) und .yaml Datei."""
        pgm_filename = f"{base_filename}.pgm"
        yaml_filename = f"{base_filename}.yaml"

        with open(pgm_filename, 'wb') as f:
            header = f"P5\n{self.width} {self.height}\n255\n"
            f.write(header.encode('ascii'))
            f.write(self.data.tobytes())

        mode = "trinary"
        origin = [0.0, 0.0, 0.0]
        metadata = {
            'image': pgm_filename,
            'mode': mode,
            'resolution': self.resolution,
            'origin': origin,
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }

        with open(yaml_filename, 'w') as f:
            yaml.dump(metadata, f, default_flow_style=False, sort_keys=False, indent=4)
        print(f"Datei gespeichert: {base_filename}")
        
if __name__ == "__main__":        
    # Karte erstellen
    my_map = PGMMap(400, 400, resolution=0.05)
    my_map.draw_rectangle(0, 0, 400, 400, value=254) # Hintergrund weiß

    # Ein gefüllter Kreis (z.B. eine Säule)
    my_map.draw_circle(100, 100, radius=30, value=0, fill=True)

    # Ein Kreisumriss (z.B. eine Begrenzung)
    my_map.draw_circle(300, 300, radius=50, value=0, fill=False)

    # Speichern
    my_map.save_map("kreis_test")