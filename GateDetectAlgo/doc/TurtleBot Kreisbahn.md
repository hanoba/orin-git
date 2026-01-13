# Kreisbahn eines TurtleBots

## 1. Grundlagen (Differential-Drive-Kinematik)

Ein TurtleBot besitzt zwei angetriebene Räder mit Achsabstand **b** und Radgeschwindigkeiten **vₗ** (links) und **vᵣ** (rechts).

Gesamtgeschwindigkeit und Drehgeschwindigkeit:

\[
v = \frac{v_R + v_L}{2}, \qquad \omega = \frac{v_R - v_L}{b}
\]

---

## 2. Kreisbahn

Wenn **vₗ ≠ vᵣ**, bewegt sich der Roboter auf einem Kreis mit Radius

\[
R = \frac{v}{\omega} = \frac{b}{2}\frac{v_L + v_R}{v_R - v_L}
\]

Der Radius **R** ist der Abstand vom Mittelpunkt zwischen den Rädern zum Zentrum der Kreisbahn.

---

## 3. ICC (Instantaneous Center of Curvature)

**ICC** = *Momentaner Krümmungsmittelpunkt* = Mittelpunkt der momentanen Kreisbahn, um den sich der Roboter dreht.

### Sonderfälle

| Bedingung | Bewegung | ICC |
|------------|-----------|-----|
| vₗ = vᵣ | Geradeausfahrt | im Unendlichen |
| vₗ ≠ vᵣ | Kreisbahn | endlicher Punkt seitlich des Roboters |
| vₗ = -vᵣ | Drehung auf der Stelle | im Zentrum des Roboters |

---

## 4. Bewegungsgleichungen

Startpose: \((x_0, y_0, \theta_0)\)

\[
\begin{aligned}
x_{\text{icc}} &= x_0 - R\sin\theta_0 \\
y_{\text{icc}} &= y_0 + R\cos\theta_0
\end{aligned}
\]

\[
\theta(t) = \theta_0 + \omega t
\]

\[
x(t) = x_{\text{icc}} + R\sin\theta(t), \qquad y(t) = y_{\text{icc}} - R\cos\theta(t)
\]

---

## 5. Diskrete Updateformel (Python)

```python
import numpy as np

def dd_update(x, y, th, vL, vR, b, dt):
    v = 0.5 * (vR + vL)
    w = (vR - vL) / b
    dth = w * dt
    if abs(w) > 1e-9:
        dx = (v / w) * (np.sin(th + dth) - np.sin(th))
        dy = (v / w) * (-np.cos(th + dth) + np.cos(th))
    else:
        dx = v * dt * np.cos(th)
        dy = v * dt * np.sin(th)
    return x + dx, y + dy, th + dth
