# 🧭 Lidar-basierter 2D-SLAM – Zusammenfassung

## 1. Ziel

Ziel ist die Nutzung einer **SLAM-Software, die nur 2D-Lidar-Daten** verwendet, also ohne IMU oder Kamera. Typisches Einsatzszenario: **YDLidar TG30** auf einem **Jetson Orin NX**.

---

## 2. Geeignete Open-Source-Projekte

| Name                      | Sprache       | Hauptmerkmale                                                      | Bemerkung                    |
| ------------------------- | ------------- | ------------------------------------------------------------------ | ---------------------------- |
| **Hector-SLAM**           | C++ (ROS)     | Echtzeit-2D-SLAM mit direktem Scan-Matching, keine Odometrie nötig | Robust auf ebenen Flächen    |
| **GMapping**              | C++ (ROS)     | Partikelfilter-SLAM, nutzt Odometrie optional                      | Klassiker, stabil            |
| **Karto-SLAM**            | C++ (ROS)     | Graph-basiert, Loop-Closing integriert                             | Gut für größere Karten       |
| **Cartographer (Google)** | C++ (ROS 1/2) | 2D/3D-SLAM, Pose-Graph-Optimierung, Loop-Closure                   | Sehr präzise, rechenintensiv |
| **BreezySLAM**            | Python/C++    | Minimalistische Implementierung, kein ROS erforderlich             | Ideal für Embedded-Systeme   |

---

## 3. Vergleich der Verfahren

| Kategorie    | Lokales Matching | Globale Optimierung | Loop-Closure | Ressourcenbedarf |
| ------------ | ---------------- | ------------------- | ------------ | ---------------- |
| Hector-SLAM  | ✅                | ❌                   | ❌            | gering           |
| GMapping     | ✅                | teilw.              | ❌            | mittel           |
| Karto-SLAM   | ✅                | ✅                   | ✅            | mittel           |
| Cartographer | ✅                | ✅                   | ✅            | hoch             |
| BreezySLAM   | ✅                | ❌                   | ❌            | sehr gering      |

---

## 4. Begriffserklärung: "Globale Optimierung (Drift über Zeit)"

* **Drift:** Summierte Fehler bei ausschließlich lokalem Scan-Matching.
* **Globale Optimierung:** Korrigiert alle gespeicherten Posen, sobald eine Schleife erkannt wird (**Loop-Closure**).
* Ergebnis: Karte wird nachträglich entzerrt, Positionsfehler werden minimiert.

**Grafisch:**

```
Start o----->---->---->
               ↑
     Ohne Optimierung: Endpunkt driftet vom Start weg
```

```
Start o----->---->---->
               ↑
     Mit Optimierung: Karte wird korrigiert, Schleife geschlossen
```

---

## 5. Empfehlungen für YDLidar TG30 + Jetson Orin NX

| Ziel                          | Empfohlene Software     |
| ----------------------------- | ----------------------- |
| Schnelle Karte ohne Odometrie | **Hector-SLAM**         |
| Große Karte mit Loop-Closing  | **Google Cartographer** |
| Minimaler Aufwand, kein ROS   | **BreezySLAM**          |

---

## 6. Nächste Schritte

1. Installiere **ROS 1 Noetic** oder **ROS 2 Humble**.
2. Konfiguriere Hector-SLAM für dein `/scan`-Topic (TG30).
3. Teste Kartenerstellung mit Echtzeitvisualisierung in `rviz`.
4. Optional: Upgrade auf Cartographer für globale Korrekturen.

---

**Kurzfassung:**
Hector-SLAM = leicht & schnell, Cartographer = präzise & aufwendig, BreezySLAM = einfach & portabel.
