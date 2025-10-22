clear
# Aperture Priority Mode“ (Blendenpriorität) bedeutet:
# Die Kamera hält die Blende fest und passt Belichtungszeit (und ggf. Gain) automatisch an,
# um ein korrekt belichtetes Bild zu erzeugen.
v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=3   #3 # 1=Manual mode,  3=Aperture Priority Mode
v4l2-ctl -d /dev/video2 --get-ctrl=auto_exposure

# exposure_dynamic_framerate erlaubt oder verbietet der Kamera,
# die tatsächliche Bildrate (Framerate) zu ändern,
# wenn sie versucht, eine korrekte Belichtung zu halten
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_dynamic_framerate=1   #1
v4l2-ctl -d /dev/video2 --get-ctrl=exposure_dynamic_framerate

# exposure_time_absolute kann nur bei Manual Mode gesetzt werden.
#v4l2-ctl -d /dev/video2 --set-ctrl=exposure_time_absolute=150
v4l2-ctl -d /dev/video2 --get-ctrl=exposure_time_absolute

python AsyncHandPoseYolo11.py
