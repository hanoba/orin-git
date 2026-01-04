# Erzeugt isaac-sim Simulationsumgebung für meinen Garten.
# Die Gartendaten sind von Moonlight/amcp übernommen.
# HB 2025-12-20

from MapGenerator import PGMMap
import math

def Move(poly, mx, my):
    length = len(poly)
    for i in range(length):
        (x,y) = poly[i]
        poly[i] = (x+mx,y+my)
    return


def Gartenzaun():
    # Messwerte Bosch Laserentfernungsmesser
    a0 = 18.73
    b0 = 15.16
    c0 =  7.86 
    m1 = 30.54
    b2 = 44.35
    c2 = 24.25
    b3 =  0.47
    
    # Dreieck D0
    wa0 = math.acos((b0*b0 + c0*c0 - a0*a0)/(2*b0*c0))
    #print("wa0 =", wa0/math.pi*180)
    
    # Dreieck D1
    wa1 = wa0
    b1 = b0 + m1
    c1 = c0
    a1 = math.sqrt(b1*b1 + c1*c1 - 2*b1*c1*math.cos(wa1))
    #print("a1 =", a1)
    wc1 = math.acos((a1*a1 + b1*b1 - c1*c1)/(2*a1*b1))
    #print("wc1 =", wc1/math.pi*180)
    
    # Dreieck D2
    a2 = a1
    wa2 = math.acos((b2*b2 + c2*c2 - a2*a2)/(2*b2*c2))
    #print("wa2 =", wa2/math.pi*180)
    wb2 = math.acos((a2*a2 + c2*c2 - b2*b2)/(2*a2*c2))
    #print("wb2 =", wb2/math.pi*180)
    
    # Dreieck D3
    c3 = c2
    wa3 = math.pi - wc1 - wb2
    a3 = math.sqrt(b3*b3 + c3*c3 - 2*b3*c3*math.cos(wa3))
    #print("a3 =", a3)
    wb3 = math.acos((a3*a3 + c3*c3 - b3*b3)/(2*a3*c3))
    #print("wb3 =", wb3/math.pi*180)
    
    # Viereck
    wpb = wa2 + wb3
    #print("wpb =", wpb/math.pi*180)
    wpa = 2*math.pi - wa0 - wpb - (math.pi - wa3 - wb3)
    #print("wpa =", wpa/math.pi*180)
    
    
    xo = 0.3
    PA=(xo,0)
    PB=(xo+b2,0)
    
    wb = wpb - math.pi/2
    PC=(xo+b2+a3*math.sin(wb), -a3*math.cos(wb))
    
    wa = wpa - math.pi/2
    PD=(xo-c0*math.sin(wa), -c0*math.cos(wa))

    check=math.sqrt((PD[0]-PC[0])**2+(PD[1]-PC[1])**2)
    #print(check, b1+b3)

    garten=[PA, PB, PC, PD]
    print(f"{garten=}")
    return garten

def Gartenhaus():
    haus = [(0,0),(7.23,0),(7.23,4.67),(0,4.67)]
    mx = 44.70 - 10.30 - 7.23 # 5.10 + 4.00 + 16.60
    my = -4.76-0.5
    Move(haus,mx,my)
    return haus

def Terrasse():
    xx = 7.23-6.08
    #yy = 4.67
    mx = 44.70 - 10.30 - 7.23 # 5.10 + 4.00 + 16.60
    my = -4.76-0.5
    t = [(xx,0),(xx+6.08,0),(xx+6.08,-4.45),(xx,-4.45)]
    Move(t, mx,my)
    return [t[3], t[2], t[1], t[0]]
    
def Schuppen():
    schuppen = [(0,0),(4.00,0),(4.00,4.50),(0,4.50)]
    mx = 5.10 + 1.6 - 0.5
    my = -4.50-0.50 - 0.3
    Move(schuppen,mx,my)
    return schuppen


# -------------------------------------------------------
# Garten
# -------------------------------------------------------
def CreateGarten():
    mapX = 1000
    mapY =  600
    res = 0.05
    map = PGMMap(mapX, mapY, resolution=res)
    map.draw_rectangle(0, 0, mapX, mapY, value=254) # Hintergrund weiß
    
    # Koordinatentransformation
    def c(A):
        x, y = A
        return  (int((x+3)/res+2.5), int((-y+2)/res+2.5))
        
    def ZaunElement(text, A, B):
        ax, ay = c(A)
        bx, by = c(B)
        map.draw_line(ax, ay, bx, by, value=0)
        print(f"{text} ({ax},{ay})   ({bx},{by})")
        
    PA, PB, PC, PD = Gartenzaun()

    # Gartentor hinzufügen
    torAbstand = 8.00
    torBreite = 0.92
    PL = (PB[0]-torAbstand-torBreite, PB[1])    # linker Torpfosten
    PR = (PB[0]-torAbstand, PB[1])              # rechter Torpfosten

    ZaunElement("ZaunAL", PA, PL)
    ZaunElement("ZaunRB", PR, PB)
    ZaunElement("ZaunBC", PB, PC)
    ZaunElement("ZaunCD", PC, PD)
    ZaunElement("ZaunDA", PD, PA)

    def Haus(haus, e):
        print(f"{haus}: {e}")
        ZaunElement("HausAB", e[0], e[1])
        ZaunElement("HausBC", e[1], e[2])
        ZaunElement("HausCD", e[2], e[3])
        ZaunElement("HausDA", e[3], e[0])
    
    Haus("Schuppen", Schuppen())
    Haus("Gartenhaus", Gartenhaus())
    terrasse = Terrasse()
    Haus("Terrasse", terrasse)

    def Bassin(terrasseUntenRechts):
        lenX = 2.40
        lenY = 2.80
        (x, y) = terrasseUntenRechts
        x += 0.50
        return [(x, y), (x+lenX, y), (x+lenX, y+lenY), (x, y+lenY)]
    
    Haus("Bassin", Bassin(terrasse[1]))

    def Strauch(name, mittelPunkt, dm):
        print(f"{name}: {mittelPunkt}  {dm=}")
        posX, posY = c(mittelPunkt)
        radius = int(dm/2/res+0.5)
        map.draw_circle(posX, posY, radius, value=0, fill=False)
    
    tx, ty = terrasse[0]
    grasMP = (tx, ty-8.0)
    grasDM = 1.50
    Strauch("Gras", grasMP, grasDM)

    strauchMP = (tx+13.00, ty-8.00)
    strauchDM = 2.00    
    Strauch("Strauch", strauchMP, strauchDM)

    # Speichern
    map.save_map("garten_map2")

CreateGarten()