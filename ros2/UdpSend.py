import subprocess
import sys
import socket
import struct
import platform
from params import Udp
import errno

IpDesktopWohnung = "127.0.0.1" #"192.168.178.42"
IpLaptopWohnung  = "192.168.178.104"
IpLaptopGarten   = "192.168.20.41"

udpIp1 = IpLaptopGarten     # Laptop Garten
udpIp2 = None               # kein Desktop im Garten
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def is_wsl():
    # Gibt True zurück, wenn wir in WSL sind
    return 'microsoft' in platform.release().lower() or 'wsl' in platform.release().lower()    

def get_active_ssid():
    """Liest die aktive WLAN-SSID über nmcli aus."""
    if is_wsl(): return "wsl"
    
    try:
        # nmcli ausführen und Ausgabe abfangen
        result = subprocess.run(
            ['nmcli', '-t', '-f', 'active,ssid', 'dev', 'wifi'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        
        # Zeilenweise durchsuchen (ersetzt grep, cut und head)
        for line in result.stdout.splitlines():
            if line.startswith('ja:') or line.startswith('yes:'):
                # Teilt den String am ersten ':' und gibt den zweiten Teil (die SSID) zurück
                return line.split(':', 1)[1]
                
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        # Greift, falls nmcli fehlschlägt oder nicht installiert ist
        print(e)
        
    return None

   
ssid = get_active_ssid()
if ssid:
    if ssid != "AndroidHanoba":
        udpIp1 = IpLaptopWohnung    # Laptop Wohnung
        udpIp2 = IpDesktopWohnung   # AZ-KENO Wohnung
        
    # Optional: Zur Kontrolle ausgeben
    print(f"Erfolg: SSID={ssid}, {udpIp1=}, {udpIp2=}")
else:
    print("Fehler: SSID konnte nicht gefunden werden.", file=sys.stderr)
    sys.exit(1)


def sendto(packet):
    # Sicherheits-Block fängt "[Errno 101] Network is unreachable" ab
    try:
        sock.sendto(packet, (udpIp1, Udp.PORT))   
        if udpIp2 is not None: sock.sendto(packet, (udpIp2, Udp.PORT)) 
        
    except OSError as e:
        # Wenn es der spezifische WSL2-Netzwerkfehler ist: Ignorieren!
        if e.errno == errno.ENETUNREACH: # ENETUNREACH ist Fehler 101
            print("WSL2 Netzwerk-Schluckauf (Errno 101). Frame wird übersprungen...")
        else:
            # Bei anderen Systemfehlern trotzdem warnen, aber NICHT abstürzen
            print(f"OS Fehler beim Senden: {e}")
            
    except Exception as e:
        # Fängt alle anderen unerwarteten Fehler ab, damit der Simulator am Leben bleibt
        print(f"Unerwarteter Sende-Fehler: {e}")       
      

def UdpSend(header, data):
    # '<h bedeutet: '<' = Little-Endian, 'h' = signed short (2 Bytes)
    # Falls du einen unsigned integer brauchst, nutze '<H'
    header_bytes = struct.pack('<h', header)

    # 1. Format-String erstellen
    # '<' bedeutet Little-Endian 
    # 'h' steht für "short" (das ist C-Sprech für einen 16-Bit Integer)
    # Wir multiplizieren das 'h' mit der Länge der Liste (ergibt hier '>4h')
    format_string = f'<{len(data)}h'

    # 2. Packen
    # Das Sternchen (*) entpackt die Liste, da pack() einzelne Argumente erwartet
    data_bytes = struct.pack(format_string, *data)

    # Zusammenfügen
    packet = header_bytes + data_bytes

    # Senden
    sendto(packet)

def UdpPrint(text):
    # '<h bedeutet: '<' = Little-Endian, 'h' = signed short (2 Bytes)
    # Falls du einen unsigned integer brauchst, nutze '<H'
    header = Udp.TEXT
    header_bytes = struct.pack('<h', header)
    text_bytes = text.encode('utf-8')

    # Zusammenfügen
    packet = header_bytes + text_bytes

    # Senden
    sendto(packet)
