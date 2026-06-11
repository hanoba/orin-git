import socket
import struct
from params import Udp
import errno
import os

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Frage Environment-Variablen ab. Falls sie nicht existieren, wird None zurückgegeben.
udpIp1 = os.environ.get('EKARREN_VIZ_IP1')
udpIp2 = os.environ.get('EKARREN_VIZ_IP2')
assert udpIp1 is not None
print(f"{udpIp1=}  {udpIp2=}")

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
      

def UdpSend(header, data=[]):
    # '<h bedeutet: '<' = Little-Endian, 'h' = signed short (2 Bytes)
    # Falls du einen unsigned integer brauchst, nutze '<H'
    header_bytes = struct.pack('<h', header)

    data_len = len(data)
    if data_len <= 0:
        # nur header senden
        sendto(header_bytes)
        return
        
    # 1. Format-String erstellen
    # '<' bedeutet Little-Endian 
    # 'h' steht für "short" (das ist C-Sprech für einen 16-Bit Integer)
    # Wir multiplizieren das 'h' mit der Länge der Liste (ergibt hier '>4h')
    format_string = f'<{data_len}h'

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
