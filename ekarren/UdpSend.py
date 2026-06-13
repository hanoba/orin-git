import socket
import struct
from params import Udp
import errno
import os

class UdpSend:
    def __init__(self, port, ip1, ip2=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port = port

        # Frage Environment-Variablen ab. Falls sie nicht existieren, wird None zurückgegeben.
        self.udpIp1 = ip1
        self.udpIp2 = ip2
        assert self.udpIp1 is not None
        print(f"{self.udpIp1=}  {self.udpIp2=}")

    def sendto(self, packet):
        # Sicherheits-Block fängt "[Errno 101] Network is unreachable" ab
        try:
            self.sock.sendto(packet, (self.udpIp1, self.port))   
            if self.udpIp2 is not None: self.sock.sendto(packet, (self.udpIp2, self.port)) 
            
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
      

    def Send(self, header, data=[]):
        # '<h bedeutet: '<' = Little-Endian, 'h' = signed short (2 Bytes)
        # Falls du einen unsigned integer brauchst, nutze '<H'
        header_bytes = struct.pack('<h', header)

        data_len = len(data)
        if data_len <= 0:
            # nur header senden
            self.sendto(header_bytes)
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
        self.sendto(packet)

    def Print(self, text):
        # '<h bedeutet: '<' = Little-Endian, 'h' = signed short (2 Bytes)
        # Falls du einen unsigned integer brauchst, nutze '<H'
        header = Udp.TEXT
        header_bytes = struct.pack('<h', header)
        text_bytes = text.encode('utf-8')

        # Zusammenfügen
        packet = header_bytes + text_bytes

        # Senden
        self.sendto(packet)
