import socket
import struct
import sys
from params import Udp

class UdpReceive:
    def __init__(self, port):
        self.receivedAddr = None
        self.buffer_size=2048
        
        # letztes Teleop-Kommando 3x wiederholen        
        self.teleopRepeatValue=3
        self.teleopRepeatCnt=0
        self.vLinearLast=0.0
        self.omegaLast=0.0

        listen_ip="0.0.0.0" # lausche auf alle Netzwerk-Interfaces
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((listen_ip, port))
        except Exception as e:
            print(f"Fehler beim Binden des Sockets: {e}")
            sys.exit(1)
        self.sock.setblocking(False)
        print(f"Warte auf UDP-Pakete auf {listen_ip}:{port}...")
    
    def Receive(self, debug=False):
        """Startet einen UDP-Server, der Daten empfängt und dekodiert."""
        try:
            packet, (addr, port) = self.sock.recvfrom(self.buffer_size)
        except BlockingIOError:
            # Dieser Fehler tritt auf, wenn der Puffer komplett leer ist.
            return
        except Exception as e:
            print(f"UDP Recv Error: {e}")
            return
            
        # Sicherheitscheck: Hat das Paket mindestens die 2 Bytes für den Header und hat es eine gerade Anzahl von Bytes?
        packet_len = len(packet)
        assert packet_len >= 2

        # Adresse merken
        self.receivedAddr = addr
        
        # Paket zerschneiden
        header_bytes = packet[:2]  # Die ersten 2 Bytes
        data_bytes = packet[2:]    # Alles ab Byte 2 bis zum Ende
        
        # Header entpacken
        # '<h' = Little-Endian, 2-Byte Signed Short
        # unpack() gibt immer ein Tupel zurück (z.B. (100,)), daher [0]
        header = struct.unpack('<h', header_bytes)[0]
        
        if header == Udp.TEXT:
            text = data_bytes.decode('utf-8')
            if debug: print(f"[UdpReceive] {text}")
            return header, text
            
        # Daten-Array entpacken
        # Wir müssen wissen, wie viele 'h' (16-Bit / 2-Byte Integer) wir erwarten.
        # Dafür teilen wir die restlichen Bytes durch 2.
        data_len = len(data_bytes) // 2
        assert data_len*2 == (packet_len-2)
        if data_len <= 0:
            return header, []
        
        # Format-String bauen (z.B. '<4h')
        format_string = f'<{data_len}h'
        
        # Daten entpacken (Ergebnis ist ein Tupel)
        data_tuple = struct.unpack(format_string, data_bytes)
        
        # Optional: Wieder in eine echte Python-Liste umwandeln
        data_list = list(data_tuple)
        
        # Ergebnis ausgeben
        #print(f"Header: {header} | Daten: {data_list}")    
            
        return header, data_list

    def ReceivedAddr(self):
        return self.receivedAddr

    def ReceiveTeleop(self, vLinear, omega):
        data = self.Receive()
        if data is None:
            if self.teleopRepeatCnt > 0:
                self.teleopRepeatCnt -= 1
                return self.vLinearLast, self.omegaLast
            return vLinear, omega
        
        print(f"Teleop received {data}")
        self.teleopRepeatCnt = self.teleopRepeatValue
        header, data_list = data
        assert header == Udp.TELEOP
        assert len(data_list) == 2
        self.vLinearLast = data_list[0]/1000.0
        self.omegaLast = data_list[1]/1000.0
        return self.vLinearLast, self.omegaLast


# Starten
# Achte darauf, dass der Port hier mit UdpPort aus deinem send()-Skript übereinstimmt!
if __name__ == "__main__":
    udp = UdpReceive()
    while True:
        udp.Receive()