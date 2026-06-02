import socket
import struct
import sys
from params import Udp

listen_ip="0.0.0.0" # lausche auf alle Netzwerk-Interfaces
buffer_size=2048


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind((listen_ip, Udp.PORT))
except Exception as e:
    print(f"Fehler beim Binden des Sockets: {e}")
    sys.exit(1)
sock.setblocking(False)
print(f"Warte auf UDP-Pakete auf {listen_ip}:{Udp.PORT}...")
    

def UdpReceive():
    """Startet einen UDP-Server, der Daten empfängt und dekodiert."""

    try:
        packet, addr = sock.recvfrom(buffer_size)
    except BlockingIOError:
        # Dieser Fehler tritt auf, wenn der Puffer komplett leer ist.
        return
    except Exception as e:
        print(f"UDP Recv Error: {e}")
        return
        
    # Sicherheitscheck: Hat das Paket mindestens die 2 Bytes für den Header und hat es eine gerade Anzahl von Bytes?
    packet_len = len(packet)
    assert packet_len >= 2

    # Paket zerschneiden
    header_bytes = packet[:2]  # Die ersten 2 Bytes
    data_bytes = packet[2:]    # Alles ab Byte 2 bis zum Ende
    
    # Header entpacken
    # '<h' = Little-Endian, 2-Byte Signed Short
    # unpack() gibt immer ein Tupel zurück (z.B. (100,)), daher [0]
    header = struct.unpack('<h', header_bytes)[0]
    
    if header == Udp.TEXT:
        text = data_bytes.decode('utf-8')
        print(text)
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

# Starten
# Achte darauf, dass der Port hier mit UdpPort aus deinem send()-Skript übereinstimmt!
if __name__ == "__main__":
    while True:
        UdpReceive()