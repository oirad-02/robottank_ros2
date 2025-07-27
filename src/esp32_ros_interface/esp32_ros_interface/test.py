import socket

HOST = '0.0.0.0'  # horche auf allen Interfaces (inkl. 192.168.10.1)
PORT = 4242       # muss mit ESP32-Code übereinstimmen

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"Server läuft auf {HOST}:{PORT}, warte auf ESP32...")

    conn, addr = s.accept()
    with conn:
        print(f"Verbunden mit ESP32: {addr}")
        while True:
            try:
                # Sende Test-Daten an ESP32
                test_data = "[1,2,3]\n"
                conn.sendall(test_data.encode())
                print(f"Gesendet: {test_data.strip()}")

                # Antwort empfangen
                data = conn.recv(64)
                if not data:
                    print("ESP32 getrennt.")
                    break
                print(f"Antwort vom ESP32: {data.decode().strip()}")
            except Exception as e:
                print(f"Fehler: {e}")
                break