import bluetooth

#requires PyBluez and to be running before you run ground station "connect" command

PORT = 1

server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_sock.bind(("", PORT))
server_sock.listen(1)

print("Waiting for connection on RFCOMM channel", PORT)
client_sock, addr = server_sock.accept()
print("Connected to", addr)

try:
    while True:
        data = client_sock.recv(1024).decode().strip()
        if not data:
            continue

        print("Received:", data)

        if data.startswith("ATT:"):
            # Parse attitude
            parts = data.replace("ATT:", "").split(",")
            vals = {kv.split("=")[0]: int(kv.split("=")[1]) for kv in parts}
            yaw, pitch, roll = vals["Y"], vals["P"], vals["R"]
            print(f"Attitude received: Y={yaw}, P={pitch}, R={roll}")
            # <-- Use yaw, pitch, roll here for code

        elif data == "CMCAL":
            print("CMCAL")
        elif data == "MMOICAL":
            print("MMOICAL")
        elif data == "SENSEM":
            print("SENSOR MODE")

except KeyboardInterrupt:
    print("Shutting down.")

client_sock.close()
server_sock.close()
