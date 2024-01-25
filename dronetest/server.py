import socket

def recv_takeoff(server_ip="127.0.0.1", server_port=50000):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    sock.bind((server_ip, server_port))

    sock.listen(1)

    client_sock, _ = sock.accept()

    while True:
        message = client_sock.recv(1024)
        print(message.decode())
        if message.decode() == "takeoff":
            break

    print("起動!")
    sock.close()

if __name__ == "__main__":
    recv_takeoff()