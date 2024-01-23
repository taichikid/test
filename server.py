import socket

# ラズベリーパイのIPアドレスとポート番号
raspberry_pi_ip = 'raspberrypi.local' #'ラズベリーパイのIPアドレス'
raspberry_pi_port = 5555

# ソケットを作成
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((raspberry_pi_ip, raspberry_pi_port))
server_socket.listen(1)

print(f"Waiting for connection on {raspberry_pi_ip}:{raspberry_pi_port}")

# クライアントからの接続を待機
client_socket, client_address = server_socket.accept()
print(f"Connected to {client_address}")

while True:
    # クライアントからデータを受信
    data = client_socket.recv(1024).decode()

    if not data:
        break

    # 受信したデータを処理（0ならTrue、1ならFalse）
    result = True if data == '1' else False

    # 結果をラズベリーパイのコンソールに表示
    print(f"Received data: {data}, Result: {result}")

# ソケットを閉じる
client_socket.close()
server_socket.close()
