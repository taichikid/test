import socket

# ラズベリーパイのIPアドレスとポート番号
raspberry_pi_ip = 'raspberrypi.local' #'ラズベリーパイのIPアドレス'
raspberry_pi_port = 5555

# ソケットを作成
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# ラズベリーパイに接続
client_socket.connect((raspberry_pi_ip, raspberry_pi_port))

while True:
    # ユーザーからの入力を受け取る
    user_input = input("Enter 0 or 1: ")

    # 入力をラズベリーパイに送信
    client_socket.send(user_input.encode())

# ソケットを閉じる
client_socket.close()
