import matplotlib.pyplot as plt
from rplidar import RPLidar

def plot_lidar_data(scan_data):
    angles = [angle for (_, angle, _) in scan_data]
    distances = [distance for (_, _, distance) in scan_data]

    plt.clf()
    plt.polar(angles, distances, 'bo-', alpha=0.75)
    plt.title('RPLiDAR S3M1 Scan')
    plt.pause(0.01)

# RPLiDARのデバイスポートを指定してインスタンスを作成
lidar = RPLidar('COM6')  # ポートは適切に変更してください

try:
    # デバイスの情報を表示
    print(lidar.info)

    # デバイスの状態を表示
    print(lidar.health)

    # スキャンを開始
    lidar.start_motor()
    lidar.start_scan()

    # グラフをリアルタイムでプロット
    for scan in lidar.iter_scans():
        plot_lidar_data(scan)

except KeyboardInterrupt:
    # Ctrl+Cが押されたときに処理を停止
    pass

finally:
    # スキャンを停止してモーターを停止
    lidar.stop_scan()
    lidar.stop_motor()
    lidar.disconnect()

plt.show()
