from threading import Thread
import socket  # 导入 socket 模块
import socket
import sys
import struct
import matplotlib.pyplot as plt
import json


# 关闭画图的窗口


SEND_BUF_SIZE = 256

RECV_BUF_SIZE = 256

Communication_Count: int = 0

receive_count: int = 0

ADDRESS = ('127.0.0.1', 8888)  # 绑定地址

g_socket_server = None  # 负责监听的socket

g_conn_pool = []  # 连接池


def init():
    """
    初始化服务端
    """
    global g_socket_server
    g_socket_server = socket.socket(
        socket.AF_INET, socket.SOCK_STREAM)  # 创建 socket 对象
    g_socket_server.bind(ADDRESS)
    g_socket_server.listen(5)  # 最大等待数（有很多人理解为最大连接数，其实是错误的）
    print("服务端已启动，等待客户端连接...")


def accept_client():
    """
    接收新连接
    """
    while True:
        client, _ = g_socket_server.accept()  # 阻塞，等待客户端连接
        # 加入连接池
        g_conn_pool.append(client)
        # 给每个客户端创建一个独立的线程进行管理
        thread = Thread(target=message_handle, args=(client,))
        # 设置成守护线程
        thread.setDaemon(True)
        thread.start()


def message_handle(client):
    """
    消息处理
    """
    client.sendall("连接服务器成功!".encode(encoding='utf8'))
    while True:
        bytes = client.recv(1024)
        print("客户端消息:", bytes.decode(encoding='utf8'))
        if len(bytes) == 0:
            client.close()
            # 删除连接
            g_conn_pool.remove(client)
            print("有一个客户端下线了。")
            break


def start_tcp_server(ip, port):
    ax = []                    # 定义一个 x 轴的空列表用来接收动态的数据
    ay = []                    # 定义一个 y 轴的空列表用来接收动态的数据
    plt.ion()                  # 开启一个画图的窗口
    icnt = 0
    # create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (ip, port)

    # bind port
    print("starting listen on ip %s, port %s" % server_address)
    sock.bind(server_address)

    while True:
        data, addr = sock.recvfrom(1024)
        print("Receive from %s:%s" % addr + str(data))

        user_dic = json.loads(data)
        ax.append(icnt)           # 添加 i 到 x 轴的数据中
        icnt += 1
        ay.append(user_dic["ob.theta"])        # 添加 i 的平方到 y 轴的数据中
        plt.clf()              # 清除之前画的图
        plt.plot(ax, ay)        # 画出当前 ax 列表和 ay 列表中的值的图形
        plt.pause(0.01)
        plt.ioff()

    print(" close client connect ")


if __name__ == '__main__':
    init()
    # 新开一个线程，用于接收新连接
    thread = Thread(target=accept_client)
    thread.setDaemon(True)
    thread.start()
    # 主线程逻辑
    while True:
        cmd = input("""--------------------------
输入1:查看当前在线人数
输入2:给指定客户端发送消息
输入3:关闭服务端
""")
        if cmd == '1':
            print("--------------------------")
            print("当前在线人数：", len(g_conn_pool))
        elif cmd == '2':
            print("--------------------------")
            index, msg = input("请输入“索引,消息”的形式：").split(",")
            g_conn_pool[int(index)].sendall(msg.encode(encoding='utf8'))
        elif cmd == '3':
            exit()
