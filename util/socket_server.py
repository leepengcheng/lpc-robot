#coding:utf-8
import struct
import socket

#开启ip和端口
ip_port = ('127.0.0.1',8080)
#生成句柄
web = socket.socket()
#绑定端口
web.bind(ip_port)
#最多连接数
web.listen(1)
#等待信息
conn=None

#开启死循环
while True:
    #阻塞
    while True:
        conn,addr = web.accept()
        if conn is not None:
            print("%s connect"%addr)
            break
        else:
            print ('nginx waiting...')
    #获取客户端请求数据
    data = conn.recv(1024)
    #打印接受数据 注：当浏览器访问的时候，接受的数据的浏览器的信息等。
    print(data)
    #向对方发送数据
    ss=struct.pack('if',3,3.14)
    conn.send(ss)
    #关闭链接    
    conn.close()




#！/usr/bin/env python
#coding:utf-8
import socket
#链接服务端ip和端口
ip_port = ('127.0.0.1',8081)
#生成一个句柄
sk = socket.socket()
#请求连接服务端
sk.connect(ip_port)
#发送数据
sk.sendall(bytes('yaoyao'))
#接受数据
server_reply = sk.recv(1024)
#打印接受的数据
print(struct.unpack('if',server_reply))
#关闭连接
sk.close()