#coding:utf-8
import websocket
import thread
import time
import vrep
import math
import numpy as np

def on_message(ws, message):
    global clientID,obj,pos
    data=message.split()
    rpy=map(lambda x:float(x)/9.8*math.pi,[data[1],data[0],data[2]])
    rpy[1]=-rpy[1]
    directions={"F":(0,0.01,0),"B":(0,-0.01,0),"L":(-0.01,0,0),"R":(0.01,0,0)}
    for x in directions.keys():
        if x==data[-1]:
           pos+=np.array(directions[x])
           break
    vrep.simxSetObjectOrientation(clientID,obj,-1,rpy,vrep.simx_opmode_oneshot)
    vrep.simxSetObjectPosition(clientID,obj,-1,pos.tolist(),vrep.simx_opmode_oneshot)
    # vrep.simxAddStatusbarMessage(clientID,message,vrep.simx_opmode_oneshot)

def on_error(ws, error):
    print(error)

def on_close(ws):
    global clientID
    vrep.simxFinish(clientID)
    print("### closed ###")

def on_open(ws):
    # def run(*args):
    #     for i in range(3):
    #         time.sleep(1)
    #         ws.send("Hello %d" % i)
    #     time.sleep(1)
    #     ws.close()
    #     print("thread terminating...")
    # thread.start_new_thread(run, ())
    ws.send("hello world")


if __name__ == "__main__":
    vrep.simxFinish(-1) # 关闭所有连接
    clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # 连接到服务端
    websocket.enableTrace(True)
    if clientID==-1:
        print("conenect to server stop")
    #获得遥控器对象
    res,obj=vrep.simxGetObjectHandle(clientID,"Wiimote",vrep.simx_opmode_blocking)
    res,pos=vrep.simxGetObjectPosition(clientID,obj,-1,vrep.simx_opmode_blocking)
    pos=np.array(pos) #转换为numpy
    # 10.74.175.238
    ws = websocket.WebSocketApp("ws://10.74.175.238:5555",
                              on_message = on_message,
                              on_error = on_error,
                              on_close = on_close)
    ws.on_open = on_open
    time.sleep(2) #暂停1秒
    ws.run_forever()