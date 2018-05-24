#coding:utf-8
import cv2
maker_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
# board=cv2.aruco.CharucoBoard_create(5,9,0.025,0.02,maker_dict)
# img=board.draw((100*6,100*9))


board=cv2.aruco.GridBoard_create(6,9,0.04,0.005,maker_dict)
img=board.draw((6*100,9*100),10,1)


cv2.imshow("",img)
cv2.waitKey(0)