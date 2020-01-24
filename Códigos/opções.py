c, img_lin = cv2.threshold(image, 130, 255, cv2.THRESH_BINARY)
propl, propr, ppropl, ppropr = 1, 1, 1, 1
for i in range(0, 640):
    c = img_lin[240, i]

    if (c == 0):
        for j in range(1, 640):
            d = img_lin[479, j]
            if i == j:
                break
            tg = 240 / (i - j)
            if d == 0 and (0 < tg < 0.5):
                if tg < 0:
                    ppropl = fabs(tg)
                else:
                    ppropr = tg
                    break
            else:
                ppropr = 1
                ppropl = 1
                break
        propl = (i / 640)
        propr = ((640 - i) / 640)
        sub = fabs(propr - propl)
        if fabs(sub) < 0.1:
            mult = 10
        if 0.1 < sub < 0.2:
            mult = 9
        if 0.2 < sub < 0.4:
            mult = 8.5
        if 0.4 < sub < 0.6:
            mult = 8
        if 0.6 < sub < 0.75:
            mult = 7.5
        if 0.75 < sub < 0.85:
            mult = 5.5
        if sub > 0.85:
            mult = 4
        l_speed = (propl * ppropl) * mult
        r_speed = (propr * ppropr) * mult
        break

#tg 2
c, img_lin = cv2.threshold(image, 130, 255, cv2.THRESH_BINARY)
    propl, propr, ppropl, ppropr = 1, 1, 1, 1

    for i in range(0,640):
        c = img_lin[240, i]
        if (c == 0):
            for j in range(1, 640):
                d = img_lin[479, j]
                if i == j:
                    break
                tg = 240 / (i - j)
                #print(tg)
                if d == 0:
                    if tg < 0:
                        ppropl = fabs(tg)
                    else:
                        ppropr = tg
                        break
                else:
                    break

            propl = (i / 640)
            propr = ((640 - i) / 640)
            sub = fabs(propr-propl)
            if sub < 0.1:
                mult = 9
            if 0.1 < sub < 0.2:
                mult = 8.5
            if 0.2 < sub < 0.4:
                mult = 7.5
            if 0.4 < sub < 0.7:
                mult = 7
            if 0.7 < sub < 0.8:
                mult = 6.5
            if sub > 0.8:
                mult = 5
            if ppropl<0.5 or ppropr<0.5:
                l_speed = (propl/ppropl) * mult
                r_speed = (propr/ppropr) * mult

            else:
                l_speed = propl * mult
                r_speed = propr * mult
            break
#opção 3
import vrep
import cv2
import array
import numpy as np
import time
from PIL import Image as I
from math import fabs


print('program started')
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
print('Connected to remote API server')
#ponteiro da camera, do motor esquerdo e do motor esquerdo
r, colorCam = vrep.simxGetObjectHandle(clientID, "kinect_rgb", vrep.simx_opmode_oneshot_wait);
r, leftmotor = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait);
r, rightmotor = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_oneshot_wait);

#para o robô
vrep.simxSetJointTargetVelocity(clientID, leftmotor, 0, vrep.simx_opmode_streaming);
vrep.simxSetJointTargetVelocity(clientID, rightmotor, 0, vrep.simx_opmode_streaming);

#pega a imagem da camera do robo
r, resolution, image = vrep.simxGetVisionSensorImage(clientID, colorCam, 1, vrep.simx_opmode_streaming);
time.sleep(0.5)

l_speed = 0
r_speed = 0

while True:
    r, resolution, image = vrep.simxGetVisionSensorImage(clientID, colorCam, 1, vrep.simx_opmode_buffer);
    mat = np.asarray(image, dtype=np.uint8)
    mat2 = mat.reshape(resolution[1], resolution[0], 1)

    image = cv2.flip(mat2, 0)


    # aqui vocês inserem a parte do controle dos motores definem os valores de l_speed e r_speed de acordo com a imagem
    c, img_lin = cv2.threshold(image, 130, 255, cv2.THRESH_BINARY)
    propl, propr, ppropl, ppropr = 1, 1, 1, 1

    for i in range(0,640):
        cor_meio = img_lin[240, i]
        if (cor_meio == 0):


            propl = (i / 640)
            propr = ((640 - i) / 640)
            sub = fabs(propr-propl)
            if sub < 0.1:
                mult = 9
            if 0.1 < sub < 0.2:
                mult = 8
            if 0.2 < sub < 0.4:
                mult = 7.5
            if 0.4 < sub < 0.7:
                mult = 7
            if 0.7 < sub < 0.8:
                mult = 6.7
            if sub > 0.8:
                mult = 4

            for j in range(1, 640):
                cor_topo = img_lin[479, j]
                if i == j:
                    break
                tg = 240 / (i - j)
                # print(tg)
                if cor_topo == 0:
                    if tg < 0:
                        ppropl = fabs(tg)
                        if ppropl < 0.5 :
                            l_speed = (propl * ppropl) * mult
                            r_speed = (propr / ppropr) * mult
                            break
                    else:
                        ppropr = tg
                        if ppropl < 0.5 or ppropr < 0.5:
                            l_speed = (propl / ppropl) * mult
                            r_speed = (propr * ppropr) * mult
                        break
                else:
                    l_speed = (propl / ppropl) * mult
                    r_speed = (propr / ppropr) * mult
                    break
            break

    # envio dos comandos para os motores
    vrep.simxSetJointTargetVelocity(clientID, leftmotor, l_speed, vrep.simx_opmode_streaming);
    vrep.simxSetJointTargetVelocity(clientID, rightmotor, r_speed, vrep.simx_opmode_streaming);

    cv2.imshow('robot camera', img_lin)
    s=cv2.waitKey(1)
    if s == 27:
        break