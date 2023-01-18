#!/usr/bin/env python
# import argparse
import os.path

# PKG = 'numpy_tutorial'
# import roslib; roslib.load_manifest(PKG)
import rospy
from std_msgs.msg import Int32
# from rospy_tutorials.msg import Floats
# from rospy.numpy_msg import numpy_msg

from d435_prueba.msg import PuntosObstaculo
from d435_prueba.msg import Obstaculo
from d435_prueba.msg import GrupoObstaculos
import pyrealsense2 as rs
import numpy as np
import cv2

def cb_apagar_nano(comando):
    print(comando)
    if comando.data==1:
        os.system('shutdown -h now')
        print("si ha llegado")

try:

    pub = rospy.Publisher('pub_grupo_obs', GrupoObstaculos,queue_size=10)
    rospy.init_node('realsense_obs')
    rospy.Subscriber("apagar_nano", Int32, cb_apagar_nano,queue_size=100)
    #r = rospy.Rate(30) # 10hz
    #puntos = PuntosObstaculo()
    #obstaculo=Obstaculo()
    #msg = GrupoObstaculos()

    pipeline = rs.pipeline()

    config = rs.config()

    #=================Diferentes resoluciones de camara============================
    #config.enable_stream(rs.stream.depth,848,480, rs.format.z16, 30)
    #config.enable_stream(rs.stream.depth,640,480, rs.format.z16, 30)
    #config.enable_stream(rs.stream.depth,640,360, rs.format.z16, 30)
    #config.enable_stream(rs.stream.depth,480,270, rs.format.z16, 30)
    config.enable_stream(rs.stream.depth,424,240, rs.format.z16, 15)
    #=================Diferentes resoluciones de camara============================

    config.enable_stream(rs.stream.infrared)
    
     # Create colorizer object
    colorizer = rs.colorizer(3)

    # Start streaming
    pipeline.start(config)
    
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height
    depth_intrinsics.model=rs.distortion.none
   
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        df=frames.get_depth_frame().as_depth_frame()
        infra_frame=frames.get_infrared_frame()

        depth_color_frame = colorizer.colorize(depth_frame)

        depth_color_image = np.asanyarray(depth_color_frame.get_data())
        infra_image=np.asanyarray(infra_frame.get_data())

        infra_image=cv2.cvtColor(infra_image,cv2.COLOR_GRAY2BGR)

        gray_img=cv2.cvtColor(depth_color_image,cv2.COLOR_BGR2GRAY)
        alto, ancho=gray_img.shape

        rojos=[]

        depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height
        depth_intrinsics.model=rs.distortion.none


        #===============PARTE DE LA PANTALLA=====================
        inicio_muestra_ancho=ancho//16
        inicio_muestra_alto=alto//16*3

        for x in range(inicio_muestra_ancho,inicio_muestra_ancho*15,20):
            for y in range(inicio_muestra_alto,alto):  
        #===============PARTE DE LA PANTALLA=====================
        

        #===============TODA LA PANTALLA=====================
        # inicio_muestra_ancho=1
        # inicio_muestra_alto=1

        # for x in range(inicio_muestra_ancho,ancho,35):
        #     for y in range(inicio_muestra_alto,alto):
        
        #===============TODA LA PANTALLA=====================        

                if y!=inicio_muestra_alto:
                    pixel_actual= gray_img.item(y,x)
                    if pixel_anterior!=0 and pixel_actual>pixel_anterior:    
                        # array_distancia=depth_frame.get_distance(x,y)*escala_distancia*100 
                        array_distancia_metros=depth_frame.get_distance(x,y)                       
                        array_distancia=depth_frame.get_distance(x,y)*100
                        pixel_anterior=pixel_actual

                        if array_distancia>0 and array_distancia<=200:
                            infra_image.itemset((y,x,2),255-array_distancia) 
                            infra_image.itemset((y,x,1),0)
                            infra_image.itemset((y,x,0),0)
                            rojos.append([x,y,array_distancia])
                    else:
                        pixel_anterior=pixel_actual
                else:
                    pixel_anterior=gray_img.item(y,x) 

        np_rojos=np.asanyarray(rojos)
        if len(np_rojos)!=0:
            np_rojos= np_rojos[np.argsort(np_rojos[:, 2])]

        grupo=[]
        if np_rojos.size!=0:
            grupo.append(np_rojos[0])
        acum=[]
        minimos=[]
        maximos=[]
        rectangulos=[]

        #puntos = PuntosObstaculo()
        obstaculo=Obstaculo()
        msg = GrupoObstaculos()
        
        umbral_x=150
        umbral_y=40
        umbral_distancia=5
        umbral_puntos=5
        color=(255,0,0)

        for i in range(np_rojos.shape[0]-1):
            if abs(np_rojos[i+1][2]-np_rojos[i][2])<=umbral_distancia and abs(np_rojos[i+1][0]-np_rojos[i][0])<=umbral_x:                    
                grupo.append(np_rojos[i+1])
                #puntos.x=int(np_rojos[i+1,0])
                #puntos.y=int(np_rojos[i+1,1])
                #puntos.d=int(np_rojos[i+1,2])
                #obstaculo.array_puntos.append(puntos)
            
            else:
                if len(grupo)>umbral_puntos:
                    # promedios=np.mean(grupo,axis=0)
                    minimos=np.min(grupo,axis=0)
                    p_inicio=int(minimos[0]-3),int(minimos[1]-3)
                    maximos=np.max(grupo,axis=0)
                    p_fin=int(maximos[0]+3),int(maximos[1]+3) 
                    # centro_x=(maximos[0]-minimos[0])//2
                    # centro_y=(maximos[1]-minimos[1])//2
                    # rectangulos.append([centro_x,centro_y,promedios[2]])
                    infra_image = cv2.rectangle(infra_image,p_inicio, p_fin,color, 1)
                    
                    for c in range(len(grupo)):
                        punto3D=rs.rs2_deproject_pixel_to_point(depth_intrinsics, [grupo[c][0], grupo[c][1]],grupo[c][2]/100)
                        puntos = PuntosObstaculo()
                        puntos.x=punto3D[0]
                        puntos.y=punto3D[1]
                        puntos.d=punto3D[2]
                        obstaculo.array_puntos.append(puntos)  
                                      
                    msg.grupo_obstaculos.append(obstaculo)
                    obstaculo=Obstaculo()
                grupo=[]          

        # acum_np=np.asarray(acum)
        #print(acum)

        if rospy.is_shutdown!=True:
            pub.publish(msg)
        
        # if len(rectangulos)>0:
        #     rectangulos_np=np.asanyarray(rectangulos) 
        #     if  len(rectangulos)>4:
        #         for i in range(4):
        #             print(rectangulos[i])
        #     else:
        #         for i in range(len(rectangulos)):
        #             print(rectangulos[i])
   
        cv2.imshow("Depth Stream", infra_image)
        key = cv2.waitKey(1)
        if key == 27:
            cv2.destroyAllWindows()
            break
        #elif key==32:
            #playback.pause()
            #while True:
                #key = cv2.waitKey(1)
                #if key==32:
                    #playback.resume()
                    #break    

finally:
    pass
    

