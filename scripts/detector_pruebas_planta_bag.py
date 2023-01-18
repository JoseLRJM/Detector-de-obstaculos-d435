#!/usr/bin/env python


from email.charset import add_charset
import pyrealsense2 as rs
import numpy as np
import cv2
# import argparse
# import os.path
from std_msgs.msg import String
# PKG = 'numpy_tutorial'
# import roslib; roslib.load_manifest(PKG)
import rospy
# from rospy_tutorials.msg import Floats
# from rospy.numpy_msg import numpy_msg

from d435_prueba.msg import PuntosObstaculo
from d435_prueba.msg import Obstaculo
from d435_prueba.msg import GrupoObstaculos

try:

    pub = rospy.Publisher('pub_grupo_obs', GrupoObstaculos,queue_size=10)
    rospy.init_node('Publicador', anonymous=True)
    #r = rospy.Rate(30) # 10hz
    #puntos = PuntosObstaculo()
    #obstaculo=Obstaculo()
    #msg = GrupoObstaculos()

    pipeline = rs.pipeline()

    config = rs.config()

    rs.config.enable_device_from_file(config, "cam_baja1.bag")

    config.enable_stream(rs.stream.depth, rs.format.z16, 30)
    config.enable_stream(rs.stream.infrared)

    pipeline.start(config)

    colorizer = rs.colorizer(3)

    device=pipeline.get_active_profile().get_device()
    playback=rs.playback(device)

    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height
    depth_intrinsics.model=rs.distortion.none
   
    while True:
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

        depth_intrinsics = rs.video_stream_profile(df.profile).get_intrinsics()
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

        puntos = PuntosObstaculo()
        obstaculo=Obstaculo()
        msg = GrupoObstaculos()
        
        umbral_x=120
        umbral_y=40
        umbral_distancia=1
        umbral_puntos=15
        color=(255,0,0)

        for i in range(np_rojos.shape[0]-1):
            if abs(np_rojos[i+1][2]-np_rojos[i][2])<=umbral_distancia and abs(np_rojos[i+1][0]-np_rojos[i][0])<=umbral_x:                    
                grupo.append(np_rojos[i+1])
                # puntos.x=int(np_rojos[i+1,0])
                # puntos.y=int(np_rojos[i+1,1])
                # puntos.d=int(np_rojos[i+1,2])
                # obstaculo.array_puntos.append(puntos)
            
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

                    punto3D1=rs.rs2_deproject_pixel_to_point(depth_intrinsics, [minimos[0], minimos[1]],minimos[2]/100)

                    puntos.x=punto3D1[0]
                    puntos.y=punto3D1[1]
                    puntos.d=punto3D1[2]
                    #print(puntos.d)
                    obstaculo.array_puntos.append(puntos)
                    #print(str(obstaculo))                   
                    punto3D2=rs.rs2_deproject_pixel_to_point(depth_intrinsics, [maximos[0], maximos[1]],maximos[2]/100)

                    puntos.x=punto3D2[0]
                    puntos.y=punto3D2[1]
                    puntos.d=punto3D2[2]
                    obstaculo.array_puntos.append(puntos)
                    
                    #acum.append([punto3D1,punto3D2])
                    #msg.header.frame_id="detector_nano_frame"
                    #msg.header.stamp=rospy.get_rostime()
                    msg.grupo_obstaculos.append(obstaculo)
                    #print(str(msg)) 
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
        elif key==32:
            playback.pause()
            while True:
                key = cv2.waitKey(1)
                if key==32:
                    playback.resume()
                    break    

finally:
    pass
    

