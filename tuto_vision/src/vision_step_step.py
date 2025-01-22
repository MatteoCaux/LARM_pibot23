import cv2
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
# from skimage.measure import label, regionprops
import math
import os 

# connect to a sensor (0: webcam)
pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)

config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

# Start streaming
pipeline.start(config)


def filtre_vert(image_couleur):
    hsv = cv2.cvtColor(image_couleur, cv2.COLOR_BGR2HSV)
    
    #min et max du vert
    min_vert = np.array([40,100,50])
    max_vert = np.array([85,255,255])

    mask_vert=cv2.inRange(hsv,min_vert,max_vert)
    mask_vert = cv2.GaussianBlur(mask_vert,(11,11),0)

    kernel= np.ones((5,5),np.uint8)

    closing=cv2.morphologyEx(mask_vert,cv2.MORPH_OPEN,kernel)

    res2_vert = cv2.cvtColor(cv2.bitwise_and(hsv,hsv,mask=closing),cv2.COLOR_HSV2BGR)

    return closing,res2_vert

def contours(image_couleur,image_neutre,mask,vert,depth_frame):
    contours=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(contours) > 0:
        for c in contours :
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            rect=cv2.boundingRect(c)
            #print(rect)
            
            #print(rayon)
            coin=rect[:2]
            largeur=rect[2]
            hauteur=rect[3]
            if rayon>20:
                x_max=coin[0]+largeur+10 if coin[0]+largeur+10 < image_couleur.shape[1] else coin[0]+largeur
                y_max=coin[1]+hauteur + 10 if coin[1]+ + 10 < image_couleur.shape[0] else coin[1]+hauteur
                y_min = coin[1] - 10 if coin[1] -10 > 0 else coin[1]
                x_min = coin[0] - 10 if coin[0] -10 > 0 else coin[0]
                crop = image_neutre[y_min:y_max,x_min:x_max]

                cv2.circle(vert, (int(x), int(y)), int(rayon), color_info, 2)
                cv2.rectangle(image_couleur, coin, (coin[0]+largeur, coin[1]+hauteur),color_info, 2)
                

                if crop.shape[0] !=0 and crop.shape[1]!=0:
                    return crop, rayon
    return None, 0

def template_matching(template_folder_path, color_image):
    img_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # # charger le template de l'objet à rechercher
    # templates = os.listdir(template_folder_path)
    # print(templates)

    # for t in templates : 
    #     template = cv2.imread(template_folder_path+t,0)
    #     # template = cv2.resize(template,(512,384))
    #     # Récupération des dimensions de l'image
    #     w, h = template.shape[::-1]

    #     # Application du template matching
    #     res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)

    #     # Sélection des meilleurs matched objects
    #     threshold = 0.5
    #     loc = np.where( res >= threshold)

    #     # Affichage de la boite englobante de chaque objet détecté
    #     for pt in zip(*loc[::-1]):
    #         cv2.rectangle(color_image, pt, (pt[0] + w, pt[1] + h), (255,0,0), 2)

    # template = cv2.imread(template_folder_path+templates[0],0)
    template = cv2.imread(template_folder_path+'FACE_2.jpg',0)

    # template = cv2.resize(template,(512,384))
    # Récupération des dimensions de l'image
    w, h = template.shape[::-1]

    # Application du template matching
    res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)

    # Sélection des meilleurs matched objects
    threshold = 0.5
    loc = np.where( res >= threshold)

    # Affichage de la boite englobante de chaque objet détecté
    for pt in zip(*loc[::-1]):
        cv2.rectangle(color_image, pt, (pt[0] + w+2, pt[1] + h+2), (255,0,0), 2)


def souris(event, x, y, flags, param):
    global lo, hi, color, hsv_px

    if event == cv2.EVENT_MOUSEMOVE:
        # Conversion des trois couleurs RGB sous la souris en HSV
        px = color_image[y,x]
        px_array = np.uint8([[px]])
        hsv_px = cv2.cvtColor(px_array,cv2.COLOR_BGR2HSV)

    if event==cv2.EVENT_MBUTTONDBLCLK:
        color=image[y, x][0]

    if event==cv2.EVENT_LBUTTONDOWN:
        if color>5:
            color-=1

    if event==cv2.EVENT_RBUTTONDOWN:
        if color<250:
            color+=1

    lo[0]=color-10
    hi[0]=color+10

def depth_of_selection(x,y,depth_frame):
    if depth_frame:
        depth = depth_frame.get_distance(x,y)
        return([x,y,depth])

def voir_carre_noir(image_crop,image_couleur, rayon):
    hsv = cv2.cvtColor(image_crop, cv2.COLOR_BGR2HSV)
    
    #min et max du noir
    min_noir = np.array([0,0,0])
    max_noir = np.array([255,255,60])

    mask_noir=cv2.inRange(hsv,min_noir,max_noir)

    kernel= np.ones((5,5),np.uint8)

    mask_noir=cv2.morphologyEx(mask_noir,cv2.MORPH_OPEN,kernel)

    contours=cv2.findContours(mask_noir, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(contours) > 0:
        for c in contours :
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            rect=cv2.boundingRect(c)

            coin=rect[:2]
            largeur=rect[2]
            hauteur=rect[3]
            carre = largeur/hauteur

            ratio = rayon/rayon_vert
            print("Le ratio noir est de "+str(ratio))
            

            if (ratio > 0.1 and ratio < 0.2) and (carre > 0.8 and carre < 1.2) :
                cv2.circle(image_crop, (int(x), int(y)), int(rayon), color_info, 2)
    return mask_noir


def voir_yeux_blanc(image_crop,image_couleur,rayon_vert):
    hsv = cv2.cvtColor(image_crop, cv2.COLOR_BGR2HSV)
    
    #min et max du vert
    min_blanc = np.array([80,30,140])
    max_blanc = np.array([255,110,255])

    mask_blanc=cv2.inRange(hsv,min_blanc,max_blanc)

    kernel= np.ones((5,5),np.uint8)

    # mask_blanc=cv2.morphologyEx(mask_blanc,cv2.MORPH_OPEN,kernel)

    # res_blanc = cv2.cvtColor(cv2.bitwise_and(hsv,hsv,mask=mask_blanc),cv2.COLOR_HSV2BGR)
    contours=cv2.findContours(mask_blanc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(contours) > 0:
        for c in contours :
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            ratio = rayon/rayon_vert
            print("Le ratio blanc est de "+str(ratio))
            if rayon/rayon_vert < 0.35 and rayon/rayon_vert > 0.25:
                cv2.circle(image_crop, (int(x), int(y)), int(rayon), color_info, 2)

    return mask_blanc



color=100

lo=np.array([color-5, 100, 50])
hi=np.array([color+5, 255,255])

color_info=(0, 0, 255)



align_to = rs.stream.depth
align = rs.align(align_to)

cap=cv2.VideoCapture(0)
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', souris)
hsv_px = [0,0,0]


while True :
    # Wait for a coherent tuple of frames: depth, color and accel
    frames = pipeline.wait_for_frames()

    color_frame = frames.first(rs.stream.color)

    aligned_frames =  align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    aligned_color_frame = aligned_frames.get_color_frame()

    # color_image = np.asanyarray(aligned_color_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    color_image_neutre=color_image.copy()

    # Affichage des composantes HSV sous la souris sur l'image
    pixel_hsv = " ".join(str(values) for values in hsv_px)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(color_image, "px HSV: "+pixel_hsv, (10, 260),
               font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    
    mask,vert=filtre_vert(color_image)

    image_cropped=None
    image_cropped, rayon_vert=contours(color_image,color_image_neutre,mask,vert,depth_frame)


    if image_cropped is not None :
        #template_matching("./tuto_vision/template/",image_cropped)
        blanc=voir_yeux_blanc(image_cropped,color_image,rayon_vert)
        black=voir_carre_noir(image_cropped,color_image,rayon_vert)
        cv2.imshow("crop",image_cropped)
        cv2.imshow("crop_noir",black)
        cv2.imshow("crop_blanc",blanc)

    # Display cropped image
    cv2.imshow("Camera", color_image)
    # cv2.imshow('Mask', mask)
    # cv2.imshow("Vert",vert)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break