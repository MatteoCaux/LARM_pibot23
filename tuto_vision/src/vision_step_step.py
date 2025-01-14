import cv2
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
# from skimage.measure import label, regionprops
import math

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
    min_vert = np.array([50,95,95])
    max_vert = np.array([75,180,180])

    mask_vert=cv2.inRange(hsv,min_vert,max_vert)
    mask_vert = cv2.GaussianBlur(mask_vert,(11,11),0)

    kernel= np.ones((5,5),np.uint8)

    closing=cv2.morphologyEx(mask_vert,cv2.MORPH_OPEN,kernel)

    res2_vert = cv2.cvtColor(cv2.bitwise_and(hsv,hsv,mask=closing),cv2.COLOR_HSV2BGR)

    return closing,res2_vert

def contours(image_couleur,mask,vert,depth_frame):
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
                cv2.circle(vert, (int(x), int(y)), int(rayon), color_info, 2)
                cv2.rectangle(image_couleur, coin, (coin[0]+largeur, coin[1]+hauteur),color_info, 2)
                coord = depth_of_selection(int(x),int(y),depth_frame)
                print(coord)

# def template_matching(template_path, image):
#     img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#     # charger le template de l'objet à rechercher
#     template = cv2.imread(template_path,0)

#     # Récupération des dimensions de l'image
#     w, h = template.shape[::-1]

#     # Application du template atching
#     res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)

#     # Sélection des meilleurs matched objects
#     threshold = 0.8
#     loc = np.where( res >= threshold)

#     # Affichage de la boite englobante de chaque objet détecté
#     for pt in zip(*loc[::-1]):
#         cv2.rectangle(image, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)

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

    # Affichage des composantes HSV sous la souris sur l'image
    pixel_hsv = " ".join(str(values) for values in hsv_px)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(color_image, "px HSV: "+pixel_hsv, (10, 260),
               font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    
    mask,vert=filtre_vert(color_image)

    contours(color_image,mask,vert,depth_frame)

    # template_matching("./tuto_vision/img/template.png",color_image)

    # Display cropped image
    cv2.imshow("Camera", color_image)
    # cv2.imshow('image2', image2)
    cv2.imshow('Mask', mask)
    cv2.imshow("Vert",vert)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break