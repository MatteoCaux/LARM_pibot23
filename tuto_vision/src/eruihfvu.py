import cv2
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
# from skimage.measure import label, regionprops
import math
import os 

templates = os.listdir("./tuto_vision/template/")


for t in templates : 
    template = cv2.imread("./tuto_vision/template/"+t)
    # template = cv2.resize(template,(512,384))
    # Récupération des dimensions de l'image
    w, h = template.shape[::-1]

    # Application du template atching
    res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)

    # Sélection des meilleurs matched objects
    threshold = 0.7
    loc = np.where( res >= threshold)

    # Affichage de la boite englobante de chaque objet détecté
    for pt in zip(*loc[::-1]):
        cv2.rectangle(color_image, pt, (pt[0] + w, pt[1] + h), (255,0,0), 2)
