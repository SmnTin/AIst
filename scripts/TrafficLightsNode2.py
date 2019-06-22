import cv2
import time

import numpy as np
from pathlib import Path

export_dir = 'saved_model'
subdirs = [x for x in Path(export_dir).iterdir()
           if x.is_dir() and 'temp' not in str(x)]
latest = str(sorted(subdirs)[-1])
print(latest)

from tensorflow.contrib import predictor

predict_fn = predictor.from_saved_model(latest)

#pred = predict_fn({'x': [X_test[10]]})['probabilities']
#print(pred)
cap = cv2.VideoCapture(0)

while(True):
  ret, img_bgr = cap.read()
  img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
  draw = img_bgr.copy()

  #plt.imshow(img, cmap='gray', vmin=0, vmax=255)
  blur = cv2.GaussianBlur(img,(5,5),0)
  kernel = np.ones((5,5),np.uint8)

  #print(img.shape)

  for i in range(0, 1):
    
    thval = i*10+80
    ret1,th = cv2.threshold(blur,thval,255,cv2.THRESH_BINARY_INV)
    th = cv2.erode(th,kernel,iterations = 1)
    th = cv2.dilate(th,kernel,iterations = 1)
    th = cv2.dilate(th,kernel,iterations = 1)
    th = cv2.erode(th,kernel,iterations = 1)
    
    contours, hierarchy = cv2.findContours(th,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
      x_,y_,w_,h_ = cv2.boundingRect(cnt)
      offy = int(0.1*y_)
      offx = int(0.1*x_)
      x = max(0, x_-offx)
      y = max(0, y_-offy)
      w = min(th.shape[1], x_ + w_ + offx) - x_
      h = min(th.shape[0], y_ + h_ + offy) - y_
      ratio = h / w 
      area1 = cv2.contourArea(cnt)
      area2 = w*h
      if(area1/area2 > 0.7 and ratio > 1.2 and ratio < 3 and area2 > 500):
        sm = cv2.resize(th[y:y+h, x:x+w],(28,28))
        eq = cv2.equalizeHist(sm)
        rd = eq/256
        pred = predict_fn({'x': [rd]})
        cl = pred['classes'][0]
        print(cl)
        print(pred['probabilities'])
        if(pred['probabilities'][0][cl] > 0.95):
          cv2.rectangle(draw,(x,y),(x+w, y+h),(0,255,0),3)
    
  draw_rgb = cv2.cvtColor(draw, cv2.COLOR_BGR2RGB)
  cv2.imshow("dr", draw)
  cv2.waitKey(27)

