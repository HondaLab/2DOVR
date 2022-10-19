import numpy as np
import cv2

# parameters to calc dist and angle
A = 644.3
B = -25.19
C = 0.6182
D = -2.104
E = 0.2202

class Picam_frame():
   def __init__(self):
      self.dist=0.0
      self.theta=0.0

   def calc(self,frame,lower,upper):
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, lower, upper)
      contours, hierarchy  = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      rects = []
      for contour in contours:
         approx = cv2.convexHull(contour)
         rect = cv2.boundingRect(approx)
         rects.append(np.array(rect))
    
      if len(rects) > 0:
         rect = max(rects, key=(lambda x: x[2] * x[3]))
         cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
         self.data=list(rect)
         self.data[0]=self.data[0] # レンズのズレ補正
         px=self.data[0]
         py=self.data[1]
         width = self.data[2]
         height = self.data[3]
         cpx = px + (width/2)
         cpy = py + (height/2)
         rad = (cpx-160)*(52/160)*(np.pi/180)
         angle = np.rad2deg(rad)
         dis = (A/(width**C)) + B + (D*(abs(rad)**E))
         dis = float(dis/100)
         #print("\r %6.4f %6.4f" % (angle, dis ), end="" )
      else: #red cup not capture
         dis = None
         rad = None

      self.dist=dis
      self.theta=rad
      return dis, rad

   def hsv(self,frame):
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      bbox = (0,0,10,10)
      bbox = cv2.selectROI(frame, False)
      #ok = tracker.init(frame, bbox)
      #print(bbox)
      x=int(bbox[0]+bbox[2]/2)
      y=int(bbox[1]+bbox[3]/2)
      print("hsv: %4d %4d %4d" % (hsv[y,x,0],hsv[y,x,1],hsv[y,x,2]))

      h_range=10 # 色相の許容範囲
      s_range=50 # 彩度の許容範囲
      v_range=50 # 明度の許容範囲
      #cvtColorでつくったhsv配列はy,xの順序なので注意
      hL=hsv[y,x,0]-h_range
      hU=hsv[y,x,0]+h_range
      sL=hsv[y,x,1]-s_range
      sU=hsv[y,x,1]+s_range
      vL=hsv[y,x,2]-v_range
      vU=hsv[y,x,2]+v_range
      lower_light=np.array([hL,sL,vL])
      upper_light=np.array([hU,sU,vU])
      
      return lower_light,upper_light
