import numpy as np
import pyrealsense2 as rs
import cv2
import math
import csv

def Start_stream():
   #幅と高さ
   W = 640
   H = 480
   #画像の中心の座標
   CENTER = [int(H / 2), int(W / 2)]
   #カメラ上の大きさ算出 S = k(M/r)の比例定数kは画角と画像サイズで決まる
   c = 8.0
   k = W / (2 * math.tan(math.radians(34.7-c)))

   # Configure depth and color streams
   pipeline = rs.pipeline()
   config = rs.config()
   config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
   config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

   # Start streaming
   profile = pipeline.start(config)

   #Depthスケール取得
   #距離[m] = depth * depth_scale
   depth_sensor = profile.get_device().first_depth_sensor()
   depth_scale = depth_sensor.get_depth_scale()

   return pipeline, depth_scale, CENTER, k

def get_distancce(depth_image, depth_scale, CENTER, k, Vw):
   #中心座標の距離を取得
   depth = np.array(depth_image[CENTER[0]-10:CENTER[0]+10, CENTER[1]:CENTER[1]+1], dtype=np.float32)
   mean_dis = np.mean(depth * depth_scale)
   if mean_dis == 0:
       return mean_dis, None, [0, 0]

   #画像中央の深度を横方向で１階微分する。
   cediff = depth - np.array(depth_image[CENTER[0]-10:CENTER[0]+10, CENTER[1]+1:CENTER[1]+2], dtype=np.float32)

   #深度の横方向に対する傾きによって、V[0]の値を少し変更する
   Vw = Vw * math.cos(math.atan(abs(cediff.mean())))
   weight = k * Vw / mean_dis
   weight = weight * 0.7
   ran = [int(weight), 20]
   # if cediff != 0:
   #     #高さの最小値に範囲の高さを合わせる→一番遠い場所で高さの計算をする
   #     ran = np.array([0, 0], dtype = np.float32)
   #     ran[0] = k[0] * V[0] / cedis
   #     i = 0
   #     maxdis = depth_image[CENTER[0]][int(CENTER[1] + (cediff / abs(cediff)) * (ran[0] / 2 - 5))]
   #     while maxdis==0:
   #         maxdis = depth_image[CENTER[0]][int(CENTER[1] + (cediff / abs(cediff)) * (ran[0] / 2 - (5 + i)))]
   #     ran[1] = k[1] * V[1] / maxdis
   # else:
   #     ran = k * V / cedis
   #範囲内の深度を横方向で２階微分する。平面ならば０になるはずである。
   depth_center_image = np.array(depth_image[int(CENTER[0]-ran[1]/2):int(CENTER[0]+ran[1]/2), int(CENTER[1]-ran[0]/2):int(CENTER[1]+ran[0]/2)], dtype=np.int32)
   depth_center_image_diff = np.diff(depth_center_image, n=1)
   #print(np.mean(depth_center_image_diff), end=" ")
   #depth_cent_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_center_image, alpha=0.03), cv2.COLORMAP_JET)
   #depth_cent_colormap = cv2.resize(depth_cent_colormap, (140, 300))
   #cv2.imshow('cent', depth_cent_colormap)
   #print(depth_center_image, depth, type(depth))
   if depth_center_image_diff.shape[1] == 0:
       variance = None
   else:
       variance = np.var(depth_center_image_diff)

   return mean_dis, variance, ran

with open('plot1.csv', mode='a', newline='') as fout:
   pp, ds, cent, k = Start_stream()
   csvout = csv.writer(fout)
   rec = False
   while True:
       frames = pp.wait_for_frames()
       depth_frame = frames.get_depth_frame()
       color_frame = frames.get_color_frame()
       if not depth_frame or not color_frame:
           continue
       depth_image = np.asanyarray(depth_frame.get_data())
       color_image = np.asanyarray(color_frame.get_data())

       mean_dis, variance, rang = get_distancce(depth_image, ds, cent, k, 450e-3)

       # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
       depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
       depth_colormap = cv2.rectangle(depth_colormap, (int(cent[1]-rang[0]/2), int(cent[0]-rang[1]/2)), (int(cent[1]+rang[0]/2), int(cent[0]+rang[1]/2)), (0, 0, 0))
       # Stack both images horizontally
       images = np.hstack((color_image, depth_colormap))
       #images = cv2.flip(images, 1)
       # Show images
       cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
       cv2.imshow('RealSense', images)

       # if variance:
       #     print(variance)
       # else:
       #     print('None')
       #     continue
       #
       # if rec == True and variance < 1000:
       #     print('データを保存しています')
       #     csvout.writerow([mean_dis, variance])
       print(mean_dis, end=" ")

       print(variance, end=" ")
       if not variance:
           print('平面ではない')
       elif variance < 0.0504*math.e**(2.0858*mean_dis)+10:
           print('平面です')
       else:
           print('平面ではない')

       if cv2.waitKey(1) & 0xFF == ord('q'):
           break
       elif cv2.waitKey(1) & 0xFF == ord('s'):
           rec = True

   pp.stop()
