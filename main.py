import numpy as np
import pyrealsense2 as rs
import cv2
import math

def Start_stream():
    #幅と高さ
    W = 640
    H = 480
    #画像の中心の座標
    CENTER = [int(H / 2), int(W / 2)]
    #カメラ上の大きさ算出 S = k(M/r)の比例定数kは画角と画像サイズで決まる
    c = [8.0, 0.5]
    k = np.array([W / (2 * math.tan(math.radians(34.7-c[0]))), H / (2 * math.tan(math.radians(21.25-c[1])))], dtype=np.float32)

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

def get_distancce(depth_image, depth_scale, CENTER, k, V):
    #中心座標の距離を取得
    depth = depth_image[CENTER[0]][CENTER[1]]
    cedis = depth * depth_scale
    if cedis == 0:
        return cedis, 'none', [0, 0]

    ran = k * V / cedis
    depth_center_image = np.array(depth_image[int(CENTER[0]-ran[1]/2):int(CENTER[0]+ran[1]/2), int(CENTER[1]-ran[0]/2):int(CENTER[1]+ran[0]/2)], dtype=np.int32)
    #depth_cent_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_center_image, alpha=0.03), cv2.COLORMAP_JET)
    #depth_cent_colormap = cv2.resize(depth_cent_colormap, (140, 300))
    #cv2.imshow('cent', depth_cent_colormap)
    #print(depth_center_image, depth, type(depth))
    depth_center_image = depth_center_image - depth
    avedis = depth_center_image.mean() * depth_scale

    return cedis, avedis, ran

pp, ds, cent, k = Start_stream()
while True:
    frames = pp.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    cedis, avedis, rang = get_distancce(depth_image, ds, cent, k, np.array([500e-3, 300e-3], dtype=np.float32))
    print(str(cedis) + 'm\n' + str(avedis) + 'm')

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    depth_colormap = cv2.rectangle(depth_colormap, (int(cent[1]-rang[0]/2), int(cent[0]-rang[1]/2)), (int(cent[1]+rang[0]/2), int(cent[0]+rang[1]/2)), (0, 0, 0))
    # Stack both images horizontally
    images = np.hstack((color_image, depth_colormap))
    images = cv2.flip(images, 1)
    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pp.stop()
