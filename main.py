import numpy as np
import pyrealsense2 as rs

def Start_stream():
    #幅と高さ
    W = 640
    H = 480
    #画像の中心の座標
    CENTER = [int(H / 2), int(W / 2)]

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)

    # Start streaming
    profile = pipeline.start(config)

    #Depthスケール取得
    #距離[m] = depth * depth_scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    return pipeline, depth_scale, CENTER

def get_distancce(pipeline, depth_scale, CENTER):
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if depth_frame:
            break

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())

    #中心座標の距離を取得
    depth = depth_image[CENTER[0]][CENTER[1]]
    cedis = depth * depth_scale

    height = 150
    width = 70
    depth_center_image = depth_image[CENTER[0]-height/2:CENTER[0]+height/2, CENTER[1]-width/2:CENTER[1]+width/2]
    depth_center_image = depth_center_image - depth
    avedis = np.average(depth_center_image) * depth_scale

    return cedis, avedis

pp, ds, cent = Start_stream()
while True:
    cedis, avedis = get_distancce(pp, ds, cent)
    print(str(cedis) + 'm\n' + str(avedis) + 'm')
    #i = input("続行:1　中断:1以外  ")
    #if i != '1':
        #break

pp.stop()
