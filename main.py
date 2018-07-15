import cv2
import numpy as np
import pyrealsense2 as rs

#幅と高さ
W = 640
H = 480
#画像の中心の座標
CENTER = [int(H / 2), int(W / 2)]

M = [0.15, 0.30]
k = 1.0e5

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

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        #中心座標の距離を取得
        distance = depth * depth_scale
        print(str(distance) + 'm')

        #画像中心にペットボトルがあった場合のペットボトルの範囲を予測
        depth = depth_image[CENTER[0]][CENTER[1]]
        if distance != 0.0 && distance <= 2:
            width = int(k * M[0] / depth) + 20
            height = int(k * M[1] / depth) + 10
            depth_center = depth_image[CENTER[0]-(height/2):CENTER[0]+(height/2), CENTER[1]-(width/2):CENTER[1]+(width/2)]
            
        else:
            continue

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        #images = np.hstack((color_image, depth_colormap))

        # Show images
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', images)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
