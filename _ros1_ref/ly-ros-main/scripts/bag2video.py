#!/usr/bin/env python3
import os
import argparse
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

def main():
    parser = argparse.ArgumentParser(description='Convert ROS bag images to video')
    parser.add_argument('-i', '--input', required=True, help='Input ROS bag file')
    parser.add_argument('-o', '--output', required=True, 
                        help='Output video file (supports .mp4, .avi, .mkv)')
    parser.add_argument('-t', '--topic', default='/ly/compressed/image',
                        help='Image topic name (default: /ly/compressed/image)or /ly/camera/image')
    parser.add_argument('-f', '--fps', type=int, default=30,
                        help='Video FPS (default: 30)')
    args = parser.parse_args()

    # 初始化视频编码器
    fourcc_dict = {
        '.mp4': 'mp4v',
        '.avi': 'XVID',
        '.mkv': 'X264'
    }
    ext = os.path.splitext(args.output)[1].lower()
    fourcc = cv2.VideoWriter_fourcc(*fourcc_dict.get(ext, 'mp4v'))

    bridge = CvBridge()
    video_writer = None
    frame_count = 0

    with rosbag.Bag(args.input, 'r') as bag:
        # 遍历消息并初始化视频参数
        for topic, msg, t in bag.read_messages(topics=[args.topic]):
            try:
                # 自动检测消息类型
                if msg._type == 'sensor_msgs/CompressedImage':
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                elif msg._type == 'sensor_msgs/Image':
                    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
                else:
                    print(f"Unsupported message type: {msg._type}")
                    continue

                # 初始化视频写入器（基于第一帧分辨率）
                if video_writer is None:
                    h, w = img.shape[:2]
                    video_writer = cv2.VideoWriter(
                        args.output, fourcc, args.fps, (w, h)
                    )
                    print(f"Video initialized: {w}x{h} @ {args.fps} FPS")

                # 写入帧
                video_writer.write(img)
                frame_count += 1
                if frame_count % 100 == 0:
                    print(f"Processed {frame_count} frames...")

            except Exception as e:
                print(f"Error processing frame: {str(e)}")
                continue

    if video_writer:
        video_writer.release()
        print(f"Conversion complete. Total frames: {frame_count}")
    else:
        print("No valid image messages found!")

if __name__ == '__main__':
    main()