import pyrealsense2 as rs

def print_intrinsics(stream_profile):
    """打印给定流的内参"""
    intrinsics = stream_profile.as_video_stream_profile().get_intrinsics()
    print(f"Intrinsics of {stream_profile.stream_type()} {stream_profile.format()}:")
    print(f" Width: {intrinsics.width}")
    print(f" Height: {intrinsics.height}")
    print(f" cx: {intrinsics.ppx}")
    print(f" cy: {intrinsics.ppy}")
    print(f" Fx: {intrinsics.fx}")
    print(f" Fy: {intrinsics.fy}")
    print(f" Distortion model: {intrinsics.model}")
    print(f" Coefficients: {intrinsics.coeffs}\n")

def main():
    # 创建连接到相机的上下文
    ctx = rs.context()
    if len(ctx.devices) == 0:
        print("No Intel RealSense device is connected.")
        return

    # 使用第一个检测到的设备
    device = ctx.devices[0]
    print("Found device:", device.get_info(rs.camera_info.name))
    print("Serial number:", device.get_info(rs.camera_info.serial_number))
    print("Firmware version:", device.get_info(rs.camera_info.firmware_version))

    # 创建管道并配置流
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(device.get_info(rs.camera_info.serial_number))
    
    # 设置深度流和颜色流的分辨率
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

    # 启动管道
    profile = pipeline.start(config)

    # 获取流配置文件并打印内参
    streams = profile.get_streams()
    for stream in streams:
        if stream.stream_type() in [rs.stream.depth, rs.stream.color]:
            print_intrinsics(stream)

    # 停止管道
    pipeline.stop()

if __name__ == "__main__":
    main()

