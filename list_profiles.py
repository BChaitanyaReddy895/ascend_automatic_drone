import pyrealsense2 as rs

def list_profiles():
    ctx = rs.context()
    if len(ctx.devices) == 0:
        print("No devices found.")
        return

    device = ctx.devices[0]
    print(f"Device found: {device.get_info(rs.camera_info.name)}")
    print(f"Serial: {device.get_info(rs.camera_info.serial_number)}")
    print("-" * 40)

    for sensor in device.query_sensors():
        print(f"Sensor: {sensor.get_info(rs.camera_info.name)}")
        for profile in sensor.get_stream_profiles():
            if profile.is_video_stream_profile():
                v = profile.as_video_stream_profile()
                print(f"  [VIDEO] {profile.stream_name()} | {v.width()}x{v.height()} @ {profile.fps()}fps | Format: {profile.format()}")
            elif profile.is_motion_stream_profile():
                print(f"  [MOTION] {profile.stream_name()} | @ {profile.fps()}fps | Format: {profile.format()}")

if __name__ == "__main__":
    list_profiles()
