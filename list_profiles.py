import pyrealsense2 as rs

def list_profiles():
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Try to get a handle to the device
    try:
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        print(f"Device found: {device.get_info(rs.camera_info.name)}")
        print(f"Serial: {device.get_info(rs.camera_info.serial_number)}")
        print("-" * 40)
        
        for sensor in device.query_sensors():
            print(f"Sensor: {sensor.get_info(rs.camera_info.name)}")
            profiles = sensor.get_stream_profiles()
            # Sort by stream type and resolution
            for p in profiles:
                # Filter for depth, ir, accel, gyro
                s = p.stream_type()
                if s in [rs.stream.depth, rs.stream.infrared, rs.stream.accel, rs.stream.gyro]:
                    if p.is_video_stream_profile():
                        v = p.as_video_stream_profile()
                        print(f"  [VIDEO] {v.stream_name()} | {v.width()}x{v.height()} @ {v.fps()}fps | Format: {v.format()}")
                    elif p.is_motion_stream_profile():
                        m = p.as_motion_stream_profile()
                        print(f"  [MOTION] {m.stream_name()} | @ {m.fps()}fps | Format: {m.format()}")
            print("-" * 40)
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    list_profiles()
