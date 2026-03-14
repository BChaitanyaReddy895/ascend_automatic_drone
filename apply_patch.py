# -*- coding: utf-8 -*-
import re
import numpy

def update_file(path, replacements):
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    for old, new in replacements:
        content = re.sub(old, new, content, flags=re.MULTILINE)
        
    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)

# 1. Update config.py
config_replacements = [
    (r'FEATURE_MAX_CORNERS = 300', r'FEATURE_MAX_CORNERS = 500'),
    (r'FEATURE_QUALITY_LEVEL = 0.001', r'FEATURE_QUALITY_LEVEL = 0.0003'),
    (r'FEATURE_MIN_DISTANCE = 5', r'FEATURE_MIN_DISTANCE = 4'),
    (r'LK_WIN_SIZE = \(21, 21\)', r'LK_WIN_SIZE = (25, 25)'),
    (r'LK_MAX_LEVEL = 3', r'LK_MAX_LEVEL = 4'),
    (r'VO_CLAHE_CLIP = 4.0', r'VO_CLAHE_CLIP = 3.0'),
    (r'VO_MIN_ALTITUDE = 0.2', r'VO_MIN_ALTITUDE = 0.4'),
    (r'VO_MAX_ALTITUDE = 5.0', r'VO_MAX_ALTITUDE = 6.0'),
    (r'VO_XY_SCALE = 2.0', r'VO_XY_SCALE = 1.0'),
    (r'VO_Z_SCALE = 1.0', r'VO_Z_SCALE = 0.7'),
    (r'MIN_FEATURES_THRESHOLD = 30', r'MIN_FEATURES_THRESHOLD = 25\nMIN_GOOD_FLOW_FRACTION = 0.35\nMAX_FLOW_MAG_PIX = 40.0\nMAX_DZ_M = 0.8'),
    (r'EKF_MEASUREMENT_NOISE_VO = 0.01', r'EKF_MEASUREMENT_NOISE_VO = 0.03'),
    (r'EKF_PROCESS_NOISE_POS = 0.01', r'EKF_PROCESS_NOISE_POS = 0.015'),
    (r'EKF_PROCESS_NOISE_VEL = 2.0', r'EKF_PROCESS_NOISE_VEL = 3.0')
]
update_file('src/config.py', config_replacements)

# 2. Update ekf_fusion.py
ekf_replacements = [
    (r'        if vo_pos is None:\n            return', r'        if vo_pos is None or np.any(np.isnan(vo_pos)):\n            return\n\n        innovation_check = vo_pos - self.state[:3]\n        if np.linalg.norm(innovation_check) > 2.0:\n            self._log(f"Large VO innovation rejected: {np.linalg.norm(innovation_check):.2f} m", level="WARNING")\n            return'),
]
update_file('src/ekf_fusion.py', ekf_replacements)

# 3. Update main.py
main_replacements = [
    (r'dx, dy, dz = vo\.update\(ir_frame, depth_frame\)', r'dx, dy, dz = vo.update(ir_frame, depth_frame)\n\n            if abs(dx) > 1.2 or abs(dy) > 1.2 or abs(dz) > 0.8:\n                logger.log("Large VO step - possible outlier - zeroing", source="Vision", level="WARNING")\n                dx = dy = dz = 0.0'),
]
update_file('src/main.py', main_replacements)

