"""Per tag: camera distance (AprilTag pose, size 0.14) and robot-frame bearing from a sweep bag."""
import json, math, sys, collections
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from sensor_msgs.msg import JointState, CameraInfo
from l2s1_marker_detection.native_apriltag import estimate_pose

SIZE = 0.14
STEADY_S = 0.8            # only use images taken after the pan stayed the same this long
bag = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
pan, pan_since, info, rows = None, None, None, collections.defaultdict(list)
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == '/ugv/pan_tilt_commanded':
        m = deserialize_message(data, JointState)
        new_pan = -m.position[m.name.index('pt_base_link_to_pt_link1')]   # left positive
        if pan is None or abs(new_pan - pan) > math.radians(0.5):
            pan_since = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9  # camera starts turning
        pan = new_pan
    elif topic == '/camera_info' and info is None:
        info = deserialize_message(data, CameraInfo)
    elif topic == '/marker_detector/detections' and pan is not None and info is not None:
        d = json.loads(deserialize_message(data, String).data)
        if float(d['stamp']) - pan_since < STEADY_S:
            continue                     # camera may still be turning: pan not reliable
        geometry = d.get('image_geometry', 'rectified')
        if geometry == 'rectified':
            K = np.array(info.p).reshape(3, 4)[:, :3]; D = None
        else:
            K = np.array(info.k).reshape(3, 3); D = np.array(info.d)
        sx, sy = d['width'] / info.width, d['height'] / info.height
        K = K * np.array([[sx], [sy], [1]])
        for item in d.get('detections', []):
            c = np.asarray(item['corners'], float).reshape(4, 2)
            try:
                pose = estimate_pose(c, SIZE, K, D)
            except Exception as e:
                continue
            t = np.asarray(pose['translation']).reshape(3)
            cam_dist = float(np.linalg.norm(t))
            cam_bearing = math.atan2(-t[0], t[2])                       # left positive
            offset = abs(c[:, 0].mean() - K[0, 2])
            rows[item['id']].append((offset, math.degrees(pan), cam_dist, math.degrees(pan + cam_bearing),
                                     float(item.get('side_px', 0)), geometry))
print(f'{"ID":>3} {"frames":>6} {"cam dist m":>10} {"spread":>7} {"bearing":>8} {"x m":>6} {"y m":>6} {"side px":>7}  best pan')
for tag in sorted(rows):
    v = sorted(rows[tag])
    near = v[:max(3, len(v) // 3)]                                      # frames nearest the image centre
    dist = np.median([x[2] for x in near]); bearing = np.median([x[3] for x in near])
    spread = np.std([x[2] for x in near])
    b = math.radians(bearing)
    print(f'{tag:>3} {len(v):>6} {dist:>10.2f} {spread:>7.2f} {bearing:>8.1f} {dist*math.cos(b):>6.2f} {dist*math.sin(b):>6.2f} '
          f'{np.median([x[4] for x in near]):>7.1f}  {np.median([x[1] for x in near]):+.0f} ({v[0][5]})')
