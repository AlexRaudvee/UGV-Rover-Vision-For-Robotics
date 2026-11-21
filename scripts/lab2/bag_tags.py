import json, math, sys, collections
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from sensor_msgs.msg import JointState
bag = sys.argv[1]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
pan, seen, sizes = None, collections.defaultdict(collections.Counter), collections.defaultdict(list)
frames = collections.Counter()
while r.has_next():
    topic, data, t = r.read_next()
    if topic == '/ugv/pan_tilt_commanded':
        m = deserialize_message(data, JointState)
        pan = round(-math.degrees(m.position[m.name.index('pt_base_link_to_pt_link1')]))
    elif topic == '/marker_detector/detections' and pan is not None:
        d = json.loads(deserialize_message(data, String).data)
        frames[pan] += 1
        for item in d.get('detections', []):
            seen[pan][item['id']] += 1; sizes[item['id']].append(item.get('side_px', 0))
for p in sorted(frames):
    print(f'pan {p:+4d} deg: {frames[p]:3d} frames  tags ' + ', '.join(f'ID {i} in {n}' for i, n in sorted(seen[p].items())))
print('mean tag size px:', {i: round(sum(v)/len(v), 1) for i, v in sizes.items()})
