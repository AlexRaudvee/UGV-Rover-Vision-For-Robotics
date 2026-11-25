"""Pure path-following calculation for offline and preview use."""
import math


def command(pose, path, lookahead=.45, max_speed=.15, max_turn=.7, goal_tolerance=.15):
    if not path or not all(math.isfinite(v) for v in pose):
        return 0., 0., 'no valid path or pose'
    x, y, yaw = pose
    if not all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in path):
        return 0., 0., 'invalid path'
    if math.dist((x,y), path[-1]) <= goal_tolerance:
        return 0., 0., 'arrived'
    closest = min(range(len(path)), key=lambda i: math.dist((x,y), path[i]))
    target = path[-1]
    for point in path[closest:]:
        if math.dist((x,y), point) >= lookahead:
            target = point
            break
    dx, dy = target[0]-x, target[1]-y
    local_x = math.cos(yaw)*dx + math.sin(yaw)*dy
    local_y = -math.sin(yaw)*dx + math.cos(yaw)*dy
    distance = math.hypot(local_x, local_y)
    if distance < 1e-6:
        return 0., 0., 'no usable lookahead'
    heading = math.atan2(local_y, local_x)
    # For a sharp turn use a very slow forward arc; pure pursuit has nearly
    # zero curvature for a target directly behind and would drive away from it.
    if abs(heading) > 1.0:
        return min(max_speed, .01), math.copysign(max_turn, heading), 'following'
    curvature = 2*local_y/(distance*distance)
    speed = min(max_speed, max_turn/max(abs(curvature), 1e-9))
    speed *= max(.25, 1.-abs(heading)/1.4)
    return speed, max(-max_turn, min(max_turn, speed*curvature)), 'following'
