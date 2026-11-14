"""Select timestamp-matched or latest received frames from bounded buffers."""


def stamp_ns(message):
    """Convert a ROS header timestamp to integer nanoseconds."""
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def select_pair(frames, measurements, processed, now, stale_after, tolerance,
                synchronize=True):
    """Pair RGB with measured range, optionally requiring timestamp agreement."""
    pending = [entry for entry in frames
               if entry[2] > processed and now - entry[1] <= stale_after]
    if not pending:
        return None
    if not synchronize:
        rgb, received, sequence = pending[-1]
        recent = [(sensor, arrival) for sensor, arrival in measurements
                  if now - arrival <= stale_after]
        if not recent:
            return rgb, None, sequence, received
        sensor, arrival = max(recent, key=lambda entry: entry[1])
        return rgb, sensor, sequence, min(received, arrival)
    for rgb, received, sequence in reversed(pending):
        matches = [(sensor, arrival) for sensor, arrival in measurements
                   if now - arrival <= stale_after and stamp_ns(sensor) > 0
                   and stamp_ns(rgb) > 0
                   and abs(stamp_ns(sensor) - stamp_ns(rgb)) <= tolerance * 1e9]
        if matches:
            sensor, arrival = min(matches, key=lambda item: abs(stamp_ns(item[0]) - stamp_ns(rgb)))
            return rgb, sensor, sequence, min(received, arrival)
    if now - pending[0][1] < min(0.25, stale_after / 2):
        return None
    rgb, received, sequence = pending[-1]
    return rgb, None, sequence, received
