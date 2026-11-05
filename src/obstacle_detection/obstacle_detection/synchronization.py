"""Bounded timestamp pairing without waiting indefinitely for missing depth."""


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def select_rgb(frames, depths, processed, now, stale_after, tolerance, use_depth):
    """Select the newest unprocessed pair, or RGB alone after a short grace period.

    RGB entries are (message, monotonic receipt, sequence); depth entries are
    (message, monotonic receipt). Timestamp matching uses the camera clock,
    whereas age and the bounded wait use the local monotonic clock.
    """
    pending = [entry for entry in frames
               if entry[2] > processed and now - entry[1] <= stale_after]
    if not pending:
        return None
    if not use_depth:
        return pending[-1]
    for entry in reversed(pending):
        rgb = entry[0]
        if any(now - received <= stale_after
               and (depth.width, depth.height) == (rgb.width, rgb.height)
               and depth.header.frame_id == rgb.header.frame_id
               and abs(stamp_ns(depth) - stamp_ns(rgb)) <= tolerance * 1e9
               for depth, received in depths):
            return entry
    # Give out-of-order depth callbacks time to arrive. Use the oldest pending
    # receipt, since the newest receipt is continually refreshed by live RGB.
    if now - pending[0][1] < min(0.25, stale_after / 2):
        return None
    return pending[-1]
