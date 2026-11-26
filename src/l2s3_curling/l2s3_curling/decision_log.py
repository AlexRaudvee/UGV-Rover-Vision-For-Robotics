"""Write avoidance evidence and compressed camera frames outside the control loop."""

from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Thread
from uuid import uuid4


CAMERA_TOPICS = {
    'oak_color': '/oak/rgb/image_rect/compressed',
}


class DecisionLog:
    def __init__(self, directory, warn):
        self.root = Path(directory)
        self.warn = warn
        self.frames = {name: deque(maxlen=32) for name in CAMERA_TOPICS}
        self.pending = Queue(maxsize=16)
        self.finished = Event()
        self.worker = Thread(target=self.write_pending, daemon=True)
        self.worker.start()
        self.new_run()

    def new_run(self):
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.directory = self.root / f'{timestamp}-{uuid4().hex[:8]}'
        self.sequence = 0

    def on_image(self, name, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if stamp > 0 and 0 < len(message.data) <= 1024 * 1024:
            self.frames[name].append((stamp, message))

    def enqueue(self, record):
        now = record['stamp_s']
        self.sequence += 1
        event_id = f'{self.sequence:04d}-{record["kind"]}'
        images = {}
        record.update(event_id=event_id, images={})
        for name, frames in self.frames.items():
            received = [(stamp, message) for stamp, message in frames if stamp <= now+.1]
            if received:
                stamp, message = min(received, key=lambda frame: abs(frame[0]-now))
                if abs(stamp-now) <= .45:
                    suffix = '.png' if 'png' in message.format.lower() else '.jpg'
                    filename = name + suffix
                    images[filename] = message
                    record['images'][name] = dict(path=f'{event_id}/{filename}', stamp_s=stamp,
                                                  age_s=now-stamp, matched_to='decision',
                                                  reference_stamp_s=now)
            if name not in record['images']:
                record['images'][name] = dict(path=None, reason='No recent frame near reference timestamp')
        try:
            self.pending.put_nowait((self.directory, record, images))
        except Full:
            self.warn('Avoidance decision log queue is full; snapshot dropped')
            return None
        return self.directory / event_id / 'decision.json'

    def write_pending(self):
        while not self.finished.is_set() or not self.pending.empty():
            try:
                directory, record, images = self.pending.get(timeout=.05)
            except Empty:
                continue
            try:
                event = directory / record['event_id']
                event.mkdir(parents=True)
                for filename, message in images.items():
                    (event / filename).write_bytes(bytes(message.data))
                encoded = json.dumps(record, allow_nan=False, indent=2)
                (event / 'decision.json').write_text(encoded+'\n', encoding='utf-8')
                with (directory / 'decisions.jsonl').open('a', encoding='utf-8') as log:
                    log.write(json.dumps(record, allow_nan=False)+'\n')
            except (OSError, TypeError, ValueError) as error:
                self.warn(f'Could not save avoidance snapshot: {error}')

    def close(self):
        self.finished.set()
        self.worker.join(timeout=2.)
