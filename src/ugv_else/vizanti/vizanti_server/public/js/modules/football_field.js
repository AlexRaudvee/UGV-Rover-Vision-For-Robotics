// Both views use the same metre-based geometry and alignment from the field server.
import {rosbridge} from './rosbridge.js';

export function transformFieldPoint(transform, point) {
    const [x, y, yaw] = transform, c = Math.cos(yaw), s = Math.sin(yaw);
    return [x + c * point[0] - s * point[1], y + s * point[0] + c * point[1], point[2]];
}

export function markerVisuals(result, ageSeconds, revision, frame) {
    if (!result || (frame && result.frame_id !== frame)) return [];
    return (result.markers || []).filter(marker => Array.isArray(marker.map_position_m) &&
        marker.map_position_m.length === 3 && marker.map_position_m.every(Number.isFinite)).map(marker => {
        const sameAlignment = revision === undefined || revision === null || result.field_revision === revision;
        const fresh = Boolean(marker.fresh && result.stream_fresh && ageSeconds < 1.5 &&
            sameAlignment && Number.isFinite(marker.range_m));
        const approximate = marker.low_resolution || marker.samples < 3;
        const distance = fresh ? `${approximate?'~':''}${marker.range_m.toFixed(approximate?1:2)} m` : 'not visible';
        return {position: marker.map_position_m, robot: fresh ? marker.robot_position_m : null,
            fresh, color: fresh ? '#6df5ec' : '#9baaaa',
            text: `ID ${marker.id} · ${distance}`, approximate: Boolean(fresh && approximate)};
    });
}

export class FieldClient {
    constructor(onUpdate = () => {}) {
        this.layout = this.state = this.ranges = null;
        this.lastState = this.lastRanges = 0;
        this.listeners = new Set([onUpdate]);
        this.topics = ['layout', 'state', 'ranges'].map(kind => {
            const name = kind === 'ranges' ? '/marker_ranges/result' : `/football_field/${kind}`;
            const topic = new ROSLIB.Topic({ros: rosbridge.ros, name,
                messageType: 'std_msgs/msg/String', compression: rosbridge.compression,
                queue_length: 1});
            topic.subscribe(message => {
                try {
                    this[kind] = JSON.parse(message.data);
                    if (kind === 'state') this.lastState = performance.now();
                    if (kind === 'ranges') this.lastRanges = performance.now();
                    for (const listener of this.listeners) listener();
                } catch (error) { console.warn('Football field message:', error); }
            });
            return topic;
        });
    }
    get connected() { return this.lastState > 0 && performance.now() - this.lastState < 2500; }
    get markerVisuals() {
        return markerVisuals(this.ranges, (performance.now()-this.lastRanges)/1000,
            this.state?.revision, this.layout?.frame_id);
    }
    get rangeStatus() {
        if (!this.ranges) return 'Waiting for AprilTag distance measurements…';
        if (this.ranges.error) return `AprilTag: ${this.ranges.error}`;
        if (performance.now()-this.lastRanges > 2000 || !this.ranges.stream_fresh) return 'AprilTag detection stream stale';
        const visible = this.markerVisuals.filter(marker => marker.fresh);
        if (!visible.length) {
            const reasons = Object.values(this.ranges.rejected || {});
            return reasons.length ? `AprilTag: ${reasons[0]}` : 'No recent AprilTag distance measurements';
        }
        return 'AprilTag ground ranges · ' + visible.map(marker => marker.text).join(' · ') +
            (visible.some(marker => marker.approximate) ? ' · small tags: approximate ranges' : '');
    }
    call(name, type, request) {
        return new Promise((resolve, reject) => {
            const service = new ROSLIB.Service({ros: rosbridge.ros, name, serviceType: type});
            const timeout = setTimeout(() => reject(Error('Field server did not respond')), 6000);
            service.callService(new ROSLIB.ServiceRequest(request), response => {
                clearTimeout(timeout); resolve(response);
            }, error => { clearTimeout(timeout); reject(Error(String(error))); });
        });
    }
    async saveStart(values) {
        if (!this.connected) throw Error('Start the football field server first');
        const parameters = Object.entries(values).map(([name, value]) => {
            if (!Number.isFinite(value)) throw Error('Enter a number for x, y and heading');
            return {name, value: {type: 3, double_value: value}}; // PARAMETER_DOUBLE
        });
        const response = await this.call('/football_field/set_parameters_atomically',
            'rcl_interfaces/srv/SetParametersAtomically', {parameters});
        if (!response.result.successful) throw Error(response.result.reason);
        if (this.state) this.state.start = {...this.state.start, ...values};
    }
    async applyStart() {
        if (!this.connected) throw Error('Start the football field server first');
        const response = await this.call('/football_field/apply_start', 'std_srvs/srv/Trigger', {});
        if (!response.success) throw Error(response.message);
    }
    bindControls(element) {
        const inputs = ['start_x_m', 'start_y_m', 'start_heading_deg'].map(element);
        let dirty = false, pending = false, message = '';
        for (const input of inputs) input.addEventListener('input', () => { dirty = true; });
        const update = () => {
            if (this.state && !dirty) inputs.forEach(input => {
                const key = input.dataset.fieldKey;
                input.value = this.state.start[key];
            });
            element('save_start').disabled = pending || !this.connected || this.state?.publish_tf === false;
            element('apply_start').disabled = pending || !this.connected || !this.state?.odom_fresh || this.state?.publish_tf === false;
            element('config_file').textContent = this.state?.config_file || 'Waiting for field server…';
            element('start_status').textContent = this.state?.error || message || (this.state?.publish_tf === false ? 'Position supplied by external localization' : '') ||
                (this.state?.aligned ? 'Start alignment active' : 'Waiting for odometry to align the start');
        };
        const perform = async action => {
            pending = true; message = ''; update();
            try {
                await action();
                dirty = false;
                message = this.state?.odom_fresh ? 'Saved start applied. Robot movement was not commanded.' :
                    'Start saved. Waiting for fresh odometry; use Apply saved start here when ready.';
            } catch (error) { message = error.message; }
            finally { pending = false; update(); }
        };
        element('save_start').onclick = () => perform(() => this.saveStart(Object.fromEntries(
            inputs.map(input => [input.dataset.fieldKey, input.value.trim() === '' ? NaN : Number(input.value)]))));
        element('apply_start').onclick = () => perform(() => this.applyStart());
        this.listeners.add(update);
        update();
    }
    close() {
        for (const topic of this.topics) topic.unsubscribe();
        this.listeners.clear();
    }
}
