// Both views use the same metre-based geometry and alignment from the field server.
import {rosbridge} from './rosbridge.js';

export function transformFieldPoint(transform, point) {
    const [x, y, yaw] = transform, c = Math.cos(yaw), s = Math.sin(yaw);
    return [x + c * point[0] - s * point[1], y + s * point[0] + c * point[1], point[2]];
}

// Horizontal viewing sector on the floor, from commanded head angles.
export function cameraFov(camera, field, ageSeconds) {
    if (!camera?.ready || !field?.aligned || !field.odom_fresh || !Number.isFinite(ageSeconds) || ageSeconds > 1 || ageSeconds < 0 ||
        !Array.isArray(field.robot) || field.robot.length !== 3 || !field.robot.every(Number.isFinite) ||
        !Array.isArray(camera.origin_base) || camera.origin_base.length !== 3 || !camera.origin_base.every(Number.isFinite) ||
        ![camera.pan_deg, camera.left_rad, camera.right_rad, camera.range_m, camera.horizontal_fov_deg].every(Number.isFinite) ||
        camera.range_m <= 0 || camera.left_rad <= 0 || camera.right_rad <= 0 ||
        camera.left_rad + camera.right_rad >= Math.PI) return [];
    const origin = camera.origin_base, pan = camera.pan_deg * Math.PI / 180;
    const points = [origin];
    for (let i = 0; i <= 24; i++) {
        const angle = pan - camera.right_rad + (camera.left_rad + camera.right_rad) * i / 24;
        points.push([origin[0] + camera.range_m * Math.cos(angle),
            origin[1] + camera.range_m * Math.sin(angle), origin[2]]);
    }
    points.push(origin);
    return points.map(point => transformFieldPoint(field.robot, point));
}

export function markerVisuals(result, ageSeconds, revision, frame) {
    if (!result || (frame && result.frame_id !== frame)) return [];
    return (result.markers || []).filter(marker => Array.isArray(marker.map_position_m) &&
        marker.map_position_m.length === 3 && marker.map_position_m.every(Number.isFinite)).map(marker => {
        const sameAlignment = revision === undefined || revision === null || result.field_revision === revision;
        const ttl = Number.isFinite(result.stale_after_s) ? result.stale_after_s : 1;
        const measurementAge = (Number.isFinite(marker.age_s) ? marker.age_s : 0) + ageSeconds;
        const fresh = Boolean(marker.fresh && result.stream_fresh && ageSeconds < 1.5 &&
            measurementAge <= ttl && sameAlignment && Number.isFinite(marker.range_m));
        const detected = marker.detected && (marker.detection_age_s || 0) + ageSeconds <= ttl;
        const approximate = marker.low_resolution || marker.samples < 3;
        const unavailable = detected ? (marker.rejection_reason ? 'range rejected' : 'range pending') : 'not visible';
        const distance = fresh ? `${approximate?'~':''}${marker.range_m.toFixed(approximate?1:2)} m` : unavailable;
        return {position: marker.map_position_m, robot: fresh ? marker.robot_position_m : null,
            fresh, color: fresh ? '#6df5ec' : '#9baaaa',
            text: `ID ${marker.id} · ${distance}`, approximate: Boolean(fresh && approximate)};
    });
}

export function triangulationVisual(result, ageSeconds, revision, frame) {
    if (!result?.fresh || result.frame_id !== frame || !Number.isFinite(ageSeconds) || ageSeconds < 0 ||
        !Number.isFinite(result.age_s) || !Number.isFinite(result.stale_after_s) ||
        result.age_s + ageSeconds > result.stale_after_s ||
        (revision != null && result.field_revision !== revision) ||
        !Array.isArray(result.position_m) || result.position_m.length !== 3 ||
        !result.position_m.every(Number.isFinite)) return null;
    return {position: result.position_m, heading: Number.isFinite(result.heading_deg) ? result.heading_deg*Math.PI/180 : null,
        odom: Array.isArray(result.odom_position_m) && result.odom_position_m.length === 3 &&
            result.odom_position_m.every(Number.isFinite) ? result.odom_position_m : null,
        color: '#ff79df', text: `TRIANGULATION (${result.position_m[0].toFixed(2)}, ${result.position_m[1].toFixed(2)})`};
}

// The controller is the only authority for target selection; never duplicate its table here.
export function avoidanceGoalVisual(status, ageSeconds, frame) {
    const goal = status?.goal;
    if (!goal || typeof frame !== 'string' || !frame || goal.frame_id !== frame ||
        !Number.isFinite(ageSeconds) || ageSeconds < 0 || ageSeconds > 2.5 ||
        !Array.isArray(goal.position_m) || goal.position_m.length !== 2 || !goal.position_m.every(Number.isFinite) ||
        !Number.isInteger(goal.target_run) || goal.target_run < 0 || goal.target_run > 5 ||
        typeof goal.target_name !== 'string') return null;
    const [x, y] = goal.position_m;
    return {position: [x, y, .025],
        text: `${goal.target_run ? `RUN ${goal.target_run}` : 'GOAL'} · ${goal.target_name} (${x.toFixed(2)}, ${y.toFixed(2)})`};
}

// A small vector duck stays readable at every zoom level and needs no image/font download.
// The ring marks the exact target coordinate; the duck sits immediately above it.
export function drawGoalDuck(ctx, x, y, size = 36) {
    ctx.save(); ctx.translate(x, y); ctx.scale(size / 40, size / 40);
    ctx.strokeStyle = '#ffe16a'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.ellipse(0, 0, 12, 4, 0, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = 'rgba(0,0,0,.25)';
    ctx.beginPath(); ctx.ellipse(0, -2, 14, 4, 0, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#ffd431'; ctx.strokeStyle = '#956400'; ctx.lineWidth = 1.3;
    ctx.beginPath(); ctx.moveTo(-10, -11); ctx.lineTo(-19, -20); ctx.lineTo(-17, -7);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.ellipse(-1, -12, 14, 10, 0, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.arc(8, -25, 8, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#ff8d24';
    ctx.beginPath(); ctx.moveTo(14, -27); ctx.lineTo(24, -23); ctx.lineTo(14, -20);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#f1b91b';
    ctx.beginPath(); ctx.ellipse(-3, -11, 7, 4, -.2, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#222b2b';
    ctx.beginPath(); ctx.arc(10, -27, 1.5, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
}

export class FieldClient {
    constructor(onUpdate = () => {}) {
        this.layout = this.state = this.ranges = this.camera = this.triangulation = this.avoidance = null;
        this.lastState = this.lastRanges = this.lastCamera = this.lastTriangulation = 0;
        this.lastAvoidance = 0;
        this.listeners = new Set([onUpdate]);
        this.topics = ['layout', 'state', 'ranges', 'camera', 'triangulation', 'avoidance'].map(kind => {
            const name = kind === 'ranges' ? '/marker_ranges/result' :
                kind === 'camera' ? '/marker_pan_tracker/state' :
                kind === 'triangulation' ? '/marker_triangulation/result' :
                kind === 'avoidance' ? '/obstacle_avoidance_adv/status' : `/football_field/${kind}`;
            const topic = new ROSLIB.Topic({ros: rosbridge.ros, name,
                messageType: 'std_msgs/msg/String', compression: rosbridge.compression,
                queue_length: 1});
            topic.subscribe(message => {
                try {
                    this[kind] = JSON.parse(message.data);
                    if (kind === 'state') this.lastState = performance.now();
                    if (kind === 'ranges') this.lastRanges = performance.now();
                    if (kind === 'camera') this.lastCamera = performance.now();
                    if (kind === 'triangulation') this.lastTriangulation = performance.now();
                    if (kind === 'avoidance') this.lastAvoidance = performance.now();
                    for (const listener of this.listeners) listener();
                } catch (error) { console.warn('Football field message:', error); }
            });
            return topic;
        });
    }
    get connected() { return this.lastState > 0 && performance.now() - this.lastState < 2500; }
    get goalVisual() {
        return avoidanceGoalVisual(this.avoidance, (performance.now()-this.lastAvoidance)/1000,
            this.layout?.frame_id);
    }
    get triangulationVisual() {
        return triangulationVisual(this.triangulation, (performance.now()-this.lastTriangulation)/1000,
            this.state?.revision, this.layout?.frame_id);
    }
    get odomVisual() {
        const robot = this.state?.robot;
        if (!this.connected || !this.state?.odom_fresh || !this.state.aligned ||
            !Array.isArray(robot) || robot.length !== 3 || !robot.every(Number.isFinite)) return null;
        return {position: [robot[0], robot[1], .02], heading: robot[2], color: '#f7c76b',
            text: `${this.state.pose_source === 'marker_ekf' ? 'FUSED' : this.state.pose_source === 'external' ? 'LOCALIZED' : 'ODOM'} (${robot[0].toFixed(2)}, ${robot[1].toFixed(2)})`};
    }
    get localizationStatus() {
        if (!this.triangulation) return 'Waiting for triangulation';
        if (!this.triangulationVisual) return `Triangulation: ${this.triangulation.error || 'measurement stale'}`;
        const delta = this.triangulation.position_delta_m;
        return `Triangulation: ${(this.triangulation.used_ids || []).length} markers` +
            (Number.isFinite(delta) ? ` · Δ ${this.state?.pose_source === 'marker_ekf' ? 'fused pose' : 'odometry'} ${delta.toFixed(2)} m (image time)` : ' · pose comparison unavailable');
    }
    get cameraFov() {
        if (!this.connected || this.state?.frame_id !== this.layout?.frame_id) return [];
        return cameraFov(this.camera, this.state, (performance.now()-this.lastCamera)/1000);
    }
    get cameraStatus() {
        if (!this.cameraFov.length) return '';
        return `Camera view (estimated) ${this.camera.horizontal_fov_deg.toFixed(0)}° · ` +
            (['tracking_markers', 'tracking_pair'].includes(this.camera.mode) ?
                (Array.isArray(this.camera.tracked_ids) ? `tracking ${this.camera.tracked_ids.length} tags` : 'tracking tags') :
                'centering forward');
    }
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
            const reasons = (this.ranges.markers || []).map(marker => marker.rejection_reason).filter(Boolean);
            return reasons.length ? `AprilTag: ${reasons[0]}` : 'No recent AprilTag distance measurements';
        }
        const ages = (this.ranges.markers || []).filter(marker => marker.fresh).map(marker => marker.age_s || 0);
        const delay = Math.max(0, ...ages) + (performance.now()-this.lastRanges)/1000;
        return 'AprilTag ground ranges · ' + visible.map(marker => marker.text).join(' · ') +
            (visible.some(marker => marker.approximate) ? ' · small tags: approximate ranges' : '') +
            (delay > .5 ? ` · image delay ${delay.toFixed(1)} s` : '');
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
            const editable = typeof this.state?.start_control_enabled === 'boolean' ?
                this.state.start_control_enabled : this.state?.publish_tf !== false;
            element('save_start').disabled = pending || !this.connected || !editable;
            element('apply_start').disabled = pending || !this.connected || !this.state?.odom_fresh || !editable;
            element('config_file').textContent = this.state?.config_file || 'Waiting for field server…';
            element('start_status').textContent = this.state?.error || message ||
                (this.state?.pose_source === 'marker_ekf' ? 'EKF owns field position; applying start resets the filter' : '') ||
                (!editable ? 'Position supplied by external localization' : '') ||
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
