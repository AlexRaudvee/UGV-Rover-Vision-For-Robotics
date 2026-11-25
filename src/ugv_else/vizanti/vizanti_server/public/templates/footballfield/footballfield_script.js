const {FieldClient, drawGoalDuck} = await import(`${base_url}/js/modules/football_field.js`);
const {view} = await import(`${base_url}/js/modules/view.js`);
const {tf} = await import(`${base_url}/js/modules/tf.js`);
const {settings} = await import(`${base_url}/js/modules/persistent.js`);
const el = name => document.getElementById(`{uniqueID}_${name}`);
const canvas = el('canvas'), ctx = canvas.getContext('2d');
const field = new FieldClient();
field.bindControls(el);
function screen(point) {
    let p = {x: point[0], y: point[1], z: point[2]};
    if (tf.fixed_frame !== field.layout.frame_id) {
        p = tf.transformPose(field.layout.frame_id, tf.fixed_frame, p, new Quaternion()).translation;
    }
    return view.fixedToScreen(p);
}
el('fit').onclick = () => {
    if (!field.layout) return;
    tf.setFixedFrame(field.layout.frame_id);
    settings.settings_default = {...settings.settings_default, fixed_frame: tf.fixed_frame};
    view.center = {x: 0, y: 0};
    view.scale = Math.min(window.innerWidth / (field.layout.length_m + 2*field.layout.border_m + 1),
        (window.innerHeight - 140) / (field.layout.width_m + 2*field.layout.border_m + 1));
    settings.view = {center: view.center, scale: view.scale}; settings.save(); view.sendUpdateEvent();
};
let lastDraw = 0;
function render(now) {
    if (!canvas.isConnected) { field.close(); return; }
    requestAnimationFrame(render);
    if (now - lastDraw < 80) return;
    lastDraw = now;
    canvas.width = window.innerWidth; canvas.height = window.innerHeight;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const layout = field.layout;
    if (!layout) { el('status').textContent = 'Waiting for /football_field/layout'; return; }
    if (tf.fixed_frame !== layout.frame_id && !tf.absoluteTransforms[layout.frame_id]) {
        el('status').textContent = `Select fixed frame ${layout.frame_id}, or wait for field alignment`; return;
    }
    ctx.beginPath();
    layout.ground.map(screen).forEach((p, i) => i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y));
    ctx.closePath(); ctx.fillStyle = layout.ground_color; ctx.fill();
    for (const line of layout.lines) {
        ctx.strokeStyle = line.color;
        ctx.lineWidth = Math.max(1, view.getMapUnitsInPixels(line.width_m));
        ctx.beginPath();
        line.points.map(screen).forEach((p, i) => i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y));
        ctx.stroke();
    }
    const fov = field.cameraFov.map(screen);
    if (fov.length) {
        ctx.beginPath();
        fov.forEach((p, i) => i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y));
        ctx.closePath(); ctx.fillStyle = 'rgba(80,180,255,0.16)'; ctx.fill();
        ctx.strokeStyle = '#50b4ff'; ctx.lineWidth = 1.5; ctx.stroke();
    }
    const start = field.state?.start;
    if (start) {
        const x = start.start_x_m, y = start.start_y_m, yaw = start.start_heading_deg * Math.PI/180;
        const a = screen([x, y, 0]), b = screen([x+.4*Math.cos(yaw), y+.4*Math.sin(yaw), 0]);
        ctx.strokeStyle = '#ffc76b'; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.arc(a.x, a.y, 8, 0, Math.PI*2); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        ctx.fillStyle = '#ffe2ac'; ctx.font = '13px sans-serif';
        ctx.fillText(`START (${x.toFixed(2)}, ${y.toFixed(2)})`, a.x+12, a.y-12);
    }
    for (const marker of field.markerVisuals) {
        const p = screen(marker.position);
        ctx.strokeStyle = marker.color; ctx.lineWidth = 2;
        if (marker.robot && marker.robot.every(Number.isFinite)) {
            const robot = screen(marker.robot);
            ctx.setLineDash([5, 5]); ctx.beginPath();
            ctx.moveTo(robot.x, robot.y); ctx.lineTo(p.x, p.y); ctx.stroke(); ctx.setLineDash([]);
        }
        ctx.fillStyle = '#153b35'; ctx.fillRect(p.x-7, p.y-7, 14, 14);
        ctx.strokeRect(p.x-7, p.y-7, 14, 14);
        ctx.fillStyle = marker.color; ctx.font = 'bold 14px sans-serif';
        ctx.fillText(marker.text, p.x+12, p.y-10);
    }
    for (const pose of [field.odomVisual, field.triangulationVisual].filter(Boolean)) {
        const p = screen(pose.position);
        ctx.strokeStyle = pose.color; ctx.lineWidth = 2.5;
        if (pose.odom) {
            const odom = screen(pose.odom);
            ctx.setLineDash([4, 4]); ctx.beginPath();
            ctx.moveTo(odom.x, odom.y); ctx.lineTo(p.x, p.y); ctx.stroke(); ctx.setLineDash([]);
        }
        ctx.beginPath(); ctx.arc(p.x, p.y, 9, 0, Math.PI*2);
        ctx.moveTo(p.x-5, p.y); ctx.lineTo(p.x+5, p.y);
        ctx.moveTo(p.x, p.y-5); ctx.lineTo(p.x, p.y+5); ctx.stroke();
        if (pose.heading !== null) {
            const end = screen([pose.position[0]+.4*Math.cos(pose.heading),
                pose.position[1]+.4*Math.sin(pose.heading), pose.position[2]]);
            ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(end.x, end.y); ctx.stroke();
        }
        ctx.fillStyle = pose.color; ctx.font = 'bold 13px sans-serif';
        ctx.fillText(pose.text, p.x+12, p.y+(pose.odom !== undefined ? 22 : -24));
    }
    const goal = field.goalVisual;
    if (goal) {
        const p = screen(goal.position);
        drawGoalDuck(ctx, p.x, p.y);
        ctx.font = 'bold 13px sans-serif'; ctx.lineWidth = 4; ctx.strokeStyle = '#19382e';
        ctx.strokeText(goal.text, p.x+25, p.y-25);
        ctx.fillStyle = '#ffe16a'; ctx.fillText(goal.text, p.x+25, p.y-25);
    }
    el('markers_status').textContent = [field.rangeStatus, field.cameraStatus, field.localizationStatus].filter(Boolean).join(' · ');
    el('status').textContent = !field.connected ? 'Field server unavailable' :
        !field.state?.aligned && field.state?.pose_source === 'marker_ekf' ? 'Waiting for EKF field alignment' :
        field.state?.odom_fresh ? (field.state?.pose_source === 'marker_ekf' ?
            'Field and fused localization active' : 'Field and robot alignment active') : 'Field ready · waiting for fresh odometry';
}
requestAnimationFrame(render);
