// A perspective view over existing ROS messages; no new rendering dependency.
const {rosbridge} = await import(`${base_url}/js/modules/rosbridge.js`);
const {settings} = await import(`${base_url}/js/modules/persistent.js`);
const el = name => document.getElementById(`{uniqueID}_${name}`);
const panel = el('panel'), canvas = el('canvas'), ctx = canvas.getContext('2d');
const config = settings['{uniqueID}'] || {enabled: true};
let azimuth = -2.2, elevation = .8, distance = 7, target = [0, 0, 0];
let robot, points = [], trail = [], lastOdom = 0, lastCloud = 0, health;
let cloudError = '', drag, previousStamp = 0, cloudStamp = 0;
const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
function toggle(enabled) {
    panel.hidden = !enabled;
    settings['{uniqueID}'] = {...config, enabled}; settings.save();
}
function origin() { el('follow').checked = false; target = [0, 0, 0]; distance = 7; }
el('toggle').onclick = () => toggle(panel.hidden);
el('close').onclick = () => toggle(false);
el('origin').onclick = origin;
el('clear').onclick = () => { trail = []; };
toggle(config.enabled !== false);
canvas.onpointerdown = event => {
    drag = [event.clientX, event.clientY]; canvas.setPointerCapture(event.pointerId);
    event.stopPropagation();
};
canvas.onpointermove = event => {
    if (!drag) return;
    azimuth -= (event.clientX - drag[0]) * .008;
    elevation = clamp(elevation + (event.clientY - drag[1]) * .008, .15, 1.45);
    drag = [event.clientX, event.clientY]; event.stopPropagation();
};
canvas.onpointerup = canvas.onpointercancel = () => { drag = undefined; };
canvas.addEventListener('wheel', event => {
    distance = clamp(distance * Math.exp(event.deltaY * .001), 1, 60);
    event.preventDefault(); event.stopPropagation();
}, {passive: false});
// Keep the 2D canvas underneath from handling orbit gestures.
for (const name of ['mousedown', 'mousemove', 'mouseup', 'touchstart', 'touchmove'])
    panel.addEventListener(name, event => event.stopPropagation());
const topics = [];
function subscribe(name, messageType, callback, throttle_rate) {
    const topic = new ROSLIB.Topic({ros: rosbridge.ros, name, messageType,
        throttle_rate, queue_length: 1, compression: rosbridge.compression});
    topic.subscribe(callback); topics.push(topic);
}
subscribe('/odom', 'nav_msgs/msg/Odometry', msg => {
    const p = msg.pose.pose.position, q = msg.pose.pose.orientation;
    if (msg.header.frame_id !== 'odom' || ![p.x,p.y,q.x,q.y,q.z,q.w].every(Number.isFinite)) return;
    const stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
    if (stamp < previousStamp) trail = [];
    previousStamp = stamp;
    robot = {x: p.x, y: p.y, yaw: Math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))};
    lastOdom = performance.now();
    const last = trail[trail.length-1];
    if (!last || Math.hypot(p.x-last[0], p.y-last[1]) > .02) {
        trail.push([p.x,p.y,.015]); if (trail.length > 1500) trail.shift();
    }
}, 100);
subscribe('/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', msg => {
    const status = msg.status.find(s => s.name === 'odometry/input_freshness');
    if (status) health = {level: status.level, text: status.message, time: performance.now()};
}, 500);
subscribe('/vizanti/depth/points', 'sensor_msgs/msg/PointCloud2', msg => {
    try {
        if (msg.header.frame_id !== 'odom') throw Error('cloud must be in odom');
        const fields = ['x','y','z'].map(name => msg.fields.find(f => f.name === name && f.datatype === 7));
        if (fields.some(f => !f)) throw Error('cloud requires FLOAT32 XYZ');
        const bytes = typeof msg.data === 'string' ? Uint8Array.from(atob(msg.data), c => c.charCodeAt(0)) : new Uint8Array(msg.data);
        const data = new DataView(bytes.buffer), decoded = [];
        const skip = Math.max(1, Math.ceil(msg.width * msg.height / 6000));
        for (let index=0; index<msg.width*msg.height; index+=skip) {
            const offset = Math.floor(index/msg.width)*msg.row_step + (index%msg.width)*msg.point_step;
            const point = fields.map(f => data.getFloat32(offset+f.offset, !msg.is_bigendian));
            if (point.every(Number.isFinite)) decoded.push(point);
        }
        points = decoded; lastCloud = performance.now(); cloudError = '';
        cloudStamp = msg.header.stamp ? msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 : 0;
    } catch (error) { points = []; cloudError = String(error); }
}, 200);
function render() {
    if (!canvas.isConnected) { for (const topic of topics) topic.unsubscribe(); return; }
    requestAnimationFrame(render);
    if (panel.hidden) return;
    const w = panel.clientWidth, h = panel.clientHeight;
    if (canvas.width !== w || canvas.height !== h) { canvas.width=w; canvas.height=h; }
    ctx.clearRect(0,0,w,h);
    if (robot && el('follow').checked) target = [robot.x,robot.y,0];
    const ca=Math.cos(azimuth), sa=Math.sin(azimuth), ce=Math.cos(elevation), se=Math.sin(elevation);
    function project(p) {
        const x=p[0]-target[0], y=p[1]-target[1], z=p[2]-target[2];
        const depth=distance-ce*(ca*x+sa*y)-se*z;
        if (depth <= .1) return null;
        const f=Math.min(w,h)*.95/depth;
        return [w/2+f*(-sa*x+ca*y), h/2-f*(-se*(ca*x+sa*y)+ce*z), depth];
    }
    function line(a,b,color,width=1) {
        a=project(a); b=project(b); if (!a || !b) return;
        ctx.strokeStyle=color; ctx.lineWidth=width; ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke();
    }
    const gx=Math.floor(target[0]), gy=Math.floor(target[1]);
    const floor=[[-10+gx,-10+gy,0],[10+gx,-10+gy,0],[10+gx,10+gy,0],[-10+gx,10+gy,0]].map(project);
    if (floor.every(Boolean)) { ctx.fillStyle='#162631'; ctx.beginPath(); floor.forEach(p=>ctx.lineTo(p[0],p[1])); ctx.closePath(); ctx.fill(); }
    for(let i=-10;i<=10;i++) {
        line([gx+i,gy-10,0],[gx+i,gy+10,0],'#304451');
        line([gx-10,gy+i,0],[gx+10,gy+i,0],'#304451');
    }
    line([0,0,0],[1,0,0],'#5ee89c',3); line([0,0,0],[0,1,0],'#5ee89c',3);
    line([0,0,0],[0,0,1],'#5ee89c',3);
    const label=project([0,0,.05]);
    if(label) { ctx.fillStyle='#8ff0b7';ctx.font='13px sans-serif';ctx.fillText('START (0, 0)',label[0]+8,label[1]-8); }
    for(let i=1;i<trail.length;i++) line(trail[i-1],trail[i],'#55baff',2);
    if(performance.now()-lastCloud < 1000) {
        const visible=points.map(p=>({p:project(p),z:p[2]})).filter(p=>p.p).sort((a,b)=>b.p[2]-a.p[2]);
        for(const {p,z} of visible) { ctx.fillStyle=`hsl(${clamp(210-z*65,0,260)},85%,65%)`;ctx.fillRect(p[0]-1,p[1]-1,2.5,2.5); }
    }
    if(robot) {
        const body=(x,y,z)=>[robot.x+Math.cos(robot.yaw)*x-Math.sin(robot.yaw)*y,robot.y+Math.sin(robot.yaw)*x+Math.cos(robot.yaw)*y,z];
        const box=(cx,cy,bz,dx,dy,dz,color)=>{
            const vertices=[];
            for(const z of [bz,bz+dz]) for(const [x,y] of [[-dx,-dy],[dx,-dy],[dx,dy],[-dx,dy]]) vertices.push(body(cx+x,cy+y,z));
            for(let i=0;i<4;i++) { line(vertices[i],vertices[(i+1)%4],color,2);line(vertices[i+4],vertices[(i+1)%4+4],color,2);line(vertices[i],vertices[i+4],color,2); }
        };
        box(0,0,.07,.16,.11,.14,'#f7c76b');
        for(const x of [-.11,.11]) for(const y of [-.14,.14]) box(x,y,0,.055,.035,.11,'#a4b2be');
        line(body(0,0,.23),body(.30,0,.23),'#ff956d',3);
    }
    const now=performance.now(), odomFresh=lastOdom && now-lastOdom<1000;
    const state=robot ? `x ${robot.x.toFixed(2)} m · y ${robot.y.toFixed(2)} m · yaw ${(robot.yaw*180/Math.PI).toFixed(1)}°` : 'Waiting for /odom';
    const inputState=health && now-health.time<2000 ? (health.level ? health.text : 'estimation inputs fresh') : 'input health unavailable';
    el('status').textContent=`${state}${odomFresh?'':' · ODOM STALE'} · ${lastCloud && now-lastCloud<1000?`${points.length} depth points · delay ${Math.max(0, previousStamp-cloudStamp).toFixed(1)} s`:(cloudError || 'depth unavailable/stale')} · ${inputState}`;
}
requestAnimationFrame(render);
