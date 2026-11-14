// Message/render smoke test without a browser or external JS dependencies.
const fs = require('fs'), vm = require('vm'), assert = require('assert').strict;
if (Number(process.versions.node.split('.')[0]) < 14 && !process.execArgv.includes('--harmony-optional-chaining'))
    process.exit(require('child_process').spawnSync(process.execPath,
        ['--harmony-optional-chaining', __filename], {stdio:'inherit'}).status);
const source = fs.readFileSync('src/ugv_else/vizanti/vizanti_server/public/templates/scene3d/scene3d_script.js','utf8')
    .replace(/^const \{rosbridge\} = await import.*$/m, 'const {rosbridge} = mocks;')
    .replace(/^const \{settings\} = await import.*$/m, 'const {settings} = mocks;')
    .replace(/^const \{FieldClient, transformFieldPoint\} = await import.*$/m, 'const {FieldClient, transformFieldPoint} = mocks;');
let frame, rectangles=0, now=1000;
let fieldClient, fieldClosed=false, groundFills=0, labels=[];
const topics = {}, elements = {};
const context = new Proxy({}, {get: (_, key) => key === 'fillRect' ? () => rectangles++ : key === 'fill' ? () => groundFills++ : key === 'fillText' ? text => labels.push(text) : () => {}});
for (const name of ['panel','canvas','follow','status','toggle','origin','clear','close','mode','field_view','markers_status'])
    elements[`{uniqueID}_${name}`] = {hidden:false, checked:false, clientWidth:1000, clientHeight:700,
        isConnected:true, getContext:()=>context, addEventListener:()=>{}, setPointerCapture:()=>{}};
const helperSource = fs.readFileSync('src/ugv_else/vizanti/vizanti_server/public/js/modules/football_field.js','utf8')
    .replace(/^import .*$/m,'').replace(/export /g, '');
const helpers = vm.runInNewContext(`(()=>{${helperSource}; return {transformFieldPoint,markerVisuals};})()`);
const state = vm.createContext({mocks:{rosbridge:{ros:{},compression:'none'}, settings:{save(){}},
    transformFieldPoint:helpers.transformFieldPoint, FieldClient:class {
        constructor(update){fieldClient=this;this.update=update;this.state=null;this.layout=null;this.connected=true;}
        bindControls(){} close(){fieldClosed=true;}
        get markerVisuals(){return helpers.markerVisuals(this.ranges,(now-1000)/1000,this.state && this.state.revision,this.layout && this.layout.frame_id);}
        get rangeStatus(){return 'AprilTag test ranges';}
    }},
    document:{getElementById:id=>elements[id]}, performance:{now:()=>now},
    requestAnimationFrame:callback=>{frame=callback;}, atob: v=>Buffer.from(v,'base64').toString('binary'),
    ROSLIB:{Topic:class {constructor(c){this.name=c.name;} subscribe(cb){topics[this.name]=cb;} unsubscribe(){delete topics[this.name];}}},
    Uint8Array, DataView, console});
vm.runInContext(source,state);
frame();
assert.match(elements['{uniqueID}_status'].textContent,/Waiting for \/odom/);
topics['/odom']({header:{frame_id:'odom',stamp:{sec:1,nanosec:0}},
    pose:{pose:{position:{x:1,y:2},orientation:{x:0,y:0,z:0,w:1}}}});
topics['/diagnostics']({status:[{name:'odometry/input_freshness',level:0,message:'fresh'}]});
const bytes=Buffer.alloc(12); bytes.writeFloatLE(2,0);bytes.writeFloatLE(0,4);bytes.writeFloatLE(1,8);
const cloud={header:{frame_id:'odom'},fields:['x','y','z'].map((name,i)=>({name,offset:i*4,datatype:7})),
    width:1,height:1,row_step:12,point_step:12,is_bigendian:false,data:bytes.toString('base64')};
topics['/vizanti/depth/points'](cloud); frame();
assert.match(elements['{uniqueID}_status'].textContent,/x 1.00 m · y 2.00 m/);
assert.match(elements['{uniqueID}_status'].textContent,/1 depth points/);
assert.equal(rectangles,1);
cloud.data = Array.from(bytes); topics['/vizanti/depth/points'](cloud);frame();
assert.equal(rectangles,2, 'byte-array cloud did not render');
fieldClient.layout={frame_id:'map',ground_color:'#28753c',ground:[[-5.2,-3.7,0],[5.2,-3.7,0],[5.2,3.7,0],[-5.2,3.7,0]],
    lines:[{points:[[-4.5,-3,0],[4.5,-3,0]],color:'#ffffff',width_m:.05}]};
fieldClient.state={aligned:true,transform:[2,-1,Math.PI/2],revision:1,
    start:{start_x_m:-3.2,start_y_m:0,start_heading_deg:0}};
fieldClient.ranges={frame_id:'map',field_revision:1,stream_fresh:true,markers:[{
    id:2,map_position_m:[0,-.75,0],robot_position_m:[-3.2,0,0],range_m:3.286,
    fresh:true,low_resolution:true,samples:5}]};
fieldClient.update();
topics['/vizanti/depth/points'](cloud);frame();
assert.match(elements['{uniqueID}_status'].textContent,/x 0.00 m · y 0.00 m · yaw 90.0°/);
assert.match(elements['{uniqueID}_mode'].textContent,/map · SPL field/);
assert(groundFills>0,'field ground did not render');
assert(labels.includes('ID 2 · ~3.3 m'),'AprilTag measured distance did not render in 3D');
assert.equal(rectangles,3,'cloud failed to render in field coordinates');
elements['{uniqueID}_origin'].onclick();frame();
elements['{uniqueID}_field_view'].onclick();frame();
const beforeStale=rectangles;
now=4000; frame();assert.equal(rectangles,beforeStale,'stale cloud remained visible');
assert(labels.includes('ID 2 · not visible'),'stale marker distance remained visible in 3D');
assert.match(elements['{uniqueID}_status'].textContent,/ODOM STALE/);
elements['{uniqueID}_toggle'].onclick();assert.equal(elements['{uniqueID}_panel'].hidden,true);
elements['{uniqueID}_canvas'].isConnected=false;frame();assert.equal(Object.keys(topics).length,0);
assert(fieldClosed,'field subscriptions were not released');
console.log('PASS: 3D field alignment, heading, cloud formats, rendering, stale data, views and cleanup');
