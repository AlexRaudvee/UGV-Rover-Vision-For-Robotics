// Message/render smoke test without a browser or external JS dependencies.
const fs = require('fs'), vm = require('vm'), assert = require('assert').strict;
const source = fs.readFileSync('src/ugv_else/vizanti/vizanti_server/public/templates/scene3d/scene3d_script.js','utf8')
    .replace(/^const \{rosbridge\} = await import.*$/m, 'const {rosbridge} = mocks;')
    .replace(/^const \{settings\} = await import.*$/m, 'const {settings} = mocks;');
let frame, rectangles=0, now=1000;
const topics = {}, elements = {};
const context = new Proxy({}, {get: (_, key) => key === 'fillRect' ? () => rectangles++ : () => {}});
for (const name of ['panel','canvas','follow','status','toggle','origin','clear','close'])
    elements[`{uniqueID}_${name}`] = {hidden:false, checked:false, clientWidth:1000, clientHeight:700,
        isConnected:true, getContext:()=>context, addEventListener:()=>{}, setPointerCapture:()=>{}};
const state = vm.createContext({mocks:{rosbridge:{ros:{},compression:'none'}, settings:{save(){}}},
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
now=4000; frame();assert.equal(rectangles,2,'stale cloud remained visible');
assert.match(elements['{uniqueID}_status'].textContent,/ODOM STALE/);
elements['{uniqueID}_toggle'].onclick();assert.equal(elements['{uniqueID}_panel'].hidden,true);
elements['{uniqueID}_canvas'].isConnected=false;frame();assert.equal(Object.keys(topics).length,0);
console.log('PASS: 3D widget pose, cloud formats, rendering, stale data, toggle and cleanup');
