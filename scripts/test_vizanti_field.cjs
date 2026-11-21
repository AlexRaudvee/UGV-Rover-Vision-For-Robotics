// Exercise the actual field client, 2D renderer and controls without browser dependencies.
const fs=require('fs'), vm=require('vm'), assert=require('assert').strict;
if (Number(process.versions.node.split('.')[0]) < 14 && !process.execArgv.includes('--harmony-optional-chaining'))
    process.exit(require('child_process').spawnSync(process.execPath,
        ['--harmony-optional-chaining', __filename], {stdio:'inherit'}).status);
const base='src/ugv_else/vizanti/vizanti_server/public/';
const listeners={}, elements={}, calls=[], drawn=[];
let frame, now=1000;
const ctx=new Proxy({}, {get:(_,name)=> (...args)=>drawn.push([name,...args])});
for(const name of ['canvas','status','fit','save_start','apply_start','config_file','start_status','markers_status',
                  'start_x_m','start_y_m','start_heading_deg']) {
    elements[`{uniqueID}_${name}`]={isConnected:true,value:'',dataset:{fieldKey:name},
        addEventListener(event,cb){this[event]=cb;},getContext:()=>ctx};
    let value='';
    Object.defineProperty(elements[`{uniqueID}_${name}`],'value',
        {get(){return value;},set(v){value=String(v);}});
}
const mocks={rosbridge:{ros:{},compression:'none'},settings:{view:{},save(){}}};
mocks.view={center:{x:0,y:0},scale:50,fixedToScreen:p=>({x:500+p.x*50,y:400-p.y*50}),
    getMapUnitsInPixels:x=>x*50,sendUpdateEvent(){}};
mocks.tf={fixed_frame:'map',absoluteTransforms:{},setFixedFrame(frame){this.fixed_frame=frame;}};
const state=vm.createContext({mocks,console,performance:{now:()=>now},
    window:{innerWidth:1000,innerHeight:800},document:{getElementById:id=>elements[id]},
    requestAnimationFrame:cb=>{frame=cb;},setTimeout,clearTimeout,
    ROSLIB:{Topic:class {
        constructor(config){this.name=config.name;}
        subscribe(cb){listeners[this.name]=cb;} unsubscribe(){delete listeners[this.name];}
    },Service:class {
        constructor(config){this.name=config.name;}
        callService(request,success){calls.push([this.name,request]);success(this.name.endsWith('apply_start')?
            {success:true}:{result:{successful:true}});}
    },ServiceRequest:class {constructor(values){Object.assign(this,values);}}}});
const moduleSource=fs.readFileSync(base+'js/modules/football_field.js','utf8')
    .replace(/^import .*$/m,'const {rosbridge}=mocks;').replace(/export /g,'');
    vm.runInContext(`Object.assign(mocks,(()=>{${moduleSource};return {FieldClient,transformFieldPoint,markerVisuals,cameraFov,triangulationVisual};})());`,state);
const source=fs.readFileSync(base+'templates/footballfield/footballfield_script.js','utf8')
    .replace(/^const (\{.*\}) = await import.*$/gm,'const $1 = mocks;');
vm.runInContext(source,state);
listeners['/football_field/layout']({data:JSON.stringify({frame_id:'map',length_m:9,width_m:6,border_m:.7,
    ground_color:'#28753c',ground:[[-5.2,-3.7,0],[5.2,-3.7,0],[5.2,3.7,0],[-5.2,3.7,0]],
    lines:[{points:[[-4.5,-3,0],[4.5,-3,0]],color:'#fff',width_m:.05}]})});
const fieldState={config_file:'/workspace/football_field.yaml',frame_id:'map',robot:[-3.2,0,0],aligned:true,odom_fresh:true,publish_tf:true,revision:2,
    start:{start_x_m:-3.2,start_y_m:0,start_heading_deg:0},error:''};
listeners['/football_field/state']({data:JSON.stringify(fieldState)});
const ranges={frame_id:'map',field_revision:2,stream_fresh:true,error:'',markers:[{
    id:2,map_position_m:[0,-.75,0],robot_position_m:[-3.2,0,0],range_m:3.286,
    fresh:true,low_resolution:true,samples:5}]};
listeners['/marker_ranges/result']({data:JSON.stringify(ranges)});
const camera={ready:true,pan_deg:90,left_rad:Math.PI/4,right_rad:Math.PI/4,
    origin_base:[.04,0,.01],range_m:4,horizontal_fov_deg:90,mode:'tracking_markers',tracked_ids:[0,4,5]};
listeners['/marker_pan_tracker/state']({data:JSON.stringify(camera)});
const fix={fresh:true,frame_id:'map',field_revision:2,position_m:[-2.9,.2,0],heading_deg:15,
    odom_position_m:[-3.2,0,0],position_delta_m:.36,age_s:.1,stale_after_s:1,used_ids:['tag:0','tag:5']};
listeners['/marker_triangulation/result']({data:JSON.stringify(fix)});
frame(1000);
assert(drawn.some(d=>d[0]==='fillText' && d[1]==='ODOM (-3.20, 0.00)'),'live odometry marker missing in 2D');
assert(drawn.some(d=>d[0]==='fillText' && d[1]==='TRIANGULATION (-2.90, 0.20)'),
    'independent triangulation marker missing in 2D');
assert.match(elements['{uniqueID}_markers_status'].textContent,/Δ odometry 0.36 m/);
assert.equal(mocks.triangulationVisual(fix,1,2,'map'),null,'expired triangulation remained live');
assert.equal(mocks.triangulationVisual(fix,0,3,'map'),null,'old start revision retained a comparison');
assert.equal(mocks.triangulationVisual(fix,0,2,'odom'),null,'triangulation used the wrong frame');
assert.equal(mocks.triangulationVisual({...fix,fresh:false},0,2,'map'),null,'rejected fix remained visible');
assert(drawn.some(d=>d[0]==='moveTo' && Math.abs(d[1]-342)<1e-9 && d[2]===400),'camera FOV origin is wrong');
assert.match(elements['{uniqueID}_markers_status'].textContent,/Camera view \(estimated\) 90°/);
assert.match(elements['{uniqueID}_markers_status'].textContent,/tracking 3 tags/);
const wedge=mocks.cameraFov(camera,{...fieldState,robot:[1,2,Math.PI/2]},0);
assert.equal(wedge.length,27);
assert(Math.abs(wedge[13][0]+3)<1e-12 && Math.abs(wedge[13][1]-2.04)<1e-12,
    'FOV failed to combine robot yaw and camera pan');
assert.equal(mocks.cameraFov(camera,fieldState,2).length,0,'stale camera FOV remained visible');
assert.equal(mocks.cameraFov(camera,{...fieldState,odom_fresh:false},0).length,0,'FOV used stale robot pose');
assert.equal(mocks.cameraFov({...camera,ready:false},fieldState,0).length,0);
assert(drawn.some(d=>d[0]==='moveTo' && d[1]===240 && d[2]===585),'carpet scale/origin is wrong');
assert(drawn.some(d=>d[0]==='fillText' && d[1]==='START (-3.20, 0.00)'),'start label missing');
assert(drawn.some(d=>d[0]==='fillText' && d[1]==='ID 2 · ~3.3 m'),'measured AprilTag distance missing in 2D');
assert(mocks.markerVisuals(ranges,0,3,'map').every(m=>!m.fresh),'previous start alignment reused a live distance');
assert(mocks.markerVisuals(ranges,3,2,'map').every(m=>!m.fresh),'stale distance remained live');
assert.equal(mocks.markerVisuals(ranges,0,2,'different_frame').length,0);
const rejected={...ranges,markers:[{...ranges.markers[0],fresh:false,detected:true,detection_age_s:0,
    rejection_reason:'corrected-bit count exceeds the configured limit'}]};
assert.match(mocks.markerVisuals(rejected,0,2,'map')[0].text,/range rejected/);
const almostStale={...ranges,stale_after_s:1,markers:[{...ranges.markers[0],age_s:.9}]};
assert(!mocks.markerVisuals(almostStale,.2,2,'map')[0].fresh,'browser retained an expired measurement');
assert.equal(elements['{uniqueID}_start_x_m'].value,'-3.2');
assert.equal(elements['{uniqueID}_save_start'].disabled,false);
const mapped=mocks.transformFieldPoint([2,-1,Math.PI/2],[1,2,3]);
assert(Math.abs(mapped[0])<1e-12 && Math.abs(mapped[1])<1e-12 && mapped[2]===3);
async function main(){
    drawn.length=0;
    listeners['/marker_triangulation/result']({data:JSON.stringify({...fix,fresh:false,error:'need at least two markers'})});
    frame(1100);
    assert(!drawn.some(d=>d[0]==='fillText' && d[1].startsWith('TRIANGULATION')),'invalidated fix remained drawn');
    assert.match(elements['{uniqueID}_markers_status'].textContent,/need at least two markers/);
    const input=elements['{uniqueID}_start_x_m']; input.value='1.25';input.input();
    listeners['/football_field/state']({data:JSON.stringify(fieldState)});
    assert.equal(input.value,'1.25','live updates overwrote an edited input');
    elements['{uniqueID}_start_y_m'].value='0';elements['{uniqueID}_start_heading_deg'].value='90';
    await elements['{uniqueID}_save_start'].onclick();
    assert.equal(calls[0][0],'/football_field/set_parameters_atomically');
    assert.equal(calls[0][1].parameters[0].value.double_value,1.25);
    assert.equal(calls[0][1].parameters[2].value.double_value,90);
    assert.match(elements['{uniqueID}_start_status'].textContent,/Saved start applied/);
    await elements['{uniqueID}_apply_start'].onclick();
    assert.equal(calls[1][0],'/football_field/apply_start');
    elements['{uniqueID}_start_x_m'].value='';elements['{uniqueID}_start_x_m'].input();
    await elements['{uniqueID}_save_start'].onclick();
    assert.equal(calls.length,2,'empty input sent an invalid start');
    assert.match(elements['{uniqueID}_start_status'].textContent,/Enter a number/);
    elements['{uniqueID}_fit'].onclick();assert.equal(mocks.tf.fixed_frame,'map');
    listeners['/football_field/state']({data:JSON.stringify({...fieldState,publish_tf:false,
        start_control_enabled:true,pose_source:'marker_ekf'})});
    listeners['/marker_triangulation/result']({data:JSON.stringify(fix)});
    drawn.length=0;frame(1200);
    assert(drawn.some(d=>d[0]==='fillText' && d[1]==='FUSED (-3.20, 0.00)'),
        'EKF-aligned robot was mislabeled as raw odometry');
    assert.match(elements['{uniqueID}_markers_status'].textContent,/Δ fused pose 0.36 m/);
    assert.equal(elements['{uniqueID}_save_start'].disabled,false,'EKF start controls were disabled');
    assert.equal(elements['{uniqueID}_apply_start'].disabled,false,'EKF reset was disabled');
    listeners['/football_field/state']({data:JSON.stringify({...fieldState,publish_tf:false,
        start_control_enabled:false,pose_source:'external'})});
    assert.equal(elements['{uniqueID}_apply_start'].disabled,true,'unrelated external localization can be reset');
    listeners['/football_field/state']({data:JSON.stringify({...fieldState,odom_fresh:false})});
    assert.equal(elements['{uniqueID}_apply_start'].disabled,true,'stale odometry can be reapplied');
    elements['{uniqueID}_canvas'].isConnected=false;frame(2000);
    assert.equal(Object.keys(listeners).length,0);
    // Existing browser layouts receive the field without losing camera placement.
    const defaults=JSON.parse(fs.readFileSync('src/polar_hist/config/vizanti_layout.json','utf8'));
    const previous={navbar:[{type:'settings',id:'settings_default'},
        {type:'compressedimage',id:'perception_rgb'},{type:'grid',id:'my_grid'}],
        layout_version:'older',view:{center:{x:2,y:1},scale:60},
        settings_default:{fixed_frame:'odom',background_color:'#123456'},
        perception_rgb:{width:'19',img_offset_x:'27',topic:'/old'}};
    const storage={settings:JSON.stringify(previous),getItem(k){return this[k];},setItem(k,v){this[k]=v;}};
    const migrationSource=fs.readFileSync(base+'js/modules/persistent.js','utf8')
        .replace(/^const defaultConfigModule = await import.*$/m,'const defaultConfigModule = {default: JSON.stringify(defaults)};')
        .replace(/export /g,'');
    const migration=vm.createContext({defaults,localStorage:storage,console});
    vm.runInContext(`${migrationSource}; globalThis.result = settings;`,migration);
    assert.equal(migration.result.settings_default.fixed_frame,'map');
    assert.equal(migration.result.settings_default.background_color,'#123456');
    assert.equal(migration.result.perception_rgb.width,'19');
    assert.equal(migration.result.perception_rgb.img_offset_x,'27');
    assert(migration.result.navbar.some(w=>w.id==='football_field'));
    assert(migration.result.navbar.some(w=>w.id==='field_robot'));
    assert(migration.result.navbar.some(w=>w.id==='my_grid'));
    vm.runInContext('globalThis.reload = new Settings();',migration);
    assert.equal(migration.reload.navbar.filter(w=>w.id==='football_field').length,1);
    console.log('PASS: 2D odometry/triangulation comparison, FOV, alignment, controls, expiry and cleanup');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
