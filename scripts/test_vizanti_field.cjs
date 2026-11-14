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
    vm.runInContext(`Object.assign(mocks,(()=>{${moduleSource};return {FieldClient,transformFieldPoint,markerVisuals};})());`,state);
const source=fs.readFileSync(base+'templates/footballfield/footballfield_script.js','utf8')
    .replace(/^const (\{.*\}) = await import.*$/gm,'const $1 = mocks;');
vm.runInContext(source,state);
listeners['/football_field/layout']({data:JSON.stringify({frame_id:'map',length_m:9,width_m:6,border_m:.7,
    ground_color:'#28753c',ground:[[-5.2,-3.7,0],[5.2,-3.7,0],[5.2,3.7,0],[-5.2,3.7,0]],
    lines:[{points:[[-4.5,-3,0],[4.5,-3,0]],color:'#fff',width_m:.05}]})});
const fieldState={config_file:'/workspace/football_field.yaml',aligned:true,odom_fresh:true,publish_tf:true,revision:2,
    start:{start_x_m:-3.2,start_y_m:0,start_heading_deg:0},error:''};
listeners['/football_field/state']({data:JSON.stringify(fieldState)});
const ranges={frame_id:'map',field_revision:2,stream_fresh:true,error:'',markers:[{
    id:2,map_position_m:[0,-.75,0],robot_position_m:[-3.2,0,0],range_m:3.286,
    fresh:true,low_resolution:true,samples:5}]};
listeners['/marker_ranges/result']({data:JSON.stringify(ranges)});
frame(1000);
assert(drawn.some(d=>d[0]==='moveTo' && d[1]===240 && d[2]===585),'carpet scale/origin is wrong');
assert(drawn.some(d=>d[0]==='fillText' && d[1]==='START (-3.20, 0.00)'),'start label missing');
assert(drawn.some(d=>d[0]==='fillText' && d[1]==='ID 2 · ~3.3 m'),'measured AprilTag distance missing in 2D');
assert(mocks.markerVisuals(ranges,0,3,'map').every(m=>!m.fresh),'previous start alignment reused a live distance');
assert(mocks.markerVisuals(ranges,3,2,'map').every(m=>!m.fresh),'stale distance remained live');
assert.equal(mocks.markerVisuals(ranges,0,2,'different_frame').length,0);
assert.equal(elements['{uniqueID}_start_x_m'].value,'-3.2');
assert.equal(elements['{uniqueID}_save_start'].disabled,false);
const mapped=mocks.transformFieldPoint([2,-1,Math.PI/2],[1,2,3]);
assert(Math.abs(mapped[0])<1e-12 && Math.abs(mapped[1])<1e-12 && mapped[2]===3);
async function main(){
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
    console.log('PASS: 2D field scale, shared alignment, persistent-pose service, input editing, stale data and cleanup');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
