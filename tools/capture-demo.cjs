// Only preset display names are personal. Geometry, identifiers and room data
// are invented here; the capture never reads production data or contacts a robot.
const {chromium} = require('playwright');
const {PNG} = require('pngjs');
const fs = require('node:fs'), path = require('node:path'), assert = require('node:assert/strict');
const root = path.resolve(__dirname, '..'), app = path.join(root, 'app'), media = path.join(root, 'docs/media');
fs.mkdirSync(media, {recursive:true});
// A synthetic occupancy scan with coarse cells and almost clean wall edges.
// No real map is used as a template.
const MAP_BG='#17578f', CELL=4, COLS=200, ROWS=140;
const palette={16:'#db8e89',17:'#b7a3d1',18:'#f3b744',19:'#f0ca68',20:'#80c3dc'};
let seed=0x31a8d21;
function random(){seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;}
function segmentAt(x,y){
 if(x>=88&&x<322&&y>=72&&y<248)return 16;
 if(x>=328&&x<488&&y>=72&&y<248)return 17;
 if(x>=492&&x<704&&y>=72&&y<280)return 18;
 if((x>=88&&x<488&&y>=252&&y<340)||(x>=88&&x<284&&y>=340&&y<392)||(x>=292&&x<456&&y>=336&&y<496))return 19;
 if((x>=492&&x<704&&y>=284&&y<456)||(x>=460&&x<492&&y>=344&&y<456))return 20;
 return 0;
}
const cells=Array.from({length:COLS*ROWS},(_,i)=>segmentAt((i%COLS)*CELL,Math.floor(i/COLS)*CELL));
const scan=new PNG({width:800,height:560}),index=new PNG({width:800,height:560});
function rgb(hex){return hex.match(/[a-f0-9]{2}/gi).map(x=>parseInt(x,16));}
function cell(x,y,color,id){
 if(x<0||y<0||x>=COLS||y>=ROWS)return;
 const c=rgb(color);
 for(let dy=0;dy<CELL;dy++)for(let dx=0;dx<CELL;dx++){
  const i=((y*CELL+dy)*800+x*CELL+dx)*4;
  scan.data.set([...c,255],i);
  if(id!==undefined)index.data.set([id,id,id,255],i);
 }
}
for(let y=0;y<ROWS;y++)for(let x=0;x<COLS;x++){
 const id=cells[y*COLS+x];
 const near=!id&&[-1,0,1].some(dy=>[-1,0,1].some(dx=>cells[(y+dy)*COLS+x+dx]));
 cell(x,y,id?palette[id]:near?'#718a9d':MAP_BG,id);
}
// Wall runs retain passage gaps and only occasional one-cell edge irregularities.
// Nothing here draws door leaves, opening arcs, tiles, windows or furniture icons.
const walls=[
 [88,72,322,72],[328,72,488,72],[492,72,704,72],[704,72,704,456],
 [460,456,704,456],[456,456,456,496],[292,496,456,496],[292,392,292,496],
 [88,392,292,392],[88,72,88,392],[324,72,324,248],
 [88,248,236,248],[288,248,388,248],[436,248,488,248],
 [488,72,488,196],[488,244,488,344],[460,344,488,344],
 [492,280,572,280],[624,280,704,280],
 // Partial returns from objects near walls, deliberately unlabelled.
 [508,92,592,92],[592,92,592,108],[652,92,684,92],
 [684,92,684,204],[668,204,684,204],[112,88,172,88],[112,88,112,104],
 [340,92,364,92],[364,92,364,124],[340,124,364,124],
 [664,356,684,356],[684,356,684,420],[652,420,684,420]
];
for(const [x0,y0,x1,y1] of walls){
 const n=Math.max(Math.abs(x1-x0),Math.abs(y1-y0))/CELL;
 for(let i=0;i<=n;i++){
  const x=Math.round((x0+(x1-x0)*i/n)/CELL),y=Math.round((y0+(y1-y0)*i/n)/CELL);
  cell(x,y,'#17262e');
  if(random()<.003)cell(x+1,y,'#314451');
 }
}
// Small dock return, without a drawing of the room's contents.
for(let x=83;x<94;x++)cell(x,122,'#d3d9d8');
const map=PNG.sync.write(scan);
const rooms = [
 ['Chambre démo',16,6,88,71,234,176], ['Bain démo',17,7,328,71,158,176],
 ['Cuisine démo',18,8,492,71,212,207], ['Passage démo',19,12,88,253,398,84],
 ['Séjour démo',20,1,492,284,212,170]
].map(([name,segment_id,tag,x,y,w,h]) => ({name,segment_id,tag,center:[(x+w/2)/8,(y+h/2)/5.6],bbox:[x/8,y/5.6,w/8,h/5.6]}));
const roomIndex={key:'invented-lidar-scan-v3',png:PNG.sync.write(index).toString('base64')};
const modes = {fan:102,water:200,mop:300,repeat:1};
function roomPreset(id,name,segments,settings,group) {
 return {id,name,kind:'segment',endpoint:'/api/clean',group,body:{segments,...settings},summary:{rooms:segments,names:segments.map(id=>rooms.find(r=>r.segment_id===id).name),modes:settings,map_flag:0}};
}
const presets = [
 roomPreset('fast','⚡ Fast main rooms 🌪️',[20],{...modes,fan:103,mop:304},'main'),
 roomPreset('deep','✨ Deep main rooms 🌪️',[19,20],{...modes,fan:104},'main'),
 {id:'cooking',name:'🧑‍🍳 After cooking 🌪️',kind:'zone',endpoint:'/api/zone',body:{zones:[
  {name:'Heating plate',rect:[625,125,700,262],...modes,fan:103},
  {name:'Sink',rect:[505,115,658,186],...modes,fan:108}
 ],...modes},summary:{zones:2,modes,map_flag:0}},
 roomPreset('bath','🧴 Shower + Toilet 🌪️+🧽',[17],{...modes,fan:104,water:240,repeat:2}),
 roomPreset('whole','🏠 Full house 🌪️+🧽',[19,16,17,18,20],{...modes,water:235})
];
const state = {devices:[{duid:'demo',name:'Robot démo',model:'roborock.vacuum.a144'}],selected:'demo',map_bg:MAP_BG,map_v:'synthetic',rooms,room_index:roomIndex,overlays:rooms.map(r=>({segment_id:r.segment_id,left_pct:r.center[0],top_pct:r.center[1]})),maps:{current:0,list:[{flag:0,name:'Appartement fictif'}]},options:{fan:{quiet:101,balanced:102,turbo:103,max:104,max_plus:108},water:{off:200,low:225,medium:235,high:245,extreme:250},mop:{standard:300,deep:301,deep_plus:303,fast:304}},status:{state:8,state_label:'charging',state_trusted:true,running:false,battery:100,in_cleaning:0,in_returning:0,fan_power:102,water_box_mode:200,mop_mode:300,repeat:1,clean_area:0,clean_time:0,error:{code:0,name:'none',severity:'none'},parts:{missing:[],busy_labels:[]},position:{pct:[44,83],at_dock:true}}};
(async()=>{
 const browser = await chromium.launch({headless:true,args:['--no-sandbox']});
 try {
  const page = await browser.newPage({viewport:{width:720,height:920},deviceScaleFactor:2,colorScheme:'dark',reducedMotion:'reduce'});
  const errors=[], posts=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.addInitScript(()=>{window.WebSocket=class{constructor(){this.readyState=0;}close(){}send(){}};});
  await page.route('**/*',async r=>{
   const u=new URL(r.request().url()),p=u.pathname;
   if(u.origin!=='http://demo.invalid')return r.abort();
   if(p==='/')return r.fulfill({contentType:'text/html',body:fs.readFileSync(path.join(app,'index.html'))});
   if(p==='/api/map.png')return r.fulfill({contentType:'image/png',body:map});
   if(/^\/(static|modeicons|roomtags|obsticons)\//.test(p)){
    const rel=p.startsWith('/static/')?p.slice(1):'static'+p,target=path.resolve(app,rel);
    if(!target.startsWith(app+path.sep)||!fs.existsSync(target))return r.fulfill({status:404,body:'missing demo asset'});
    return r.fulfill({contentType:target.endsWith('.svg')?'image/svg+xml':target.endsWith('.js')?'application/javascript':target.endsWith('.png')?'image/png':'application/octet-stream',body:fs.readFileSync(target)});
   }
   if(r.request().method()==='POST')posts.push(p);
   let data={};
   if(p==='/api/state')data=state;
   if(p==='/api/status')data=state.status;
   if(p==='/api/trace')data={w:800,h:560,mm_px:12.5,pts:[],goto:[],n:0};
   if(p==='/api/presets')data={presets};
   if(p==='/api/obstacles')data={obstacles:[]};
   if(p==='/api/queue')data={count:0,jobs:[]};
   return r.fulfill({contentType:'application/json',body:JSON.stringify(data)});
  });
  await page.goto('http://demo.invalid');
  await page.waitForFunction(()=>latest?.rooms?.length===5&&PRESETS.length===5);
  await page.waitForFunction(()=>document.getElementById('map').naturalWidth===800);
  await page.locator('#map').evaluate(img=>img.decode());
  await page.waitForFunction(()=>roomIdx&&getComputedStyle(document.getElementById('mapph')).display==='none');
  if(process.argv.includes('--check-selection')){
   await require('./check-room-selection.cjs')(page);
   assert.deepEqual(errors,[]);
   assert.deepEqual(posts,[], 'selection checks must not issue commands');
   return;
  }
  await page.locator('.preset[data-id="fast"]').click();
  await page.locator('.preset[data-id="bath"]').click();
  await page.mouse.move(0,0);
  assert.deepEqual(await page.evaluate(()=>[...previewSegs].sort((a,b)=>a-b)),[17,20], 'only the two selected rooms should be highlighted');
  // The combined-job preview veils rooms outside the selected presets.
  await page.locator('#fab_primary').click();
  await page.waitForFunction(()=>previewVeil&&document.getElementById('presetsheet').classList.contains('show'));
  await page.waitForTimeout(350);
  await page.screenshot({path:path.join(media,'combined-presets.png')});
  await page.evaluate(()=>{closePresetSheet();clearPresetSel();openPresetSheet(PRESETS.find(p=>p.id==='cooking'));});
  await page.waitForTimeout(350);
  await page.screenshot({path:path.join(media,'zone-presets.png')});
  assert.deepEqual(errors,[]);
  assert.deepEqual(posts,[], 'documentation capture must not issue commands');
  console.log('Generated two screenshots with invented rooms and geometry; no hardware requests.');
 } finally { await browser.close(); }
})().catch(e=>{console.error(e);process.exit(1);});
