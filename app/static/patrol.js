/* Tap destinations on the floor map; saved routes use native map coordinates.
   The server owns sequencing, arrival checks and the connection lease. */
let patrolMap=null, patrolRoutes=[], patrolSteps=[], patrolId=null, patrolEditing=false;
let patrolRun=null, patrolRemote={active:false}, patrolPending=false, patrolStartRequest=null;
let patrolTimer=null, patrolEpoch=0, patrolMapReady=false;
let patrolView="saved";
const patrolBusy=()=>patrolPending || !!patrolRun || !!patrolRemote.active;
function patrolEditorNote(text,error=false){
  $("patrol_editor_status").textContent=error?text.replace(/^\d{3}[:\s]+/,""):text;
  $("patrol_editor_status").classList.toggle("err",error);
}
function patrolNote(text,error=false){
  $("patrol_status").textContent=text; $("patrol_status").classList.toggle("err",error);
  if(patrolEditing)patrolEditorNote(text,error);
}
function patrolMode(mode,force=false){
  if(patrolBusy() && !force) return;
  patrolView=mode;
  const editing=mode==="edit";
  patrolEditing=editing;
  patrolEditorNote("");
  $("patrol_editor").hidden=!editing;
  $("patrol_direct_home").hidden=mode!=="direct";
  $("patrol_saved_panel").hidden=mode!=="saved";
  $("patrol_map_wrap").hidden=mode==="saved";
  $("patrol_hint").hidden=mode==="saved";
  for(const tab of ["saved","direct"]){
    const selected=mode===tab || (editing && tab==="saved");
    $("patrol_"+tab).classList.toggle("on",selected);
    $("patrol_"+tab).setAttribute("aria-pressed",String(selected));
  }
  $("patrol_hint").textContent=editing ? "Touchez la carte pour ajouter les destinations dans l’ordre, puis ajoutez des rotations ou des pauses." : "Touchez un point sur la carte pour y aller. La caméra reste ouverte à l’arrivée.";
  patrolDraw(); patrolRenderSteps();
}
function patrolSync(){
  const busy=patrolBusy();
  $("patrol_stop").hidden=!busy;
  $("rc_arm").disabled=busy;
  $("ch_menu").disabled=busy;
  $("rc_halt").classList.toggle("live",busy || rcOn);
  $("patrol_map").setAttribute("aria-disabled",String(busy || !patrolMapReady));
  for(const id of ["patrol_saved","patrol_direct","patrol_edit","patrol_name","patrol_turn","patrol_angle","patrol_speed","patrol_wait","patrol_new","patrol_save","patrol_play","patrol_direct_home"]){
    $(id).disabled=busy || !patrolMapReady;
  }
  $("patrol_editor").querySelectorAll("li button").forEach(b=>b.disabled=busy || b.dataset.edge==="1");
  $("patrol_editor").querySelectorAll("li select").forEach(s=>s.disabled=busy);
  $("patrol_delete").hidden=!patrolId;
  $("patrol_delete").disabled=busy;
  $("patrol_home").disabled=busy || !patrolMapReady || patrolSteps.some(s=>s.kind==="home");
  $("patrol_list").querySelectorAll("button").forEach(b=>b.disabled=busy || b.dataset.wrongMap==="1");
}
function patrolPointToMap(p){
  const m=patrolMap, [ox,oy]=m.origin;
  const ax=m.x_axis[0]-ox, ay=m.x_axis[1]-oy, bx=m.y_axis[0]-ox, by=m.y_axis[1]-oy;
  const det=ax*by-ay*bx;
  return {x:((p.x-ox)*by-(p.y-oy)*bx)/det*m.w, y:(ax*(p.y-oy)-ay*(p.x-ox))/det*m.h};
}
function patrolDraw(){
  if(!patrolMap) return;
  const m=patrolMap, svg=$("patrol_map");
  const pts=(patrolEditing?patrolSteps:patrolRun?.steps||[]).map((s,i)=>({...s,n:i+1})).filter(s=>s.kind==="goto").map(s=>({...patrolPointToMap(s),n:s.n}));
  const boxes=(latest?.rooms||[]).map(r=>r.bbox).filter(b=>b?.length===4);
  let x0=0,y0=0,x1=m.w,y1=m.h;
  if(boxes.length){
    x0=Math.min(...boxes.map(b=>b[0]/100*m.w));y0=Math.min(...boxes.map(b=>b[1]/100*m.h));
    x1=Math.max(...boxes.map(b=>(b[0]+b[2])/100*m.w));y1=Math.max(...boxes.map(b=>(b[1]+b[3])/100*m.h));
  }
  for(const p of pts){x0=Math.min(x0,p.x);y0=Math.min(y0,p.y);x1=Math.max(x1,p.x);y1=Math.max(y1,p.y);}
  const pad=Math.max(x1-x0,y1-y0)*.1;
  x0=Math.max(0,x0-pad);y0=Math.max(0,y0-pad);x1=Math.min(m.w,x1+pad);y1=Math.min(m.h,y1+pad);
  svg.setAttribute("viewBox",`${x0} ${y0} ${Math.max(1,x1-x0)} ${Math.max(1,y1-y0)}`);
  const size=Math.max(x1-x0,y1-y0)/28;
  svg.innerHTML=`<image href="${m.image}" width="${m.w}" height="${m.h}"/>`+
    pts.map(p=>`<circle cx="${p.x}" cy="${p.y}" r="${size/2}" fill="#11bdb0" stroke="#fff" stroke-width="${size/12}"/><text x="${p.x}" y="${p.y}" text-anchor="middle" dominant-baseline="central" fill="#082b2a" font-family="sans-serif" font-weight="700" font-size="${size*.6}">${p.n}</text>`).join("");
  svg.parentElement.style.background=m.bg;
}
function patrolStepLabel(s,i){
  if(s.kind==="goto") return `${i+1}. Aller au point ${i+1}`;
  if(s.kind==="home") return `${i+1}. Retour à la station`;
  if(s.kind==="turn") return `${i+1}. Tourner de ${Math.abs(s.degrees)}° à ${s.degrees<0?"gauche":"droite"}`;
  return `${i+1}. Observer pendant ${s.seconds} s`;
}
function patrolRenderSteps(){
  const list=$("patrol_steps");list.replaceChildren();
  patrolSteps.forEach((s,i)=>{
    const li=document.createElement("li"),label=document.createElement("span");label.className="patrolstepname";label.textContent=patrolStepLabel(s,i);li.append(label);
    if(s.kind==="home")li.classList.add("patrolfinalstep");
    if(s.kind==="turn"){
      const speed=document.createElement("select");speed.className="patrolspeed";
      speed.setAttribute("aria-label",`Vitesse de rotation de l’étape ${i+1}`);
      for(const [value,text] of [[.5,"Lente · ×0,5"],[1,"Normale · ×1"],[2,"Rapide · ×2"]]){
        const option=document.createElement("option");option.value=value;option.textContent=text;speed.append(option);
      }
      speed.value=String(s.speed??1);
      speed.onchange=()=>{if(!patrolBusy())s.speed=Number(speed.value);};label.append(speed);
    }
    for(const [text,title,delta] of [["↑","Monter cette étape",-1],["↓","Descendre cette étape",1],["×","Supprimer cette étape",0]]){
      if(s.kind==="home" && delta)continue;
      const b=document.createElement("button");b.className="btn sm";b.textContent=text;b.title=title;b.setAttribute("aria-label",title);
      const edge=delta && (i+delta<0 || i+delta>=patrolSteps.length || s.kind==="home" || patrolSteps[i+delta]?.kind==="home");b.dataset.edge=edge?"1":"0";
      b.onclick=()=>{if(patrolBusy() || edge)return;if(delta)[patrolSteps[i],patrolSteps[i+delta]]=[patrolSteps[i+delta],patrolSteps[i]];else patrolSteps.splice(i,1);patrolRenderSteps();patrolDraw();};li.append(b);
    }
    list.append(li);
  });
  patrolSync();
}
function patrolAdd(step){
  if(patrolBusy())return;
  if(patrolSteps.length>=40){toast("Maximum de 40 étapes.",true);return;}
  const home=patrolSteps.findIndex(s=>s.kind==="home");
  if(step.kind==="home" && home>=0)return;
  if(home>=0)patrolSteps.splice(home,0,step);else patrolSteps.push(step);
  patrolRenderSteps();patrolDraw();
}
function patrolRenderList(){
  const list=$("patrol_list");list.replaceChildren();
  if(!patrolRoutes.length){const p=document.createElement("p");p.className="patrolhint";p.textContent="Aucun parcours enregistré.";list.append(p);}
  for(const r of patrolRoutes){
    const row=document.createElement("div");row.className="patrolcard";
    const wrong=patrolMap && r.map_flag!==patrolMap.map_flag;
    const info=document.createElement("button");info.className="patrolrouteinfo";
    info.setAttribute("aria-label",`Modifier le parcours ${r.name}`);info.dataset.wrongMap=wrong?"1":"0";
    const name=document.createElement("strong");name.textContent=r.name;
    const detail=document.createElement("small");detail.textContent=`${r.steps.length} étape${r.steps.length===1?"":"s"} · ${wrong?"Autre carte":"Modifier"}`;
    info.append(name,detail);
    info.onclick=()=>{
      if(patrolBusy() || wrong)return;
      patrolId=r.id;patrolSteps=structuredClone(r.steps);$("patrol_name").value=r.name;patrolMode("edit");
      $("patrol_name").scrollIntoView({behavior:"smooth",block:"nearest"});
    };
    const play=document.createElement("button");play.className="btn sm patrolrouteplay";
    play.title=`Lancer ${r.name}`;play.setAttribute("aria-label",play.title);play.dataset.wrongMap=wrong?"1":"0";
    play.innerHTML='<svg viewBox="0 0 24 24" fill="currentColor" aria-hidden="true"><path d="m8 5 11 7-11 7z"/></svg>';
    play.onclick=()=>{if(!patrolBusy() && !wrong)patrolStart({id:r.id},r.steps);};
    row.append(info,play);list.append(row);
  }
  patrolSync();
}
async function patrolLoadRoutes(){
  const r=await api("/api/patrol/routes");patrolRoutes=r.routes||[];patrolRemote=r.status||{active:false};patrolRenderList();
  if(patrolRemote.active && !patrolRun){clearTimeout(patrolTimer);patrolTimer=setTimeout(patrolPoll,1000);}
}
async function patrolOpen(){
  if(!$("rc_nav").hidden){
    await patrolStop();$("rc_nav").hidden=true;$("rc_manual").hidden=false;$("rc_goto").classList.remove("on");$("rc_goto").setAttribute("aria-expanded","false");return;
  }
  if(chPlayRun)await chPlayStop();if(chRec)chRecStop();if(chPrev)chPreviewStop();
  await rcEnd();
  $("rc_nav").hidden=false;$("rc_manual").hidden=true;$("rc_goto").classList.add("on");$("rc_goto").setAttribute("aria-expanded","true");
  patrolMode("saved",true);
  patrolMapReady=false;patrolSync();patrolNote("Chargement de la carte…");
  try{
    const next=await api("/api/patrol/map");
    if(patrolMap && (patrolMap.map_flag!==next.map_flag || patrolMap.duid!==next.duid)){
      patrolSteps=[];patrolId=null;$("patrol_name").value="";
    }
    patrolMap=next;patrolMap.image="/api/map.png?t="+Date.now();
    patrolMapReady=true;await patrolLoadRoutes();patrolDraw();
    patrolNote(patrolRemote.active?"Un parcours est déjà en cours.":"");patrolSync();
  }catch(e){patrolNote(e.message,true);}
}
async function patrolStart(body,steps){
  if(patrolBusy() || !patrolMapReady)return;
  const epoch=++patrolEpoch;patrolPending=true;patrolSync();patrolNote("Démarrage…");
  let req=null;
  try{
    if(chPlayRun)await chPlayStop();await rcEnd();
    if(epoch!==patrolEpoch || document.hidden)return;
    req=post("/api/patrol/session",{action:"start",map_flag:patrolMap.map_flag,...body});patrolStartRequest=req;
    const res=await req;
    if(epoch!==patrolEpoch || document.hidden){await post("/api/patrol/session",{action:"stop",token:res.token});return;}
    patrolRun={token:res.token,steps:structuredClone(steps)};patrolRemote=res;patrolNote(res.message);patrolDraw();
    patrolTimer=setTimeout(patrolPoll,1000);
  }catch(e){if(epoch===patrolEpoch)patrolNote(e.message,true);}
  finally{if(patrolStartRequest===req)patrolStartRequest=null;patrolPending=false;patrolSync();}
}
async function patrolPoll(){
  const run=patrolRun;
  if(!run){
    if(!patrolRemote.active)return;
    try{patrolRemote=await api("/api/patrol/session");patrolNote(patrolRemote.message,patrolRemote.phase==="error");patrolSync();
      if(patrolRemote.active)patrolTimer=setTimeout(patrolPoll,2000);
    }catch(e){patrolNote(e.message,true);}
    return;
  }
  try{
    const st=await post("/api/patrol/session",{action:"heartbeat",token:run.token});
    if(patrolRun!==run)return;
    patrolRemote=st;patrolNote(st.message,st.phase==="error");
    if(st.active && (st.camera_revision||0)>(run.cameraRevision||0)){
      run.cameraRevision=st.camera_revision;
      if(camPc){
        camDetach();
        camNote("Reconnexion vidéo après le déplacement…");
        // Do not await WebRTC negotiation here: the route heartbeat must keep running.
        camStart().catch(e=>patrolNote(e.message,true));
      }
    }
    if(!st.active){
      patrolRun=null;patrolSync();
      if(st.end_session || st.phase!=="complete")await camStop();
      return;
    }
    patrolTimer=setTimeout(patrolPoll,2000);
  }catch(e){if(patrolRun===run){await patrolStop();await camStop();patrolNote("Connexion perdue. Parcours arrêté.",true);}}
}
async function patrolStop(){
  ++patrolEpoch;clearTimeout(patrolTimer);patrolTimer=null;
  let token=patrolRun?.token;const hadRemote=patrolRemote.active;
  const pending=patrolStartRequest;
  patrolRun=null;
  if(pending){try{token=(await pending).token;}catch(_){}}
  if(!token && !hadRemote){patrolSync();return;}
  try{patrolRemote=await post("/api/patrol/session",{action:"stop",...(token?{token}:{})});patrolNote(patrolRemote.message);}
  catch(e){patrolRemote={active:false};patrolNote("Arrêt non confirmé. La session expire sans connexion.",true);}
  patrolPending=false;patrolSync();
}
let patrolPointer=null;
$("patrol_map").addEventListener("pointerdown",e=>{patrolPointer={id:e.pointerId,x:e.clientX,y:e.clientY};});
$("patrol_map").addEventListener("pointercancel",()=>patrolPointer=null);
$("patrol_map").addEventListener("pointerup",e=>{
  const down=patrolPointer;patrolPointer=null;
  if(!down || down.id!==e.pointerId || Math.hypot(e.clientX-down.x,e.clientY-down.y)>8 || patrolBusy() || !patrolMapReady || patrolView==="saved")return;
  const svg=$("patrol_map"),matrix=svg.getScreenCTM();if(!matrix)return;
  const q=svg.createSVGPoint();q.x=e.clientX;q.y=e.clientY;const p=q.matrixTransform(matrix.inverse()),m=patrolMap;
  if(p.x<0||p.y<0||p.x>m.w||p.y>m.h)return;
  const x=Math.round(m.origin[0]+p.x/m.w*(m.x_axis[0]-m.origin[0])+p.y/m.h*(m.y_axis[0]-m.origin[0]));
  const y=Math.round(m.origin[1]+p.x/m.w*(m.x_axis[1]-m.origin[1])+p.y/m.h*(m.y_axis[1]-m.origin[1]));
  const step={kind:"goto",x,y};
  if(patrolEditing)patrolAdd(step);else patrolStart({steps:[step],end_session:false},[step]);
});
$("rc_goto").onclick=()=>patrolOpen().catch(e=>patrolNote(e.message,true));
$("patrol_saved").onclick=()=>patrolMode("saved");
$("patrol_direct").onclick=()=>patrolMode("direct");
$("patrol_edit").onclick=()=>$("patrol_new").click();
function patrolRotationDuration(){
  const seconds=Math.abs(Number($("patrol_angle").value))*Math.PI/180/(.6*Number($("patrol_speed").value));
  $("patrol_duration").textContent=`Rotation ≈ ${seconds.toLocaleString("fr-FR",{maximumFractionDigits:1})} s`;
}
$("patrol_angle").onchange=patrolRotationDuration;
$("patrol_speed").onchange=patrolRotationDuration;
patrolRotationDuration();
$("patrol_turn").onclick=()=>patrolAdd({kind:"turn",degrees:Number($("patrol_angle").value),speed:Number($("patrol_speed").value)});
$("patrol_wait").onclick=()=>patrolAdd({kind:"wait",seconds:Number($("patrol_seconds").value)});
$("patrol_home").onclick=()=>patrolAdd({kind:"home"});
$("patrol_direct_home").onclick=()=>patrolStart({steps:[{kind:"home"}],end_session:true},[{kind:"home"}]);
$("patrol_new").onclick=()=>{if(patrolBusy())return;patrolId=null;patrolSteps=[];$("patrol_name").value="";patrolMode("edit");};
$("patrol_save").onclick=async()=>{
  patrolEditorNote("");
  try{const r=await post("/api/patrol/routes",{id:patrolId,name:$("patrol_name").value,map_flag:patrolMap.map_flag,steps:patrolSteps});patrolId=r.route.id;await patrolLoadRoutes();patrolEditorNote("Parcours enregistré.");}
  catch(e){patrolEditorNote(e.message,true);}
};
$("patrol_play").onclick=()=>patrolStart({steps:patrolSteps,end_session:true},patrolSteps);
$("patrol_delete").onclick=async()=>{
  if(patrolBusy() || !patrolId)return;
  const id=patrolId,name=patrolRoutes.find(r=>r.id===id)?.name||$("patrol_name").value;
  if(!await ask(`Supprimer « ${name} » ?`))return;
  try{
    await post("/api/patrol/routes",{id,delete:true});
    patrolId=null;patrolSteps=[];$("patrol_name").value="";await patrolLoadRoutes();patrolRenderSteps();patrolDraw();patrolNote("Parcours supprimé.");
  }catch(e){patrolNote(e.message,true);}
};
$("patrol_stop").onclick=async()=>{stopClips();await patrolStop();await camStop();};
// Keep manual driving mutually exclusive with route navigation.
const patrolManualClick=$("rc_arm").onclick;
$("rc_arm").onclick=async()=>{
  if(patrolBusy())return;
  $("rc_nav").hidden=true;$("rc_manual").hidden=false;$("rc_goto").classList.remove("on");$("rc_goto").setAttribute("aria-expanded","false");
  await patrolManualClick();
};
window.addEventListener("pagehide",()=>{
  if(patrolRun?.token)navigator.sendBeacon("/api/patrol/session",new Blob([JSON.stringify({action:"stop",token:patrolRun.token})],{type:"application/json"}));
});
