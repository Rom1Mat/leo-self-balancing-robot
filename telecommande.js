// Ce fichier est integre dans le HTML embarque du sketch, sans serveur externe.
const $=id=>document.getElementById(id);
const joy=$('joystick'), knob=$('knob');
let socket=null, reconnectTimer=null, connectionSince=0, clientId=-1, ownerId=-1;
let robotStamp=null, robotClockAt=0, telemetryAt=-Infinity, feedbackAt=-Infinity, lastAckAt=0, sequence=0;
let pointerId=null, heldD=0, heldT=0, motorsEnabled=false, danceEnabled=false;
let lastDriveSent=0, lastHeartbeat=0, rtt=0, lastNoticeAt=0;
let pendingStop=false, bufferBusySince=null;
let controlSource=null, modePending=false;
const sentTimes=new Map();
const now=()=>performance.now();
function tell(message){$('notice').textContent=message;lastNoticeAt=now();}
function linkReady(){
  return socket && socket.readyState===WebSocket.OPEN && robotStamp!==null && now()-feedbackAt<400;
}
function currentRobotStamp(){
  if(robotStamp===null)return 0;
  return (robotStamp+Math.floor(Math.min(300,Math.max(0,now()-robotClockAt))))>>>0;
}
function socketSend(message){
  if(!socket || socket.readyState!==WebSocket.OPEN) return false;
  // Ne pas ajouter des consignes derriere des donnees qui attendent deja.
  if(socket.bufferedAmount>0) return false;
  socket.send(message); return true;
}
function renderStick(x=0,y=0){
  knob.style.transform=`translate(-50%,-50%) translate(${x}px,${y}px)`;
  $('joystickRead').textContent=`Avance ${Math.round(heldD*100)} % · rotation ${Math.round(heldT*100)} %`;
  joy.classList.toggle('active',pointerId!==null);
}
function resetStick(){
  const previous=pointerId;
  pointerId=null; heldD=0; heldT=0;
  if(previous!==null && joy.hasPointerCapture && joy.hasPointerCapture(previous)){
    try{joy.releasePointerCapture(previous);}catch(e){}
  }
  renderStick();
}
function sendDrive(held){
  if(!socket || socket.readyState!==WebSocket.OPEN) return false;
  if(held && (!linkReady() || controlSource!=='web' || modePending)) return false;
  const seq=(++sequence)>>>0;
  const message=`D,${seq},${currentRobotStamp()},${Math.round(heldD*1000)},${Math.round(heldT*1000)},${held?1:0}`;
  if(!socketSend(message)) return false;
  sentTimes.set(seq,now());
  if(sentTimes.size>64) sentTimes.delete(sentTimes.keys().next().value);
  lastDriveSent=now(); return true;
}
function stopDrive(){
  const wasHeld=pointerId!==null;
  resetStick();
  if(wasHeld){pendingStop=true;flushStop();}
}
function flushStop(){
  if(pendingStop && sendDrive(false))pendingStop=false;
}
function dropConnection(message){
  controlSource=null;
  resetStick(); sentTimes.clear(); robotStamp=null; telemetryAt=-Infinity;feedbackAt=-Infinity;
  pendingStop=false;bufferBusySince=null;
  const old=socket; socket=null;
  if(old){try{old.close();}catch(e){}}
  if(message) tell(message);
  updateConnection();
  if(reconnectTimer===null) reconnectTimer=setTimeout(()=>{reconnectTimer=null;connectSocket();},1000);
}
function connectSocket(){
  if(document.hidden || socket) return;
  clientId=-1; ownerId=-1; sequence=0; robotStamp=null;rtt=0;pendingStop=false;bufferBusySince=null;controlSource=null;resetStick();
  connectionSince=now();
  const ws=new WebSocket(`ws://${location.hostname}:81/`);
  socket=ws;
  ws.onopen=()=>{if(socket!==ws)return;connectionSince=now();resetStick();updateConnection();};
  ws.onmessage=event=>{if(socket===ws) receivePacket(String(event.data));};
  ws.onerror=()=>{if(socket===ws)dropConnection('Connexion WebSocket interrompue.');};
  ws.onclose=()=>{if(socket===ws)dropConnection('Connexion perdue : relache le joystick.');};
  updateConnection();
}
function updateConnection(){
  const ready=linkReady();
  const web=controlSource==='web' && !modePending;
  $('connection').textContent=ready?
    `Connecte · WebSocket${rtt?' · '+Math.round(rtt)+' ms':''}${ownerId>=0 && ownerId!==clientId?' · autre ecran aux commandes':''}`:
    'Connexion au robot…';
  $('connection').classList.toggle('online',!!ready);
  joy.classList.toggle('unavailable',!ready || !motorsEnabled || !web);
  joy.setAttribute('aria-disabled',String(!ready || !motorsEnabled || !web));
  $('btn').disabled=!motorsEnabled && (!ready || !web);
  $('modeWeb').disabled=!ready || modePending;
  $('modeXbox').disabled=!ready || modePending;
  $('modeWeb').setAttribute('aria-pressed',String(controlSource==='web'));
  $('modeXbox').setAttribute('aria-pressed',String(controlSource==='xbox'));
  $('modeWeb').className=controlSource==='web'?'go':'';
  $('modeXbox').className=controlSource==='xbox'?'go':'';
  $('sourceRead').textContent=controlSource===null?'Lecture de la source active…':
    controlSource==='web'?'Pilotage Web actif · commandes Xbox bloquees':'Pilotage Xbox actif · joystick et gestes web bloques';
  for(const button of document.querySelectorAll('[onclick^="sendGesture"],#danceBtn'))button.disabled=!ready || !web;
  $('danceBtn').disabled=!ready || !web || !motorsEnabled || danceEnabled;
  $('sa').disabled=$('sb').disabled=!ready || !web;
}
async function chooseControlSource(source){
  if(modePending || !linkReady())return;
  stopDrive();modePending=true;updateConnection();
  const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),4000);
  try{
    const response=await fetch('/control-mode?mode='+encodeURIComponent(source),{method:'POST',signal:controller.signal});
    if(!response.ok)throw Error('Source non changee');
    tell('Source changee : relache les commandes avant de reprendre.');
  }catch(error){tell('Changement de source non confirme.');}
  finally{clearTimeout(timeout);modePending=false;updateConnection();}
}
function receivePacket(text){
  const fields=text.split(','), kind=fields.shift();
  if(kind==='W'){clientId=Number(fields[0]);return;}
  if(kind==='Y'){
    if(fields.length===1 && ['0','1'].includes(fields[0]))
      $('dynamicRead').textContent='Balancier : '+(fields[0]==='1'?'active':'desactive')+' · bouton Y';
    return;
  }
  if(kind==='C'){
    if(fields.length!==5 || !['0','1'].includes(fields[0]))return;
    const selected=fields[0]==='1'?'web':'xbox';
    if(selected!==controlSource){
      resetStick();controlSource=selected;
      // Neutre explicite pour re-armer le serveur ; jamais de reprise d'un appui ancien.
      pendingStop=selected==='web';flushStop();
    }
    const connected=fields[1]==='1',age=Number(fields[2]);
    $('xboxRead').textContent=!connected?'Xbox deconnectee':
      `Xbox connectee · age du rapport ${Number.isFinite(age)?age:'—'} ms · `+
      (controlSource==='web'?'commandes desactivees':fields[4]==='1'?'aux commandes':fields[3]==='1'?'prete':'recentre le stick gauche');
    updateConnection();return;
  }
  if(kind==='R'){
    if(fields.length!==8)return;
    const reasons=['prete au neutre','mode Web','deconnectee','attente du premier rapport','recentre le stick gauche','moteurs arretes','autre commande active','commande appliquee'];
    $('xboxDebug').textContent=`Rapports ${fields[0]} · notifications ${fields[1]} · lectures HID ${fields[2]} · rejetes ${fields[3]} · LT ${Math.round(Number(fields[4])/1023*100)} % / RT ${Math.round(Number(fields[5])/1023*100)} % · consigne ${Math.round(Number(fields[6])/10)} % · ${reasons[Number(fields[7])]||'etat inconnu'}`;
    return;
  }
  if(kind==='A'){
    const seq=Number(fields[0]), sent=sentTimes.get(seq);
    if(sent!==undefined){
      rtt=now()-sent;sentTimes.delete(seq);lastAckAt=now();feedbackAt=now();
      const stamp=Number(fields[1]);
      if(Number.isInteger(stamp) && stamp>=0 && stamp<=0xffffffff){robotStamp=stamp;robotClockAt=now();}
    }
    return;
  }
  if(kind==='B' || kind==='E'){
    const wasHeld=pointerId!==null;
    resetStick();if(wasHeld){pendingStop=true;flushStop();}
    tell(fields.join(','));return;
  }
  if(kind!=='S' || fields.length<27) return;
  const stamp=Number(fields.shift()), owner=Number(fields.shift());
  if(!Number.isInteger(stamp) || !Number.isInteger(owner)) return;
  robotStamp=stamp;robotClockAt=now();ownerId=owner;telemetryAt=now();feedbackAt=now();
  renderTelemetry(fields); updateConnection();
}
// Petite zone neutre, puis progression douce pour doser les faibles vitesses.
function joystickAxes(dx,dy,radius){
  let x=dx/radius,y=dy/radius;
  const length=Math.hypot(x,y);
  if(length>1){x/=length;y/=length;}
  const magnitude=Math.min(length,1), dead=0.08;
  if(magnitude<=dead) return {d:0,t:0,x:x*radius,y:y*radius};
  const scale=((magnitude-dead)/(1-dead))/magnitude;
  const curve=value=>Math.sign(value)*Math.pow(Math.abs(value),1.35);
  return {d:curve(-y*scale),t:curve(x*scale),x:x*radius,y:y*radius};
}
function movePointer(clientX,clientY){
  const rect=joy.getBoundingClientRect(), radius=Math.min(rect.width,rect.height)*0.32;
  const axes=joystickAxes(clientX-rect.left-rect.width/2,clientY-rect.top-rect.height/2,radius);
  heldD=axes.d;heldT=axes.t;renderStick(axes.x,axes.y);
}
function beginPointer(id,x,y){
  if(pointerId!==null) return false;
  if(controlSource!=='web' || modePending){tell('Selectionne « Piloter avec le site » pour utiliser le joystick.');return false;}
  if(pendingStop){tell('Freinage en cours de transmission : reprends le joystick.');return false;}
  if(!linkReady()){tell('Attends la connexion au robot.');return false;}
  if(!motorsEnabled){tell('Demarre les moteurs avant de piloter.');return false;}
  pointerId=id;lastAckAt=now();movePointer(x,y);
  // Si le socket est momentanement occupe, le prochain tick enverra la position courante.
  sendDrive(true);
  return true;
}
// Safari : supprimer les gestes du navigateur uniquement sur la zone de pilotage.
for(const name of ['contextmenu','selectstart','dragstart']) joy.addEventListener(name,e=>e.preventDefault());
joy.addEventListener('touchstart',e=>e.preventDefault(),{passive:false});
joy.addEventListener('touchmove',e=>e.preventDefault(),{passive:false});
if('PointerEvent' in window){
  joy.addEventListener('pointerdown',e=>{
    e.preventDefault();
    if(e.button!==0 || e.isPrimary===false) return;
    if(beginPointer(e.pointerId,e.clientX,e.clientY)){
      try{joy.setPointerCapture(e.pointerId);}catch(error){stopDrive();}
    }
  });
  joy.addEventListener('pointermove',e=>{
    if(e.pointerId!==pointerId)return;e.preventDefault();movePointer(e.clientX,e.clientY);
  });
  const release=e=>{if(e.pointerId===pointerId){e.preventDefault();stopDrive();}};
  joy.addEventListener('pointerup',release);
  joy.addEventListener('pointercancel',e=>{if(e.pointerId===pointerId)tell('Appui annule par le navigateur : reprends le joystick.');release(e);});
  joy.addEventListener('lostpointercapture',e=>{if(e.pointerId===pointerId)tell('Appui tactile interrompu : reprends le joystick.');release(e);});
  window.addEventListener('pointerup',release);
  window.addEventListener('pointercancel',release);
}else{
  // Repli pour les anciens Safari sans Pointer Events.
  joy.addEventListener('touchstart',e=>{
    e.preventDefault();const t=e.changedTouches[0];beginPointer(t.identifier,t.clientX,t.clientY);
  },{passive:false});
  joy.addEventListener('touchmove',e=>{
    e.preventDefault();for(const t of e.changedTouches)if(t.identifier===pointerId)movePointer(t.clientX,t.clientY);
  },{passive:false});
  const release=e=>{for(const t of e.changedTouches)if(t.identifier===pointerId){e.preventDefault();stopDrive();}};
  window.addEventListener('touchend',release,{passive:false});
  window.addEventListener('touchcancel',release,{passive:false});
}
function brakeRobot(){
  resetStick();pendingStop=false;
  if(!socketSend('Z')){
    dropConnection('Freinage demande.');
    fetch('/dance/stop').catch(()=>tell('Connexion perdue.'));
  }
}
function cutMotors(){
  resetStick();pendingStop=false;
  if(!socketSend('X')){
    dropConnection('Arret moteurs demande.');
    fetch('/reset').catch(()=>tell('Connexion perdue.'));
  }
}
function sendGesture(value){
  if(controlSource!=='web' || modePending){tell('Selectionne le pilotage Web pour les bras ou la danse.');return;}
  stopDrive();
  if(pendingStop){tell('Freinage en cours : reessaie le geste.');return;}
  if(!linkReady() || !socketSend(`G,${currentRobotStamp()},${value}`)) tell('Connexion indisponible : geste non lance.');
}
function launchDance(){sendGesture(4);}
$('btn').onclick=()=>{
  if(motorsEnabled){cutMotors();return;}
  if(controlSource!=='web' || modePending)return;
  if(!linkReady() || !socketSend(`M,${currentRobotStamp()},1`)) tell('Attends la connexion au robot.');
};
window.addEventListener('blur',()=>{stopDrive();});
window.addEventListener('pagehide',()=>{stopDrive();dropConnection();});
document.addEventListener('visibilitychange',()=>{
  if(document.hidden){stopDrive();dropConnection();}
  else{if(reconnectTimer!==null){clearTimeout(reconnectTimer);reconnectTimer=null;}connectSocket();}
});
// 25 Hz pour le pilotage, 10 Hz pour la presence de la telecommande.
setInterval(()=>{
  if(!socket){if(!document.hidden && reconnectTimer===null)connectSocket();return;}
  const stamp=now();
  if(socket.readyState===WebSocket.CONNECTING && stamp-connectionSince>3000){dropConnection('Reconnexion au robot…');return;}
  if(socket.readyState===WebSocket.OPEN && stamp-(robotStamp===null?connectionSince:feedbackAt)>450){
    dropConnection('Robot muet : deplacement arrete.');return;
  }
  // Tolere un tampon transitoire ; une congestion persistante impose toujours un arret.
  if(socket.readyState===WebSocket.OPEN && socket.bufferedAmount>0){
    if(bufferBusySince===null)bufferBusySince=stamp;
    if(stamp-bufferBusySince>200){dropConnection('Envoi bloque : deplacement arrete.');return;}
  }else bufferBusySince=null;
  flushStop();
  if(!linkReady())return;
  if(pointerId!==null){
    if(stamp-lastAckAt>350){dropConnection('Commandes non confirmees : deplacement arrete.');return;}
    if(stamp-lastDriveSent>=40)sendDrive(true);
  }
  if(!pendingStop && stamp-lastHeartbeat>=(controlSource==='xbox'?500:100) && socketSend(`H,${currentRobotStamp()}`))lastHeartbeat=stamp;
  if(lastNoticeAt && stamp-lastNoticeAt>5000){$('notice').textContent='';lastNoticeAt=0;}
  updateConnection();
},20);
function bind(id,vid,param,dec){
  const slider=$(id), label=$(vid);let timer=null;
  slider.oninput=()=>{
    label.textContent=dec?Number(slider.value).toFixed(dec):slider.value;
    clearTimeout(timer);
    timer=setTimeout(()=>fetch('/set?'+param+'='+encodeURIComponent(slider.value)).catch(()=>tell('Reglage non transmis.')),100);
  };
}
bind('kp','vkp','kp',0);bind('kd','vkd','kd',0);bind('ma','vma','maxaccel',0);
bind('sk','vsk','spkp',2);bind('ki','vki','ki',2);
bind('tg','vtg','target',1);bind('ms','vms','maxspeed',0);bind('gl','vgl','gyrolp',2);
bind('ds','vds','drivespeed',0);bind('tr','vtr','turnspeed',0);
bind('sa','vsa','servo1',0);bind('sb','vsb','servo2',0);
$('cap').onclick=()=>fetch('/capture').then(r=>r.text()).then(v=>{$('tg').value=v;$('vtg').textContent=Number(v).toFixed(1);}).catch(()=>tell('Capture non transmise.'));
const canvas=$('chart'),cx=canvas.getContext('2d'),buf=new Array(120).fill(0);
function draw(){
  const w=canvas.width,h=canvas.height,mid=h/2,scale=mid/45;
  cx.clearRect(0,0,w,h);cx.strokeStyle='#d7e2ef';cx.beginPath();cx.moveTo(0,mid);cx.lineTo(w,mid);cx.stroke();
  cx.strokeStyle='#287bc4';cx.lineWidth=2;cx.beginPath();
  buf.forEach((value,i)=>{const x=i/(buf.length-1)*w,y=mid-value*scale;i?cx.lineTo(x,y):cx.moveTo(x,y);});cx.stroke();
}
function renderTelemetry(p){
  motorsEnabled=p[2]==='1';danceEnabled=p[21]==='1';
  $('ang').textContent=Number(p[0]).toFixed(1);$('out').textContent=Math.round(p[1]);
  $('btn').textContent=motorsEnabled?'Couper moteurs':'Demarrer moteurs';$('btn').className=motorsEnabled?'stop':'go';
  $('danceBtn').disabled=danceEnabled || !motorsEnabled || !linkReady() || controlSource!=='web' || modePending;
  $('danceRead').textContent=(danceEnabled?'Danse ':'Danse arretee · ')+(Number(p[22])/1000).toFixed(1)+' / 32 s · rayon estime '+Number(p[23]).toFixed(2)+' m'+(p[24]==='1'?' · limite atteinte : bras seulement':'');
  $('driveRead').textContent='Consigne '+Math.round(p[14])+' pas/s · rotation '+Math.round(p[15])+' pas/s · correction angle '+Number(p[16]).toFixed(2)+'° dont I '+Number(p[18]).toFixed(2)+'°';
  $('controlRead').textContent='Acceleration '+Math.round(p[10])+' pas/s² · calcul '+p[12]+' µs · '+(p[13]==='1'?'limite atteinte':'sans saturation');
  $('loop').textContent='Boucle '+Math.round(1e6/Number(p[3]))+' Hz · intervalle max '+p[4]+' µs';
  $('dbg').textContent='Erreur '+Number(p[5]).toFixed(2)+'° · gyro '+Number(p[11]).toFixed(1)+'°/s · age commande '+p[20]+' ms';
  if(!motorsEnabled && pointerId!==null)stopDrive();
  buf.push(Number(p[0]));buf.shift();draw();
}
renderStick();connectSocket();
