// Execute les fonctions de decision et le decodeur reels, traduits en JavaScript.
// Ne simule ni le transport radio ni la dynamique physique du robot.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const firmware=fs.readFileSync('self_balancing_robot/self_balancing_robot.ino','utf8');
const header=fs.readFileSync('self_balancing_robot/XboxBle.h','utf8');
function extract(source,name){
  const start=source.indexOf(name+'(');assert(start>=0);
  const open=source.indexOf('{',start);let end=open+1,depth=1;
  while(depth){if(source[end]==='{')depth++;if(source[end]==='}')depth--;end++;}
  return source.slice(start,end);
}
function translate(source,name,args){
  let text=extract(source,name);
  text='function '+name+'('+args+')'+text.slice(text.indexOf('{'));
  return text.replace(/\(uint32_t\)\(now - input.receivedMs\)/g,'((now - input.receivedMs) >>> 0)')
    .replace(/\(uint32_t\)\(now - lastTick\)/g,'((now - lastTick) >>> 0)')
    .replace(/\(uint32_t\)\(now - requestAt\)/g,'((now - requestAt) >>> 0)')
    .replace(/\(int32_t\)\(input.neutralSerial - xboxNeutralFence\)/g,'((input.neutralSerial - xboxNeutralFence) | 0)')
    .replace(/XboxBle::Input|\buint32_t\b|\buint16_t\b|\buint8_t\b|\bfloat\b|\bbool\b/g,m=>m==='float'?'FLOAT':m)
    .replace(/\b(?:uint16_t|FLOAT)\(/g,'Number(')
    .replace(/\b(?:XboxBle::Input|uint32_t|uint16_t|uint8_t|FLOAT|bool|int)\s+(?=\w+\s*=)/g,'let ')
    .replace(/\(int\)\(/g,'Math.trunc(')
    .replace(/XboxBle::/g,'XboxBle.')
    .replace(/(\d+(?:\.\d*)?)f\b/g,'$1');
}
const atomic=value=>({value,store(v){this.value=v;},load(){return this.value;}});
const ctx=vm.createContext({
  constrain:(v,l,h)=>Math.max(l,Math.min(h,v)),fabsf:Math.abs,powf:Math.pow,fminf:Math.min,sinf:Math.sin,DYNAMIC_ARM_DIRECTION:-1,Serial:{printf(){}},
  dynamicArmsEnabled:false,dynamicArmsReturning:false,dynamicDrive:0,dynamicPhase:0,dynamicLeftVelocity:0,dynamicRightVelocity:0,
  clock:1000,MANUAL_TIMEOUT_MS:350,webControlEnabled:false,WEBSOCKETS_SERVER_CLIENT_MAX:4,events:[],wsNeedsRelease:[],manualOwner:-1,
  xboxOwnsControl:false,xboxReady:false,xboxArmsReady:false,xboxBaseline:false,xboxLastSerial:0,xboxSession:0,xboxNeutralFence:0,
  xboxPreviousButtons:0,xboxArmLeft:0,xboxArmRight:0,
  armPos1:90,armPos2:90,armGesture:-1,danceRunning:false,danceReturningArms:false,
  lastTick:0,leftPosition:90,rightPosition:90,SERVO1_PIN:15,SERVO2_PIN:16,
  servoWrite(pin,pos){ctx.events.push({pin,pos});},driveCmd:atomic(0),turnCmd:atomic(0),
  lastDriveMs:atomic(0),motorsOn:atomic(false),motorRequest:atomic(-1),danceActive:atomic(false),
  cancelDance(){ctx.events.push('cancel');ctx.danceActive.store(false);},
  selectGesture(g){ctx.events.push('gesture:'+g);},
  millis(){return ctx.clock;}, XboxBle:{snapshot:()=>ctx.input}
});
vm.runInContext(translate(header,'axis','raw')+'\n'+translate(header,'turn','raw')+'\n'+translate(header,'trigger','raw')+'\n'+translate(header,'decode','bytes,size,out')+
  '\n'+translate(firmware,'stopRemoteMotion','')+'\n'+translate(firmware,'serviceXbox',''),ctx);
ctx.XboxBle.axis=ctx.axis;ctx.XboxBle.turn=ctx.turn;ctx.XboxBle.trigger=ctx.trigger;
vm.runInContext(translate(firmware,'setControlSource','web').replace('int i = 0','let i = 0'),ctx);
for(const [name,args] of [['dynamicArmTarget','command,filtered,phase,side'],['dynamicArmVelocity','position,target,velocity,dt'],['dynamicArmPosition','position,target,velocity,dt']])
  vm.runInContext(translate(firmware,name,args),ctx);
let armCode=translate(firmware,'updateXboxArms','');
armCode=armCode.replace(/static let lastTick = 0;/,'').replace(/static let leftPosition = 90(?:\.0)?, rightPosition = 90(?:\.0)?;/,'');
vm.runInContext(armCode,ctx);
vm.runInContext(translate(header,'advertisementMatches','xboxName,microsoft,gamepad,savedAddress'),ctx);
assert(ctx.advertisementMatches(true,false,false,false),'Nom Xbox suffisant meme sans UUID HID annonce');
assert(ctx.advertisementMatches(false,true,true,false),'Microsoft + apparence manette');
assert(ctx.advertisementMatches(false,false,false,true),'Adresse deja memorisee');
assert(!ctx.advertisementMatches(false,true,false,false),'Un appareil Microsoft quelconque ne suffit pas');
assert(!ctx.advertisementMatches(false,false,true,false),'Un appareil HID quelconque ne suffit pas');
assert(header.includes('scan->start(3000, false)'));
assert(header.includes('(uint32_t)(millis() - scanStart) < 4500'));
assert(header.includes('scan termine, %d appareil(s) BLE detecte(s)'));
assert(header.includes('client->connect(NimBLEAddress(std::string(saved.c_str()), savedType), true)'));
assert(header.includes('prefs.putUChar("type", savedType)'));
assert(header.includes('NimBLEDevice::getBondedAddress(0)'));
assert(header.includes('BLE_HCI_SCAN_FILT_NO_WL_INITA'));
assert(header.includes('getConnInfo().getIdAddress()'));
assert(!header.includes('deleteBond('));
function packet({x=32768,y=32768,rightY=32768,lt=0,rt=0,buttons=0,hat=0,connected=true,session=1}={}){
  const previous=ctx.input,serial=(previous?.serial||0)+1;
  const sameSession=previous?.session===session && previous?.connected && connected;
  const neutral=ctx.axis(y)===0 && ctx.turn(x)===0;
  ctx.input={x,y,rightY,lt,rt,buttons,hat,connected,valid:true,session,receivedMs:ctx.clock,serial,
    neutralSeen:neutral || (sameSession && previous.neutralSeen),
    neutralSerial:neutral?serial:sameSession?previous.neutralSerial:0};
  ctx.serviceXbox();ctx.updateXboxArms();ctx.clock+=20;
}
// Format BLE, signes et zone neutre.
for(const v of [0,65535])assert(Math.abs(ctx.axis(v))===1);
assert.equal(ctx.axis(32768),0);assert.equal(ctx.axis(32767),0);
assert(ctx.axis(40000)>0 && ctx.axis(40000)<.1);
const bytes=new Uint8Array(16);bytes.set([0,128,0,128,0,128,0,128]);
let out={};assert(ctx.decode(bytes,16,out));assert.equal(out.x,32768);assert.equal(out.y,32768);
bytes[13]=0x4b;bytes[12]=3;assert(ctx.decode(bytes,16,out));assert.equal(out.buttons,0x4b);assert.equal(out.hat,3);
assert(!ctx.decode(bytes,15,{}));assert(!ctx.decode(bytes,17,{}));
bytes[12]=9;assert(!ctx.decode(bytes,16,{}));bytes[12]=0;bytes[9]=4;assert(!ctx.decode(bytes,16,{}));
// A demarre uniquement au neutre et sur un nouvel appui.
packet({buttons:1});assert.equal(ctx.motorRequest.value,-1);
packet();packet({buttons:1,y:0});assert.equal(ctx.motorRequest.value,-1);
packet();packet({buttons:1});assert.equal(ctx.motorRequest.value,1);
ctx.motorsOn.store(true);packet();packet({y:0,x:65535});
assert(ctx.xboxOwnsControl);assert.equal(ctx.driveCmd.value,1);assert.equal(ctx.turnCmd.value,1);
packet({y:65535});assert.equal(ctx.driveCmd.value,-1);assert.equal(ctx.turnCmd.value,0);
packet({lt:1023,rt:1023});assert.equal(ctx.driveCmd.value+0,0);
packet({x:0});assert.equal(ctx.turnCmd.value,-1);assert.equal(ctx.driveCmd.value+0,0);
packet({y:0});assert.equal(ctx.driveCmd.value,1);
packet({y:65535});assert.equal(ctx.driveCmd.value,-1);
packet({rt:1023,lt:1023});assert.equal(ctx.driveCmd.value+0,0,'Les gachettes ne pilotent plus la translation');
packet({y:30000});assert.equal(ctx.driveCmd.value+0,0,'Zone neutre verticale');
packet({y:16000});assert(ctx.driveCmd.value>0 && ctx.driveCmd.value<1,'Translation progressive');
packet();assert(!ctx.xboxOwnsControl);assert.equal(ctx.driveCmd.value+0,0);assert.equal(ctx.turnCmd.value,0);assert(ctx.motorsOn.value);
packet({y:0});const received=ctx.lastDriveMs.value;
ctx.clock+=100;ctx.serviceXbox();assert.equal(ctx.lastDriveMs.value,ctx.clock);assert.equal(ctx.input.receivedMs,received);
ctx.clock+=30000;ctx.serviceXbox();assert(ctx.xboxOwnsControl);assert.equal(ctx.driveCmd.value,1);
packet({connected:false});assert(!ctx.xboxOwnsControl);assert.equal(ctx.driveCmd.value+0,0);
packet({y:0});packet({y:0});assert(!ctx.xboxOwnsControl);
packet();packet({y:0});assert(ctx.xboxOwnsControl);
ctx.stopRemoteMotion();packet({y:0});assert(!ctx.xboxOwnsControl);
packet();packet({y:0});assert(ctx.xboxOwnsControl);
packet({buttons:2,y:0});assert(!ctx.xboxOwnsControl);assert.equal(ctx.motorRequest.value,0);
ctx.motorsOn.store(false);packet();packet({y:0});assert(!ctx.xboxOwnsControl);
assert.equal(ctx.motorRequest.value,0,'Le stick ne demarre jamais les moteurs');
packet();ctx.motorRequest.store(-1);packet({buttons:8});assert.equal(ctx.motorRequest.value,-1,'X ne coupe plus les moteurs');
// Les bras sont independants et accessibles moteurs coupes.
packet();const left0=ctx.armPos1,right0=ctx.armPos2;
for(let i=0;i<20;i++)packet({hat:1});
assert(ctx.armPos1<left0);assert.equal(ctx.armPos2,right0);
const left1=ctx.armPos1;
for(let i=0;i<20;i++)packet({rightY:0});
assert.equal(ctx.armPos1,left1);assert(ctx.armPos2<right0);
for(let i=0;i<150;i++)packet({hat:1,rightY:0});
assert.equal(ctx.armPos1,0);assert.equal(ctx.armPos2,0);
for(let i=0;i<150;i++)packet({hat:5,rightY:65535});
assert.equal(ctx.armPos1,180);assert.equal(ctx.armPos2,180);
packet();for(let i=0;i<10;i++)packet();assert.equal(ctx.armPos1,180);assert.equal(ctx.armPos2,180);
ctx.armPos2=130;packet();packet({rightY:65535});assert(ctx.armPos2>130 && ctx.armPos2<140,'Repart de la pose imposee par une autre commande');
// Pilotage simultane des deux bras et des roues, sans LB.
packet();ctx.motorsOn.store(true);packet({y:0,hat:1,rightY:0});
assert(ctx.xboxOwnsControl);assert(ctx.xboxArmLeft>0 && ctx.xboxArmRight>0);
packet({hat:1,rightY:0});assert(!ctx.xboxOwnsControl);assert.equal(ctx.driveCmd.value+0,0);assert(ctx.xboxArmLeft>0);
ctx.clock+=1000;ctx.updateXboxArms();assert(ctx.xboxArmLeft>0);
ctx.input.connected=false;ctx.serviceXbox();ctx.updateXboxArms();assert.equal(ctx.xboxArmLeft,0);assert.equal(ctx.xboxArmRight,0);
ctx.serviceXbox();packet();
ctx.manualOwner=2;ctx.driveCmd.store(.3);packet({y:0});
assert(!ctx.xboxOwnsControl);assert.equal(ctx.driveCmd.value,.3);
ctx.manualOwner=-1;packet({y:0});assert(!ctx.xboxOwnsControl);
packet();packet({y:0});assert(ctx.xboxOwnsControl);
packet({connected:false});assert(!ctx.xboxOwnsControl);assert.equal(ctx.driveCmd.value+0,0);
packet({session:2,y:0});packet({session:2,y:0});assert(!ctx.xboxOwnsControl);
packet({session:2});packet({session:2,y:0});assert(ctx.xboxOwnsControl);
ctx.input.receivedMs=0xfffffff0;ctx.clock=30;ctx.serviceXbox();assert(ctx.xboxOwnsControl);
// A valide deja le neutre : premier mouvement du stick avant que balanceStep ne consomme A.
ctx.clock=10000;ctx.motorsOn.store(false);ctx.stopRemoteMotion();packet({session:3});
packet({session:3,buttons:1});assert(ctx.xboxReady);assert.equal(ctx.motorRequest.value,1);
packet({session:3,y:10368});assert(ctx.xboxReady);
ctx.motorsOn.store(true);ctx.motorRequest.store(-1);ctx.serviceXbox();
assert(ctx.xboxOwnsControl,'Le meme rapport frais est applique des que les moteurs demarrent');
assert(ctx.driveCmd.value>0);
// Un bras tenu vers le haut ne doit pas bloquer le rearmement des commandes du stick.
ctx.stopRemoteMotion();packet({session:3,hat:1});assert(ctx.xboxReady);
packet({session:3,hat:1,y:13568});assert(ctx.xboxOwnsControl);assert(ctx.driveCmd.value>0);
// Les autres boutons sans action ne verrouillent plus les commandes du stick.
ctx.stopRemoteMotion();packet({session:3,buttons:0x40});assert(ctx.xboxReady);
packet({session:3,buttons:0x40,y:13568});assert(ctx.xboxOwnsControl);
// Silence BLE au repos : le premier nouvel appui doit etre traite immediatement.
packet({session:3});ctx.clock+=2000;ctx.serviceXbox();
assert(ctx.xboxReady && ctx.xboxBaseline);
packet({session:3,y:10368});assert(ctx.xboxOwnsControl);assert(ctx.driveCmd.value>0);
// Une deconnexion PENDANT le mouvement impose le recentrage.
ctx.input.connected=false;ctx.serviceXbox();assert(!ctx.xboxReady);assert.equal(ctx.driveCmd.value+0,0);
packet({session:3,y:10368});assert(!ctx.xboxOwnsControl);
packet({session:3});packet({session:3,y:10368});assert(ctx.xboxOwnsControl);
// Le premier appui sur un bras apres un silence au repos reste lui aussi utilisable.
packet({session:3});ctx.clock+=1000;ctx.serviceXbox();
assert(ctx.xboxArmsReady);const idleArmPosition=ctx.armPos1;
packet({session:3,hat:5});assert(ctx.xboxArmLeft<0);
// Exclusivite de source, changement en mouvement et absence de reprise automatique.
packet({session:3});packet({session:3,y:7168});assert(ctx.xboxOwnsControl);
ctx.setControlSource(true);assert(ctx.webControlEnabled);assert.equal(ctx.driveCmd.value+0,0);
assert(!ctx.xboxOwnsControl && ctx.wsNeedsRelease.every(Boolean));
packet({session:3});packet({session:3,buttons:1});assert(!ctx.xboxOwnsControl && !ctx.xboxReady);
packet({session:3,y:3968,hat:1,rightY:0});assert.equal(ctx.driveCmd.value+0,0);assert.equal(ctx.xboxArmLeft,0);
ctx.motorRequest.store(-1);packet({session:3,buttons:2});assert.equal(ctx.motorRequest.value,0,'B reste une coupure dans les deux modes');
ctx.setControlSource(false);packet({session:3,y:3968});assert(!ctx.xboxOwnsControl);
packet({session:3});packet({session:3,y:3968});assert(ctx.xboxOwnsControl);
assert.equal(ctx.turn(32768),0);assert.equal(ctx.turn(0),-1);assert.equal(ctx.turn(65535),1);
assert(ctx.turn(45000)>ctx.axis(45000),'La rotation est plus franche a mi-course');
assert(firmware.includes('if(!webControlEnabled){ webSocket.sendTXT(client, "B,Pilotage Web desactive : mode Xbox")'));
assert(firmware.includes('server.on("/control-mode", HTTP_POST, handleControlMode)'));
// Normalisation proportionnelle des commandes du stick et champs du rapport HID.
assert.equal(ctx.trigger(0),0);assert.equal(ctx.trigger(50),0);assert.equal(ctx.trigger(1023),1);
assert(ctx.trigger(512)>.4 && ctx.trigger(512)<.6);
bytes[9]=0;bytes[8]=123;bytes[10]=255;bytes[11]=3;bytes[6]=0;bytes[7]=0;
assert(ctx.decode(bytes,16,out));assert.equal(out.lt,123);assert.equal(out.rt,1023);assert.equal(out.rightY,0);
assert(firmware.includes('lastDriveMs.store(now); // Etat HID maintenu'));
assert(header.includes('void onDisconnect(NimBLEClient*, int reason) override'));
assert(header.includes('client->setClientCallbacks(&linkCallbacks, false)'));
assert(!header.includes('notifications silencieuses'));
assert(!header.includes('readValue();\n          publishReport'));
assert(firmware.includes('if(xboxOwnsControl){ webSocket.sendTXT(client, "B,Manette Xbox aux commandes'));
assert(firmware.includes('xTaskCreatePinnedToCore(XboxBle::task, "xbox-ble", 8192, NULL, 1, NULL, 0)'));
assert(header.includes('reference.size() != 2 || uint8_t(reference[0]) != 1 || uint8_t(reference[1]) != 1'));
console.log('OK Xbox : rapport HID, stick gauche proportionnel, rotation sans LB, A/B, bras independants et bornes, persistance et deconnexion, reconnexion neutre et arbitrage web.');

// Mode Y : bascule sur front d'appui, dynamique et retour au neutre.
packet({session:3});ctx.motorsOn.store(true);ctx.armPos1=ctx.armPos2=90;
packet({session:3,buttons:16});assert(ctx.dynamicArmsEnabled);
for(let i=0;i<10;i++)packet({session:3,buttons:16});
assert(ctx.dynamicArmsEnabled,'Maintenir Y ne fait pas clignoter le mode');
const poses=[];
for(let i=0;i<150;i++){packet({session:3,y:0});poses.push([ctx.armPos1,ctx.armPos2]);}
assert(poses.at(-1).every(a=>a<90),'Les deux mains sont projetees vers avant');
assert(Math.max(...poses.slice(70).map(a=>a[0]))-Math.min(...poses.slice(70).map(a=>a[0]))>=2,'Balancement en marche continue');
for(let i=0;i<150;i++){packet({session:3,y:65535});poses.push([ctx.armPos1,ctx.armPos2]);}
assert(poses.at(-1).every(a=>a>90),'Les deux mains passent vers arriere');
for(let i=1;i<poses.length;i++)for(let arm=0;arm<2;arm++){
 assert(poses[i][arm]>=30 && poses[i][arm]<=150);
 assert(Math.abs(poses[i][arm]-poses[i-1][arm])<=4,'Mouvement borne a 200 degres/s');
}
packet({session:3,y:0,hat:1});assert(ctx.xboxArmLeft>0,'Commande manuelle du bras disponible');
for(let i=0;i<80;i++)packet({session:3,y:0,hat:1});
assert(ctx.armPos1<30,'Le bras commande manuellement reste prioritaire');
packet({session:3,y:0,buttons:16});assert(!ctx.dynamicArmsEnabled);
for(let i=0;i<200;i++)packet({session:3,y:0});
assert.equal(ctx.armPos1,90);assert.equal(ctx.armPos2,90);assert(!ctx.dynamicArmsReturning);
assert.equal(ctx.driveCmd.value,1,'Y ne modifie pas la translation');
packet({session:3});packet({session:3,buttons:16});assert(ctx.dynamicArmsEnabled);
for(let i=0;i<50;i++)packet({session:3,y:0});
packet({session:3,buttons:2});ctx.motorsOn.store(false);
const stopped=[ctx.armPos1,ctx.armPos2];
for(let i=0;i<30;i++)packet({session:3});
assert.deepEqual([ctx.armPos1,ctx.armPos2],stopped,'B suspend aussi le mouvement automatique');
ctx.setControlSource(true);packet({session:3,buttons:16});assert(ctx.dynamicArmsEnabled,'Y ignore en mode Web');
ctx.setControlSource(false);packet({session:3});packet({session:3,buttons:16});assert(!ctx.dynamicArmsEnabled);
console.log('OK balancier : Y, maintien, projection avant/arriere, oscillation, bornes, priorite manuelle, retour doux, suspension B et mode Web.');

// Une notification publie un etat, pas une impulsion de 350 ms.
const rx=vm.createContext({clock:20000,constrain:(v,l,h)=>Math.max(l,Math.min(h,v)),fabsf:Math.abs,powf:Math.pow,
  millis:()=>rx.clock,latest:{connected:true,valid:false,serial:0,session:19,rejected:0,
    notifications:0,reads:0,notificationMs:0,neutralSeen:false,neutralSerial:0}});
for(const [name,args] of [['decode','bytes,size,out'],['axis','raw'],['turn','raw']])
  vm.runInContext(translate(header,name,args),rx);
const publish=translate(header,'publishReport','bytes,size')
  .replace('Input parsed;','let parsed={};').replace(/port(?:ENTER|EXIT)_CRITICAL\(&inputMux\);/g,'');
vm.runInContext(publish,rx);
const neutralReport=new Uint8Array(16);neutralReport.set([0,128,0,128,0,128,0,128]);
ctx.stopRemoteMotion();ctx.xboxBaseline=false;ctx.motorsOn.store(true);ctx.webControlEnabled=false;
assert(rx.publishReport(neutralReport,16));ctx.input=rx.latest;ctx.clock=rx.clock;ctx.serviceXbox();
const heldReport=neutralReport.slice();heldReport[2]=heldReport[3]=0;heldReport[0]=heldReport[1]=255;
assert(rx.publishReport(heldReport,16));ctx.input=rx.latest;ctx.serviceXbox();
const receivedAt=rx.latest.receivedMs;
for(let i=0;i<3000;i++){
  ctx.clock+=20;ctx.serviceXbox();
  assert(ctx.xboxOwnsControl);assert.equal(ctx.driveCmd.value,1);assert.equal(ctx.turnCmd.value,1);
  assert.equal(ctx.lastDriveMs.value,ctx.clock);
}
assert.equal(rx.latest.receivedMs,receivedAt,'Le maintien ne fabrique pas de rapports');
rx.clock=ctx.clock;assert(rx.publishReport(neutralReport,16));ctx.input=rx.latest;ctx.serviceXbox();
assert.equal(ctx.driveCmd.value+0,0);assert.equal(ctx.turnCmd.value+0,0);
// Maintien des bras, puis perte de connexion : les deux consignes sont annulees.
rx.clock+=20;heldReport[12]=1;heldReport[6]=heldReport[7]=0;
rx.publishReport(heldReport,16);ctx.input=rx.latest;ctx.clock=rx.clock;ctx.serviceXbox();ctx.updateXboxArms();
assert(ctx.xboxArmLeft>0 && ctx.xboxArmRight>0);
rx.latest.connected=false;rx.latest.valid=false;ctx.serviceXbox();ctx.updateXboxArms();
assert.equal(ctx.driveCmd.value+0,0);assert.equal(ctx.turnCmd.value+0,0);
assert.equal(ctx.xboxArmLeft,0);assert.equal(ctx.xboxArmRight,0);assert(!ctx.xboxReady);
assert(!rx.publishReport(heldReport,16),'Pas de rapport accepte apres deconnexion');
console.log('OK persistance : 60 s sans nouveau rapport, translation et rotation maintenues, relachement immediat, bras et arret sur deconnexion.');
