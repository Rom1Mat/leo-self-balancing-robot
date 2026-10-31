const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const root=__dirname;
const source=fs.readFileSync(path.join(root,'self_balancing_robot/self_balancing_robot.ino'),'utf8');
const html=fs.readFileSync(path.join(root,'telecommande.html'),'utf8');
const js=fs.readFileSync(path.join(root,'telecommande.js'),'utf8');
const normalized=value=>value.replace(/\r\n/g,'\n').trim();
assert(normalized(source.match(/R"HTML\(([\s\S]*?)\)HTML";/)[1])===normalized(html),'HTML embarque synchronise');
assert(normalized(html.match(/<script>([\s\S]*?)<\/script>/)[1])===normalized(js),'JavaScript embarque synchronise');
new vm.Script(js);
function translated(name){
  let text=source.match(new RegExp('bool '+name+'\\([^)]*\\)\\{([\\s\\S]*?)\\n\\}'))[0];
  return text.replace(/bool (\w+)\(([^)]*)\)/,(_,n,args)=>`function ${n}(${args.replace(/uint32_t /g,'')})`)
    .replace(/\(uint32_t\)\(now - stamp\)/g,'((now - stamp) >>> 0)')
    .replace(/\(int32_t\)\(value - previous\)/g,'((value - previous) | 0)');
}
const ctx=vm.createContext({FRAME_MAX_AGE_MS:350});
vm.runInContext(translated('freshRemoteFrame')+'\n'+translated('newerRemoteSequence'),ctx);
assert(ctx.freshRemoteFrame(1000,1000));
assert(ctx.freshRemoteFrame(1000,650));
assert(!ctx.freshRemoteFrame(1000,649));
assert(!ctx.freshRemoteFrame(1000,1001));
assert(ctx.freshRemoteFrame(50,0xffffff80)); // Debordement de millis().
assert(ctx.newerRemoteSequence(1,0));
assert(!ctx.newerRemoteSequence(2,2));
assert(!ctx.newerRemoteSequence(1,2));
assert(ctx.newerRemoteSequence(0,0xffffffff));
assert(!ctx.newerRemoteSequence(0xffffffff,0));
const telemetry=source.slice(source.indexOf('void formatTelemetry('),source.indexOf('void handleData('));
const format=telemetry.match(/snprintf\(b, size, "([^"]+)"/)[1];
assert.equal((format.match(/%/g)||[]).length,25);
assert(source.includes('"D,%lu,%lu,%d,%d,%d%c"'));
assert(source.includes('length >= 96'));
assert(source.includes("memchr(payload, '\\0', length) != NULL"));
assert(source.includes('if(manualOwner == client || danceClient == client) stopRemoteMotion();'));
assert(source.includes('if(manualOwner >= 0) wsNeedsRelease[manualOwner] = true;'));
assert(source.includes('wsNeedsRelease[client] = false;'));
assert(source.includes('if(wsNeedsRelease[client])'));
assert(!js.includes("fetch('/drive"));
assert(!js.includes("fetch('/data"));
assert(source.includes('server.send(410, "text/plain", "Pilotage migre en WebSocket'));
assert(source.includes('webSocket.onEvent(handleSocketEvent);'));
assert(source.includes('WiFi.setSleep(false);'));
assert.equal((source.match(/if\(!webControlEnabled\)\{ webSocket.sendTXT\(client, "B,Pilotage Web desactive : mode Xbox"/g)||[]).length,3,'D, M et G bloques cote serveur en mode Xbox');
assert(source.includes('server.on("/control-mode", HTTP_POST, handleControlMode)'));
assert(source.includes('bool webControlEnabled = false;'));
assert(source.includes('formatControlState(source, sizeof(source))'));
assert(source.includes('"A,%lu,%lu"'));
assert(source.includes('void updateArmGesture(){'));
assert(!source.includes('void sweepArm('));
console.log('OK : expiration 350 ms, sequence et debordements, rearmement, telemetrie 25 champs, taille des trames, HTTP ancien neutralise et sources embarquees synchronisees.');
