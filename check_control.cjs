// Tests numeriques des fonctions extraites du sketch et controles de l'interface.
// Execution en double precision JS : ce n'est pas une compilation ESP32.
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const assert = require('node:assert/strict');
const source = fs.readFileSync(path.join(__dirname, 'self_balancing_robot', 'self_balancing_robot.ino'), 'utf8');
function extract(name, returnType = 'float') {
  const start = source.indexOf(returnType + ' ' + name + '(');
  assert(start >= 0, name);
  let body = source.indexOf('{', start), end = body + 1, depth = 1;
  while(depth && end < source.length) {
    if(source[end] === '{') depth++;
    if(source[end] === '}') depth--;
    end++;
  }
  return source.slice(start, end)
    .replace(/(?:float|void) (\w+)\(([^)]*)\)/, (_, fn, args) => `function ${fn}(${args.replace(/\bconst\s+|\b(?:float|bool|int)\s*\*?\s*/g, '')})`)
    .replace(/\(int\)\(/g, 'Math.trunc(')
    .replace(/\b(?:float|bool|int)\b/g, 'let')
    .replace(/(\d+(?:\.\d*)?)f\b/g, '$1');
}
const ctx = vm.createContext({constrain: (x, a, b) => Math.min(b, Math.max(a, x)), fminf: Math.min, fabsf: Math.abs, THROTTLE_SCALE: 50});
vm.runInContext(extract('limitAcceleration') + '\n' + extract('integrateSpeed') + '\n' + extract('driveSpeedTarget') + '\n' + extract('speedTargetAngle') + '\n' + extract('nextSpeedIntegral'), ctx);
const accel = ctx.limitAcceleration, integrate = ctx.integrateSpeed;
const close = (a, b) => assert(Math.abs(a - b) < 1e-7, `${a} != ${b}`);
// Erreur persistante : la vitesse continue de monter, sans etre recalculee a zero.
let speed = 0;
const a = accel(2, 0, 12000, 1200, 80000);
assert.equal(a, 24000);
for(let i = 0; i < 50; i++) speed = integrate(speed, a, 0.002, 9000);
close(speed, 2400);
close(integrate(speed, 0, 0.002, 9000), speed);
// Les deux signes fonctionnent, avec la meme limitation.
assert.equal(accel(100, 0, 12000, 1200, 80000), 80000);
assert.equal(accel(-100, 0, 12000, 1200, 80000), -80000);
assert.equal(accel(0, 10, 12000, 1200, 80000), -12000);
assert.equal(accel(0, -10, 12000, 1200, 80000), 12000);
for(let i = 0; i < 10000; i++) speed = integrate(speed, 80000, 0.002, 9000);
assert.equal(speed, 9000);
// Apres une longue saturation, la premiere correction opposee freine immediatement.
assert.equal(integrate(speed, -80000, 0.002, 9000), 8840);
for(let i = 0; i < 10000; i++) speed = integrate(speed, -80000, 0.002, 9000);
assert.equal(speed, -9000);
assert.equal(integrate(speed, 80000, 0.002, 9000), -8840);
function run(dt) {
  let v = 0;
  for(let i = 0; i < Math.round(0.1 / dt); i++) v = integrate(v, a, dt, 9000);
  return v;
}
close(run(0.001), run(0.002));
close(run(0.004), run(0.002));
// Consignes avant/arriere inversees selon le retour utilisateur, reserve conservee.
assert.equal(ctx.driveSpeedTarget(1, 3000, 9000), 3000);
assert.equal(ctx.driveSpeedTarget(-1, 3000, 9000), -3000);
close(ctx.driveSpeedTarget(0, 3000, 9000), 0);
assert.equal(ctx.driveSpeedTarget(1, 6000, 9000), 5400);
assert.equal(ctx.driveSpeedTarget(1, 3000, 2000), 1200);
assert.equal(ctx.driveSpeedTarget(0.5, 3000, 9000), 1500);
assert.equal(ctx.driveSpeedTarget(1, 5200, 9000), 5200);
assert.equal(ctx.driveSpeedTarget(-1, 5200, 9000), -5200);
assert.equal(ctx.driveSpeedTarget(1, 5200, 6000), 3600);
// Au repos, vitesse nulle : l'inclinaison demandee passe de 0.4 a 1.2 degre.
close(0.08 * (3000 / 10000) * 50, 1.2);
assert(source.includes('float driveSpeed = 4800.0f;'));
assert(source.includes('float turnSpeed = 2500.0f;'));
assert(source.includes('turnF  += (commandTurn - turnF ) * turnBlend;'));
assert(source.includes('turnF * requestedTurn, -turnRoom, turnRoom'));
// Gains demandes, coherents entre le firmware et les valeurs initiales HTML.
assert(source.includes('float Kp = 18000.0f;'));
assert(source.includes('float Kd = 700.0f;'));
assert(source.includes('float maxAccel = 200000.0f;'));
for(const [id, value] of [['kp',18000],['kd',700],['ma',200000],['ds',4800],['tr',2500],['ms',12000]]) {
  const input = source.match(new RegExp('<input[^>]+id="' + id + '"[^>]+>'))[0];
  assert(input.includes(`value="${value}"`));
}
close(ctx.speedTargetAngle(8.7, 1.2), 9.9);
close(ctx.speedTargetAngle(8.7, -1.2), 7.5);
assert(source.includes('float targetTotal = speedTargetAngle(cfg.targetAngle, targetAdjust);'));
// Regression du signe dans un modele ideal de pendule inverse.
// Hypotheses de test UNIQUEMENT : roues 30 mm, CG 200 mm, moteur 200 pas/tour,
// pas de glissement, couple suffisant, capteurs ideaux. Ne valide pas le robot reel.
function simulatedMotion(invertedFeedback = false, ki = 0, bias = 0, holdSeconds = 4) {
  const dt = 0.001, stepsPerMetre = 3200 / (2 * Math.PI * 0.03);
  let theta = 0, rate = 0, speed = 0, filteredSpeed = 0, atRelease = 0, integral = 0;
  for(let i = 0; i < (holdSeconds + 4) * 1000; i++) {
    const target = i < holdSeconds * 1000 ? 2250 : 0;
    filteredSpeed += (speed - filteredSpeed) * (1 - Math.exp(-dt / 0.05));
    const error = (filteredSpeed - target) / 10000;
    integral = ctx.nextSpeedIntegral(integral, error, dt, 0.08, ki, false);
    const correction = Math.max(-6, Math.min(6, (0.08 * error + ki * integral) * 50));
    const tiltTarget = (invertedFeedback ? -1 : 1) * ctx.speedTargetAngle(0, correction);
    const acceleration = accel(tiltTarget - (theta * 180 / Math.PI + bias), rate * 180 / Math.PI, 18000, 700, 200000);
    const nextSpeed = integrate(speed, acceleration, dt, 9000);
    const wheelAcceleration = -(nextSpeed - speed) / dt / stepsPerMetre;
    const angularAcceleration = (9.81 * theta - wheelAcceleration) / 0.2;
    rate += angularAcceleration * dt;
    theta += rate * dt;
    speed = nextSpeed;
    if(Math.abs(theta) > Math.PI / 4) return {fell: true};
    if(i === holdSeconds * 1000 - 1) atRelease = speed;
  }
  return {fell: false, atRelease, finalSpeed: speed};
}
const motion = simulatedMotion();
assert(!motion.fell);
assert(Math.abs(motion.atRelease - 2250) < 50);
assert(Math.abs(motion.finalSpeed) < 50);
assert(simulatedMotion(true).fell); // L'ancien signe est detecte par ce test.
// Integrale bornee a +/-2 degres, decharge autorisee pendant une saturation.
assert(source.includes('float spKp = 0.08f, spKi = 0.05f;'));
assert.equal(ctx.nextSpeedIntegral(0, 0.3, 0.002, 0.08, 0.05, true), 0);
let integral = 0;
for(let i = 0; i < 10000; i++) integral = ctx.nextSpeedIntegral(integral, 0.3, 0.002, 0.08, 0.05, false);
close(integral * 0.05 * 50, 2);
assert(ctx.nextSpeedIntegral(integral, -0.3, 0.002, 0.08, 0.05, true) < integral);
close(ctx.nextSpeedIntegral(integral, 0.3, 0.002, 0.08, 0, false), 0);
close(ctx.nextSpeedIntegral(integral, 0.3, 0.002, 0.08, 0.5, false) * 0.5 * 50, 2);
assert.equal(ctx.nextSpeedIntegral(0, 1, 0.002, 1, 0.05, false), 0);
const biasedP = simulatedMotion(false, 0, -1, 12);
const biasedPI = simulatedMotion(false, 0.05, -1, 12);
assert(!biasedP.fell && !biasedPI.fell);
assert(Math.abs(biasedP.atRelease - 2250) > 2000);
assert(Math.abs(biasedPI.atRelease - 2250) < 100);
// Bornes sur une grille d'erreurs, vitesses angulaires et dt.
for(const e of [-10,-2,0,2,10]) for(const rate of [-100,-10,0,10,100]) {
  const acc = accel(e, rate, 12000, 1200, 80000);
  assert(Math.abs(acc) <= 80000);
  for(const dt of [0.001,0.002,0.004]) {
    const v = integrate(0, acc, dt, 9000);
    assert(Math.abs(v) <= 80000 * dt + 1e-8);
    assert(Math.abs(v) <= 9000);
  }
}
assert(source.includes('motorSpeed = integrateSpeed(previousSpeed, dbgAccel, dt, cfg.maxSpeed);'));
assert(!/outLP|motorSpeedFilt|adaptiveCoefficient|OUTPUT_SCALE|deadband/.test(source));
assert(source.includes('motorSpeed = 0; robotSpeedFilt = 0; speedIntegral = 0;'));
assert(source.includes('if(!readMPU(ax, az, gy))'));
assert(source.includes('dt > 0.020f'));
assert(source.includes('motorRequest.exchange(-1)'));
assert(source.includes('dbgCommandAgeMs > MANUAL_TIMEOUT_MS'));
assert(source.includes('const double STEP_SPS_MAX = 20000.0;'));
assert(source.includes('float maxSpeed = 12000.0f;'));
// Pas de traitements HTTP / allocations String dans les sections critiques.
for(const critical of source.matchAll(/portENTER_CRITICAL\(&configMux\);([\s\S]*?)portEXIT_CRITICAL\(&configMux\);/g)) {
  assert(!critical[1].includes('server.'));
}
const html = source.match(/R"HTML\(([\s\S]*?)\)HTML";/)[1];
const js = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(js);
const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(m => m[1]);
assert.equal(new Set(ids).size, ids.length);
for(const m of js.matchAll(/bind\('([^']+)','([^']+)'/g)) assert(ids.includes(m[1]) && ids.includes(m[2]));
for(const m of js.matchAll(/getElementById\('([^']+)'\)/g)) assert(ids.includes(m[1]));
for(const m of js.matchAll(/bind\('[^']+','[^']+','([^']+)'/g)) assert(source.includes(`hasArg("${m[1]}")`));
console.log('OK : gains par defaut, integration, bornes, freinage, dt reel, resets, timeout, pilotage, signe du retour vitesse et reglages web.');
console.log(`Modele ideal uniquement : consigne +2250 pas/s, vitesse ${motion.atRelease.toFixed(0)} au relachement, puis ${motion.finalSpeed.toFixed(0)} apres freinage.`);
console.log(`Modele avec biais d'angle de -1 degre : P seul ${biasedP.atRelease.toFixed(0)} pas/s ; PI ${biasedPI.atRelease.toFixed(0)} pas/s, pour une consigne de 2250.`);

// Choregraphie : execute les fonctions de transition reelles du sketch.
function danceArray(name) {
  const match = source.match(new RegExp('const (?:float|int) ' + name + '\\[\\d+\\] = \\{([\\s\\S]*?)\\};'));
  assert(match, name);
  return match[1].split(',').map(x => Number(x.trim().replace(/f$/, '')));
}
const driveSequence = danceArray('DANCE_DRIVE'), turnSequence = danceArray('DANCE_TURN');
ctx.DANCE_MOVE_MS = 30000;
vm.runInContext(extract('danceBlend') + '\n' + extract('danceMotionAt') + '\n' + extract('danceStabilityScale'), ctx);
assert.equal(driveSequence.length, 10);
assert.equal(turnSequence.length, 10);
assert(driveSequence.some(x => x > 0) && driveSequence.some(x => x < 0));
assert(turnSequence.some(x => x > 0) && turnSequence.some(x => x < 0));
for(let i = 0; i < 10; i += 2) {
  close(driveSequence[i] + driveSequence[i+1], 0);
  close(turnSequence[i] + turnSequence[i+1], 0);
}
for(const sequence of [driveSequence, turnSequence]) {
  close(ctx.danceMotionAt(0, sequence), 0);
  close(ctx.danceMotionAt(30000, sequence), 0);
  close(ctx.danceMotionAt(31999, sequence), 0);
  for(let ms = 0; ms < 30000; ms += 10) assert(Math.abs(ctx.danceMotionAt(ms, sequence)) <= 1);
  for(let ms = 3000; ms < 30000; ms += 3000)
    assert(Math.abs(ctx.danceMotionAt(ms - 0.001, sequence) - ctx.danceMotionAt(ms, sequence)) < 0.0001);
}
// Un repos d'une seconde avant chaque nouveau mouvement, y compris aux inversions.
for(const sequence of [driveSequence, turnSequence]) {
  for(let base=0; base<30000; base+=3000) {
    for(let phase=2000; phase<3000; phase+=10)close(ctx.danceMotionAt(base+phase,sequence),0);
    close(ctx.danceMotionAt(base+800,sequence),sequence[base/3000]);
  }
  // Derivee de consigne bornee : transition de vitesse, jamais un saut.
  let prior=0;
  for(let ms=0;ms<=32000;ms+=2){
    const value=ctx.danceMotionAt(ms,sequence);
    assert(Math.abs(value-prior)<=0.00376); prior=value;
  }
}
close(ctx.danceStabilityScale(0,0,false),1);
close(ctx.danceStabilityScale(1.5,12,false),1);
close(ctx.danceStabilityScale(5,0,false),0);
close(ctx.danceStabilityScale(0,40,false),0);
close(ctx.danceStabilityScale(0,0,true),0);
assert(ctx.danceStabilityScale(3,20,false)>0 && ctx.danceStabilityScale(3,20,false)<1);
assert(source.includes('if(danceStability.load() >= 0.6f)'));
assert(source.includes('commandDrive *= stability; commandTurn *= stability;'));
assert(source.includes('left = 90 + (int)(40.0f * wave)'));
assert(source.includes('80.0f * tickMs / 1000.0f'));
assert(source.includes('DANCE_MOVE_MS = 30000, DANCE_TOTAL_MS = 32000'));
assert(source.includes('PI * 0.090f / (200.0f * 16.0f)'));
assert(source.includes('WHEEL_TRACK_M = 0.160f'));
assert(source.includes('if(radius >= DANCE_RADIUS_STOP_M) danceTravelBlocked.store(true);'));
assert(source.includes('if(dancing && danceTravelBlocked.load()){ commandDrive = 0; commandTurn = 0; }'));
assert(source.includes('server.on("/dance/stop", handleDanceStop);'));
assert(source.includes('position += constrain(target - position, -6, 6);'));
const danceUpdate = source.slice(source.indexOf('void updateDance(){'), source.indexOf('void updateDanceOdometry('));
assert(!/\bdelay\(|\bwhile\(/.test(danceUpdate));
assert(danceUpdate.includes('elapsed >= DANCE_TOTAL_MS'));
// Verifie l'odometrie REELLE du sketch apres traduction, pas la dynamique physique.
ctx.METRES_PER_STEP = Math.PI * 0.09 / 3200;
ctx.WHEEL_TRACK_M = 0.160;
ctx.DANCE_RADIUS_STOP_M = 0.65;
ctx.cosf = Math.cos; ctx.sinf = Math.sin; ctx.sqrtf = Math.sqrt;
ctx.danceX=0; ctx.danceY=0; ctx.danceHeading=0;
ctx.danceActive={load:()=>true};
ctx.danceRadiusEstimate={value:0,store(value){this.value=value;}};
ctx.danceTravelBlocked={value:false,store(value){this.value=value;}};
vm.runInContext(extract('updateDanceOdometry','void'),ctx);
ctx.updateDanceOdometry(1000,0,1);
close(ctx.danceX, 1000 * ctx.METRES_PER_STEP);
close(ctx.danceY, 0);
ctx.danceX=0.64;
ctx.updateDanceOdometry(1000,0,1);
assert(ctx.danceTravelBlocked.value);
// Empreinte des consignes avec un suivi de vitesse a retard simple (0.5 s).
// Ce modele de suivi sert au test de trajectoire, sans valider les gains du robot.
ctx.danceX=0; ctx.danceY=0; ctx.danceHeading=0;
ctx.danceTravelBlocked.value=false;
let dv=0, df=0, tf=0, peak=0;
const dt=0.01;
for(let tick=0; tick<3200; tick++) {
  const ms=tick*10, blocked=ctx.danceTravelBlocked.value;
  const d=blocked?0:ctx.danceMotionAt(ms,driveSequence);
  const t=blocked?0:ctx.danceMotionAt(ms,turnSequence);
  df+=(d-df)*(1-Math.exp(-dt/0.06));
  tf+=(t-tf)*(1-Math.exp(-dt/0.2));
  const target=ctx.driveSpeedTarget(df,5200,9000);
  dv+=(target-dv)*(1-Math.exp(-dt/0.5));
  ctx.updateDanceOdometry(dv,tf*400,dt);
  peak=Math.max(peak,ctx.danceRadiusEstimate.value);
}
assert(peak > 0.4 && peak < 0.65);
assert(!ctx.danceTravelBlocked.value, "La choregraphie nominale ne doit pas declencher la limite de rayon");
assert(Math.abs(dv)<500);
console.log(`OK danse : 32 s, rampes et repos avant inversion, repli selon stabilite, bras doux, geometrie 90/160, arret et odometrie. Modele de suivi simple : rayon max ${peak.toFixed(2)} m.`);

// Les interactions tactiles et le transport sont verifies dans check_remote_browser.cjs.
assert(source.includes('WebSocketsServer webSocket(81);'));
assert(source.includes('if(type != WStype_TEXT || length == 0 || length >= 96) return;'));
assert(source.includes('webSocket.onEvent(handleSocketEvent);'));
assert(source.includes('webSocket.broadcastTXT(packet);'));
assert(source.includes('const uint32_t MANUAL_TIMEOUT_MS = 350;'));
assert(!source.includes('void sweepArm('));
console.log('OK : transport WebSocket enregistre, taille bornee, watchdog 350 ms, gestes non bloquants.');
