// Test du HTML embarque avec serveur robot simule, sans connexion au vrai robot.
const fs=require('node:fs'),http=require('node:http'),path=require('node:path'),assert=require('node:assert/strict');
const {chromium}=require('playwright');
async function main(){
  const html=fs.readFileSync(path.join(__dirname,'telecommande.html'));
  const server=http.createServer((req,res)=>{res.setHeader('content-type','text/html; charset=utf-8');res.end(html);});
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  let browser;
  try{
    browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_BROWSER_PATH?{executablePath:process.env.PLAYWRIGHT_BROWSER_PATH}:{})});
    const context=await browser.newContext({viewport:{width:390,height:844},isMobile:true,hasTouch:true,deviceScaleFactor:1});
    await context.addInitScript(()=>{
      window.__mode="xbox";window.__messages=[];window.__ack=true;window.__telemetry=true;window.__robotOn=false;window.__dance=false;
      const realFetch=window.fetch.bind(window);
      window.fetch=(url,options)=>{
        if(String(url).startsWith('/control-mode?')){
          if(options?.method!=='POST')throw Error('Le changement de mode doit etre un POST');
          window.__mode=new URL(String(url),location.href).searchParams.get('mode');
          window.__socket.telemetry();return Promise.resolve(new Response(window.__mode));
        }
        return realFetch(url,options);
      };
      class MockSocket{
        static CONNECTING=0;static OPEN=1;static CLOSING=2;static CLOSED=3;
        constructor(url){
          this.url=url;this.readyState=0;this.bufferedAmount=0;window.__socket=this;
          setTimeout(()=>{this.readyState=1;this.onopen?.();this.receive('W,0');this.telemetry();},15);
          this.timer=setInterval(()=>{if(window.__telemetry)this.telemetry();},100);
        }
        receive(data){if(this.readyState===1)this.onmessage?.({data});}
        telemetry(){
          const p=[8.7,0,window.__robotOn?1:0,2000,2200,0,0,0,0,0,0,0,900,0,0,0,0,.08,0,0,0,window.__dance?1:0,0,0,0];
          this.receive('R,10,8,2,0,0,0,0,0');
          this.receive('C,'+(window.__mode==='web'?1:0)+',1,10,1,0');
          this.receive('S,'+(Date.now()>>>0)+',-1,'+p.join(','));
        }
        send(message){
          if(this.readyState!==1)throw Error('socket ferme');
          window.__messages.push({message,time:performance.now()});
          const p=message.split(',');
          if(p[0]==='D' && window.__ack)setTimeout(()=>this.receive('A,'+p[1]+','+(Date.now()>>>0)),5);
          if(p[0]==='M'){window.__robotOn=true;this.telemetry();}
          if(p[0]==='X'){window.__robotOn=false;window.__dance=false;this.telemetry();}
          if(p[0]==='Z'){window.__dance=false;this.telemetry();}
          if(p[0]==='G'){window.__dance=p[2]==='4';this.telemetry();}
        }
        close(){this.readyState=3;clearInterval(this.timer);setTimeout(()=>this.onclose?.(),0);}
      }
      window.WebSocket=MockSocket;
    });
    const page=await context.newPage(), errors=[];
    page.on('pageerror',error=>errors.push(error.message));
    await page.goto(`http://127.0.0.1:${server.address().port}`);
    await page.waitForFunction(()=>document.getElementById('connection').classList.contains('online'));
    await page.evaluate(()=>window.__socket.receive('Y,1'));
    assert.match(await page.locator('#dynamicRead').textContent(),/Balancier : active/);
    await page.evaluate(()=>window.__socket.receive('Y,0'));
    assert.match(await page.locator('#dynamicRead').textContent(),/Balancier : desactive/);
    assert.equal(await page.getByRole('button',{name:'Demarrer moteurs',exact:true}).isDisabled(),true);
    await page.getByRole('button',{name:'Piloter avec le site',exact:true}).tap();
    await page.waitForFunction(()=>controlSource==='web' && !modePending);
    await page.getByRole('button',{name:'Demarrer moteurs',exact:true}).tap();
    await page.waitForFunction(()=>document.getElementById('btn').textContent==='Couper moteurs');
    await page.locator('#joystick').scrollIntoViewIfNeeded();
    const originalScroll=await page.evaluate(()=>scrollY);
    const box=await page.locator('#joystick').boundingBox(), x=box.x+box.width/2,y=box.y+box.height/2;
    const cdp=await context.newCDPSession(page);
    async function touch(type,points){await cdp.send('Input.dispatchTouchEvent',{type,touchPoints:points});}
    await touch('touchStart',[{x,y,id:1}]);
    await touch('touchMove',[{x:x+42,y:y-60,id:1}]);
    await page.waitForTimeout(1150);
    const moving=await page.evaluate(()=>window.__messages.filter(x=>x.message.startsWith('D,')).map(x=>x.message.split(',')));
    assert(moving.some(p=>Number(p[3])>100 && Number(p[4])>100 && p[5]==='1'));
    assert(moving.length>=15,'Commandes maintenues a environ 25 Hz');
    assert.equal(await page.evaluate(()=>String(getSelection())),'');
    assert.equal(await page.locator('#joystick').evaluate(el=>getComputedStyle(el).touchAction),'none');
    assert.equal(await page.locator('#joystick').evaluate(el=>getComputedStyle(el).webkitUserSelect),'none');
    assert.equal(await page.locator('#joystick').evaluate(el=>el.dispatchEvent(new MouseEvent('contextmenu',{cancelable:true}))),false);
    assert.equal(await page.evaluate(()=>scrollY),originalScroll);
    await page.screenshot({path:path.join(__dirname,'telecommande-mobile.png'),fullPage:false});
    await touch('touchMove',[{x:385,y:70,id:1}]); // Sortie du cercle, appui capture.
    await page.waitForTimeout(100);
    assert.equal(await page.evaluate(()=>pointerId!==null),true);
    await touch('touchEnd',[]);
    await page.waitForTimeout(100);
    assert.equal(await page.evaluate(()=>pointerId),null);
    const lastDrive=await page.evaluate(()=>window.__messages.filter(x=>x.message.startsWith('D,')).at(-1).message);
    assert(lastDrive.endsWith(',0,0,0'),'Relachement = zero explicite');
    await page.getByRole('button',{name:/Danse dynamique/}).tap();
    await page.waitForFunction(()=>window.__messages.some(x=>x.message.startsWith('G,')&&x.message.endsWith(',4')));
    await page.getByRole('button',{name:'STOP deplacement',exact:true}).tap();
    assert(await page.evaluate(()=>window.__messages.some(x=>x.message==='Z')));
    await page.evaluate(y=>scrollTo(0,y),originalScroll);
    // Le WebSocket reste ouvert en mode Xbox, mais aucun ordre de marche web ne part.
    await page.evaluate(()=>chooseControlSource('xbox'));
    assert.equal(await page.evaluate(()=>controlSource),'xbox');
    assert.equal(await page.locator('#joystick').getAttribute('aria-disabled'),'true');
    const sentBefore=await page.evaluate(()=>window.__messages.filter(m=>m.message.startsWith('D,')&&m.message.endsWith(',1')).length);
    await touch('touchStart',[{x,y:y-40,id:7}]);await touch('touchEnd',[]);
    assert.equal(await page.evaluate(()=>pointerId),null);
    assert.equal(await page.evaluate(()=>window.__messages.filter(m=>m.message.startsWith('D,')&&m.message.endsWith(',1')).length),sentBefore);
    assert.equal(await page.evaluate(()=>window.__socket.readyState),1);
    await page.evaluate(()=>chooseControlSource('web'));
    assert.equal(await page.evaluate(()=>controlSource),'web');
    assert.equal(await page.evaluate(()=>pointerId),null);
    await page.evaluate(y=>scrollTo(0,y),originalScroll);
    // Changement provenant d'un autre ecran pendant un appui : annulation locale.
    await touch('touchStart',[{x,y:y-45,id:8}]);
    assert.equal(await page.evaluate(()=>pointerId!==null),true);
    await page.evaluate(()=>{window.__mode='xbox';window.__socket.telemetry();});
    assert.equal(await page.evaluate(()=>pointerId),null);
    assert.equal(await page.evaluate(()=>heldD),0);
    await touch('touchEnd',[]);
    await page.evaluate(()=>chooseControlSource('web'));
    assert.equal(await page.evaluate(()=>pointerId),null);
    // Telemetrie presente mais commandes non acquittees : couper le pilotage.
    await page.evaluate(()=>{window.__ack=false;});
    await touch('touchStart',[{x,y,id:2}]);
    await touch('touchMove',[{x,y:y-70,id:2}]);
    await page.waitForTimeout(600);
    assert.equal(await page.evaluate(()=>pointerId),null);
    assert.equal(await page.evaluate(()=>heldD),0);
    await touch('touchEnd',[]);
    await page.evaluate(()=>{window.__ack=true;});
    await page.waitForFunction(()=>document.getElementById('connection').classList.contains('online'));
    assert.equal(await page.evaluate(()=>pointerId),null,'Aucune reprise automatique apres reconnexion');
    // Une courte occupation du tampon ne doit pas annuler l'appui.
    await page.evaluate(()=>{window.__socket.bufferedAmount=100;});
    await page.evaluate(y=>scrollTo(0,y),originalScroll);
    await touch('touchStart',[{x,y,id:3}]);
    await touch('touchMove',[{x,y:y-65,id:3}]);
    await page.waitForTimeout(100);
    assert.equal(await page.evaluate(()=>pointerId!==null),true,JSON.stringify(await page.evaluate(()=>({notice:document.getElementById('notice').textContent,mode:controlSource,pendingStop,ready:!!linkReady(),scroll:scrollY,rect:document.getElementById('joystick').getBoundingClientRect().toJSON()}))));
    assert.equal(await page.evaluate(()=>window.__socket.readyState),1);
    await page.evaluate(()=>{window.__socket.bufferedAmount=0;});
    await page.waitForTimeout(100);
    assert(await page.evaluate(()=>window.__messages.filter(x=>x.message.startsWith('D,')).at(-1).message.endsWith(',1')));
    // Relachement pendant l'occupation : priorite au zero des que le tampon se libere.
    await page.evaluate(()=>{window.__socket.bufferedAmount=100;});
    await touch('touchEnd',[]);
    assert.equal(await page.evaluate(()=>pointerId),null);
    assert.equal(await page.evaluate(()=>pendingStop),true);
    await page.waitForTimeout(60);
    await page.evaluate(()=>{window.__socket.bufferedAmount=0;});
    await page.waitForFunction(()=>!pendingStop);
    assert(await page.evaluate(()=>window.__messages.filter(x=>x.message.startsWith('D,')).at(-1).message.endsWith(',0,0,0')));
    assert.equal(await page.evaluate(()=>window.__socket.readyState),1);
    // Les ACK confirment la liaison meme avec une telemetrie retardee.
    await page.evaluate(()=>{window.__telemetry=false;});
    await touch('touchStart',[{x,y:y-50,id:4}]);
    await page.waitForTimeout(750);
    assert.equal(await page.evaluate(()=>pointerId!==null),true);
    assert.equal(await page.evaluate(()=>window.__socket.readyState),1);
    assert(await page.evaluate(()=>now()-robotClockAt<100));
    await touch('touchEnd',[]);
    await page.evaluate(()=>{window.__telemetry=true;});
    // Un appui annule reste un arret, jamais une reprise automatique.
    await touch('touchStart',[{x,y:y-50,id:5}]);
    await touch('touchCancel',[]);
    assert.equal(await page.evaluate(()=>pointerId),null);
    // Une congestion persistante conserve la coupure de securite.
    await page.evaluate(()=>{window.__socket.bufferedAmount=100;});
    await touch('touchStart',[{x,y,id:6}]);
    await page.waitForTimeout(300);
    assert.equal(await page.evaluate(()=>pointerId),null);
    assert.equal(await page.evaluate(()=>heldD),0);
    await touch('touchEnd',[]);
    assert.deepEqual(errors,[]);
    console.log('OK navigateur mobile : source Xbox au demarrage, selection Web/Xbox par POST, joystick bloque en Xbox, changement distant en mouvement, gestes, danse/stop, perte ACK, reconnexion neutre et congestion.');
  }finally{
    if(browser)await browser.close();
    await new Promise(resolve=>server.close(resolve));
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});
