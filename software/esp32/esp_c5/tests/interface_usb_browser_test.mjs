// Test navigateur avec USB simule. Ne jamais connecter un port serie reel ici.
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {resolve} from 'node:path';
const {chromium}=await import(pathToFileURL(process.env.PLAYWRIGHT_MODULE).href);
const browser=await chromium.launch({headless:true,executablePath:'C:/Program Files/Google/Chrome/Application/chrome.exe'});
try {
  const context=await browser.newContext({viewport:{width:1280,height:1050}});
  await context.addInitScript(()=>{
    window.__writes=[]; window.__fakeFault=false; window.__badFirmware=false; window.__firmware='TOUT_MOTEUR_4';
    let input, motors=[0,0], dirs=[0,0], inverse=[1,1], angle=0, speed=30, signal=true;
    const encoder=new TextEncoder();
    const port={
      open:async()=>{},setSignals:async()=>{},close:async()=>{},
      readable:new ReadableStream({start(c){input=c;}}),
      writable:new WritableStream({write(bytes){
        const command=new TextDecoder().decode(bytes).trim(); window.__writes.push(command);
        if(command.startsWith('m ')) motors.fill(Number(command.slice(2)));
        if(command.startsWith('g ')) motors[0]=Number(command.slice(2));
        if(command.startsWith('d ')) motors[1]=Number(command.slice(2));
        if(command.startsWith('sensg ')) dirs[0]=Number(command.slice(6));
        if(command.startsWith('sensd ')) dirs[1]=Number(command.slice(6));
        if(command.startsWith('inverseg ')) inverse[0]=Number(command.slice(9));
        if(command.startsWith('inversed ')) inverse[1]=Number(command.slice(9));
        if(command.startsWith('s ')) {angle=Number(command.slice(2));signal=true;}
        if(command.startsWith('v ')) speed=Number(command.slice(2));
        if(command==='centre') {angle=0;signal=true;}
        if(command==='stop') {motors.fill(0);signal=false;}
        if(command==='etat') input.enqueue(encoder.encode([
          '[FIRMWARE] '+(window.__badFirmware?'ROVER_1':window.__firmware),
          `[MOTEURS] G=${motors[0]} % ; D=${motors[1]} % ; BRAKE=0 ; defaut=${window.__fakeFault?1:0}`,
          ...['GAUCHE','DROITE'].map((nom,i)=>`[${nom}] Demande=${motors[i]} % ; PWM=${motors[i]} % ; sens=${dirs[i]} ; DIR=${dirs[i]^inverse[i]} ; inverse=${inverse[i]}`),
          `[SERVO] Signal ${signal?'actif':'coupe'} ; consigne en cours ${angle.toFixed(1)} ; cible ${angle.toFixed(1)} ; vitesse max ${speed.toFixed(1)} deg/s`,
          '        Plage 800..2200 us ; angles +/-90.0 (INDICATIFS). Position physique non mesuree.',
          ...(window.__fakeFault?['[DEFAUT] Cause=test_simule ; GPIO=12']:[]),''].join('\n')));
      }})
    };
    const serial=new EventTarget(); serial.requestPort=async()=>port;
    Object.defineProperty(navigator,'serial',{value:serial,configurable:true});
  });
  const page=await context.newPage();
  const errors=[]; page.on('pageerror',error=>errors.push(error.message));
  await page.goto('http://127.0.0.1:8767/');
  assert(await page.getByRole('button',{name:'Appliquer la vitesse'}).isDisabled());
  await page.getByRole('button',{name:'Connecter l’ESP'}).click();
  await page.waitForFunction(()=>document.querySelector('#badge').textContent==='USB connecté');
  assert.deepEqual(await page.evaluate(()=>window.__writes),['etat']);
  assert((await page.locator('#servo-state').textContent()).includes('signal actif'));
  await page.getByRole('button',{name:'Recentrer à 0°'}).click();
  await page.waitForFunction(()=>window.__writes.includes('centre'));
  await page.locator('#motor-number').fill('5');
  await page.getByRole('button',{name:'Appliquer la vitesse'}).click();
  await page.locator('#angle-number').fill('-20');
  await page.getByRole('button',{name:'Appliquer l’angle'}).click();
  await page.locator('#servo-speed').fill('60');
  await page.getByRole('button',{name:'Régler',exact:true}).click();
  await page.waitForFunction(()=>window.__writes.includes('v 60'));
  const sent=await page.evaluate(()=>window.__writes);
  assert(sent.includes('m 5')&&sent.includes('s -20')&&sent.includes('v 60'));
  await page.locator('#left-speed').fill('7');
  await page.locator('#left-form button').click();
  await page.locator('#right-speed').fill('13');
  await page.locator('#right-form button').click();
  await page.waitForFunction(()=>window.__writes.includes('d 13'));
  await page.locator('#refresh').click();
  await page.waitForFunction(()=>document.querySelector('#motor-state').textContent.includes('gauche 7 %'));
  assert((await page.locator('#motor-state').textContent()).includes('droite 13 %'));
  await page.locator('#motor-zero').click();
  await page.locator('#left-reverse').click();
  await page.locator('#right-invert').click();
  await page.waitForFunction(()=>window.__writes.includes('inversed 0'));
  assert((await page.evaluate(()=>window.__writes)).includes('sensg 1'));
  await page.locator('#refresh').click();
  await page.waitForFunction(()=>document.querySelector('#right-state').textContent.includes('inversion=0'));
  await page.screenshot({path:resolve('.pio/interface_usb_desktop.png'),fullPage:true});
  await page.keyboard.press('Escape');
  await page.waitForFunction(()=>window.__writes.includes('stop'));
  assert.equal(await page.locator('#motor-number').inputValue(),'0');
  await page.setViewportSize({width:390,height:844});
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  await page.screenshot({path:resolve('.pio/interface_usb_mobile.png'),fullPage:true});
  await page.evaluate(()=>{window.__fakeFault=true;});
  await page.getByRole('button',{name:'Actualiser l’état'}).click();
  await page.waitForFunction(()=>document.querySelector('#badge').textContent==='Défaut ESP');
  assert(await page.getByRole('button',{name:'Appliquer la vitesse'}).isDisabled());
  assert(await page.getByRole('button',{name:'Appliquer l’angle'}).isDisabled());
  assert(await page.locator('#stop').isEnabled());
  await page.getByRole('button',{name:'Déconnecter',exact:true}).click();
  await page.waitForFunction(()=>document.querySelector('#badge').textContent==='Déconnecté');
  await page.reload();
  await page.evaluate(()=>{window.__firmware='ROVER_2';});
  await page.getByRole('button',{name:'Connecter l’ESP'}).click();
  await page.waitForFunction(()=>document.querySelector('#badge').textContent==='USB connecté');
  assert.deepEqual(await page.evaluate(()=>window.__writes),['etat']);
  await page.getByRole('button',{name:'Déconnecter',exact:true}).click();
  await page.reload();
  await page.evaluate(()=>{window.__firmware='ROVER_3';}); // memes commandes que ROVER_2
  await page.getByRole('button',{name:'Connecter l’ESP'}).click();
  await page.waitForFunction(()=>document.querySelector('#badge').textContent==='USB connecté');
  assert.deepEqual(await page.evaluate(()=>window.__writes),['etat']);
  await page.getByRole('button',{name:'Déconnecter',exact:true}).click();
  await page.reload();
  await page.evaluate(()=>{window.__badFirmware=true;});
  await page.getByRole('button',{name:'Connecter l’ESP'}).click();
  await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('Firmware non reconnu'));
  assert(await page.getByRole('button',{name:'Appliquer la vitesse'}).isDisabled());
  assert.deepEqual(errors,[]);
  console.log('Navigateur OK : aucune commande de mouvement a la connexion ; m/s/v ; STOP ; defaut bloque ; firmware controle ; mobile sans debordement. USB simule uniquement.');
  await context.close();
} finally {await browser.close();}
