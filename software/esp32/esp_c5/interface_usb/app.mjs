import {UsbLink} from './serial-link.mjs';
const $ = id => document.getElementById(id);
let compatible=false, fault=null, poll=null, handshake=null, connecting=false;
let firmwareDetected=null;
const inversions={left:null,right:null};
const entries=[];
function log(text) {
  entries.push(text); if(entries.length>250) entries.shift();
  $('log').textContent=entries.join('\n'); $('log').scrollTop=$('log').scrollHeight;
}
function message(text,error=false) { $('message').textContent=text; $('message').classList.toggle('error',error); }
function controls() {
  const ready=link.active && compatible && fault===false;
  document.querySelectorAll('[data-controls]').forEach(e=>e.disabled=!ready);
  $('stop').disabled=!link.active; $('refresh').disabled=!link.active;
  $('disconnect').disabled=!link.active; $('connect').disabled=link.active||connecting||!navigator.serial;
  $('badge').textContent=!link.active?'Déconnecté':fault===true?'Défaut ESP':ready?'USB connecté':'Vérification…';
  $('badge').className='badge'+(fault===true?' error':ready?' connected':'');
  if(ready) { clearTimeout(handshake); message('Prêt. Règle une consigne, puis clique sur Appliquer.'); }
}
const link=new UsbLink(navigator.serial,{
  connection(active) {
    if(!active) {
      compatible=false; fault=null; inversions.left=inversions.right=null; clearInterval(poll); clearTimeout(handshake);
      firmwareDetected=null;
      $('firmware-state').textContent='Interface USB 2 · attend ROVER_3, ROVER_2 ou TOUT_MOTEUR_4 · ESP non connecté.';
      message('Déconnecté. Ferme le moniteur série avant de reconnecter l’ESP.');
    }
    controls();
  },
  sent(command) { if(command!=='etat') log('→ '+command); },
  error(error) { message('Erreur USB : '+error.message+' Ferme le moniteur série de VS Code si le port est occupé.',true); log('! '+error.message); },
  line(line) {
    log('← '+line);
    const firmware=line.trim().match(/^\[FIRMWARE\] ([A-Z0-9_]+)$/);
    if(firmware) {
      firmwareDetected=firmware[1];
      compatible=['TOUT_MOTEUR_4','ROVER_2','ROVER_3'].includes(firmwareDetected);
      $('firmware-state').textContent=`Interface USB 2 · ESP : ${firmwareDetected} · ${compatible?'compatible':'attend ROVER_3, ROVER_2 ou TOUT_MOTEUR_4'}`;
    }
    const motor=line.match(/^\[MOTEURS\] G=(\d+) % ; D=(\d+) % ; BRAKE=(\d) ; defaut=(\d)/);
    if(motor) {
      fault=motor[4]==='1';
      $('motor-state').textContent=`ESP : gauche ${motor[1]} % · droite ${motor[2]} % · frein ${motor[3]}`;
    }
    const wheel=line.match(/^\[(GAUCHE|DROITE)\] Demande=(\d+) % ; PWM=(\d+) % ; sens=(\d) ; DIR=(\d) ; inverse=(\d)/);
    if(wheel) {
      const side=wheel[1]==='GAUCHE'?'left':'right';
      inversions[side]=Number(wheel[6]);
      $(side+'-state').textContent=`ESP : ${wheel[2]} % · PWM ${wheel[3]} % · ${wheel[4]==='0'?'avant':'arrière'} · DIR=${wheel[5]} · inversion=${wheel[6]}`;
    }
    const servo=line.match(/\[SERVO\] Signal (actif|coupe) ; consigne en cours ([+\-\d.]+) ; cible ([+\-\d.]+) ; vitesse max ([\d.]+)/);
    if(servo) $('servo-state').textContent=`ESP : cible ${servo[3]}° · consigne en cours ${servo[2]}° · ${servo[4]}°/s · signal ${servo[1]==='actif'?'actif':'coupé'}`;
    const range=line.match(/angles \+\/-([\d.]+)/);
    if(range) {
      const limit=Number(range[1]);
      for(const id of ['angle','angle-number']) { $(id).min=-limit; $(id).max=limit; }
      $('angle-note').textContent=`0 = milieu · plage −${limit}° à +${limit}° · ${line.includes('INDICATIFS')?'angles indicatifs, course non mesurée':'course renseignée par l’utilisateur'}.`;
    }
    controls();
    if(line.startsWith('[DEFAUT]')) { fault=true; controls(); message(line,true); }
    else if(fault===true) message('Défaut ESP : commandes de mouvement bloquées. La cause apparaît dans le retour série.',true);
  }
});
async function send(command) {
  try { await link.send(command); } catch(error) { if(!error.message.includes('annulée par STOP')) message(error.message,true); }
}
async function stop() {
  $('motor').value=$('motor-number').value=0; $('motor-value').textContent='0 %';
  $('left-speed').value=$('right-speed').value=0;
  try { await link.stop(); await link.send('etat'); } catch(error) { message('STOP non confirmé : '+error.message,true); }
}
$('connect').addEventListener('click',async()=>{
  connecting=true; compatible=false; fault=null; firmwareDetected=null; controls();
  try {
    await link.connect();
    handshake=setTimeout(()=>{if(link.active && !compatible) message(`Firmware non reconnu : ${firmwareDetected??'aucun identifiant reçu'}. Cette interface attend ROVER_3, ROVER_2 ou TOUT_MOTEUR_4. Téléverse l’environnement correspondant, puis reconnecte.`,true);},2500);
    poll=setInterval(()=>{if(link.active) send('etat');},1500);
  } catch(error) {
    if(error.name==='NotFoundError') message('Connexion annulée : aucun port choisi.');
    else message('Impossible d’ouvrir l’USB. Ferme le moniteur série de VS Code. '+error.message,true);
  } finally { connecting=false; controls(); }
});
$('disconnect').addEventListener('click',()=>link.disconnect().catch(error=>message(error.message,true)));
$('stop').addEventListener('click',stop);
$('refresh').addEventListener('click',()=>send('etat'));
$('clear').addEventListener('click',()=>{entries.length=0; $('log').textContent='';});
function bind(slider,number,out,suffix) {
  $(slider).addEventListener('input',()=>{ $(number).value=$(slider).value; $(out).textContent=$(slider).value+suffix; });
  $(number).addEventListener('input',()=>{ if($(number).validity.valid) {$(slider).value=$(number).value; $(out).textContent=$(number).value+suffix;} });
}
bind('motor','motor-number','motor-value',' %'); bind('angle','angle-number','angle-value','°');
$('motor-form').addEventListener('submit',e=>{e.preventDefault(); if(e.target.reportValidity()) send('m '+$('motor-number').value);});
$('servo-form').addEventListener('submit',e=>{e.preventDefault(); if(e.target.reportValidity()) send('s '+$('angle-number').value);});
$('speed-form').addEventListener('submit',e=>{e.preventDefault(); if(e.target.reportValidity()) send('v '+$('servo-speed').value);});
$('motor-zero').addEventListener('click',()=>{ $('motor').value=$('motor-number').value=0; $('motor-value').textContent='0 %'; $('left-speed').value=$('right-speed').value=0; send('m 0'); });
for(const [side,cmd] of [['left','g'],['right','d']]) {
  $(side+'-form').addEventListener('submit',e=>{e.preventDefault();if(e.target.reportValidity())send(cmd+' '+$(side+'-speed').value);});
  $(side+'-forward').addEventListener('click',()=>send('sens'+cmd+' 0'));
  $(side+'-reverse').addEventListener('click',()=>send('sens'+cmd+' 1'));
  $(side+'-invert').addEventListener('click',()=>{
    if(inversions[side]===null){message('Actualise l’état avant de corriger le sens.',true);return;}
    send('inverse'+cmd+' '+(inversions[side]===0?1:0));
  });
}
$('center').addEventListener('click',()=>{ $('angle').value=$('angle-number').value=0; $('angle-value').textContent='0°'; send('centre'); });
document.addEventListener('keydown',e=>{if(e.key==='Escape'&&link.active){e.preventDefault(); stop();}});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&link.active) stop();});
window.addEventListener('pagehide',()=>{if(link.active) link.stop().catch(()=>{});});
if(navigator.serial) navigator.serial.addEventListener('disconnect',event=>{if(event.target===link.port||event.port===link.port) link.disconnect(false);});
if(!navigator.serial) message('Ouvre cette page dans Chrome ou Edge : ce navigateur ne propose pas l’accès série USB.',true);
controls();
