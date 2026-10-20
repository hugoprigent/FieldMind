import test from 'node:test';
import assert from 'node:assert/strict';
import {UsbLink} from '../interface_usb/serial-link.mjs';

function fakePort() {
  let incoming, releaseWrite;
  const writes=[];
  const port={
    writes, blocked:false, closed:false,
    readable:new ReadableStream({start(controller){incoming=controller;}}),
    writable:new WritableStream({async write(bytes){
      const text=new TextDecoder().decode(bytes); writes.push(text);
      if(port.blocked) await new Promise(resolve=>releaseWrite=resolve);
    }}),
    open:async options=>{assert.equal(options.baudRate,115200);},
    setSignals:async signals=>{assert.equal(signals.dataTerminalReady,false);assert.equal(signals.requestToSend,false);},
    close:async()=>{port.closed=true;},
    receive:text=>incoming.enqueue(new TextEncoder().encode(text)),
    unblock:()=>{port.blocked=false;releaseWrite?.();}
  };
  return port;
}
const nextTick=()=>new Promise(resolve=>setTimeout(resolve,0));

test('connexion sans mouvement et lecture des lignes fragmentees',async()=>{
  const port=fakePort(), lines=[];
  const link=new UsbLink({requestPort:async()=>port},{line:line=>lines.push(line)});
  await link.connect();
  assert.deepEqual(port.writes,['etat\n']);
  port.receive('[FIRMWARE] TOUT_MO');
  port.receive('TEUR_2\r\n[MOTEURS] etat\n');
  await nextTick();
  assert.deepEqual(lines,['[FIRMWARE] TOUT_MOTEUR_2','[MOTEURS] etat']);
  await link.disconnect();
  assert.equal(port.writes.at(-1),'stop\n');
  assert.equal(port.closed,true);
  assert.equal(link.active,false);
});

test('STOP supprime les mouvements encore en file',async()=>{
  const port=fakePort();
  const link=new UsbLink({requestPort:async()=>port});
  await link.connect();
  port.blocked=true;
  const inFlight=link.send('m 5');
  const pending=link.send('m 50').then(()=>false,()=>true);
  const pendingServo=link.send('s 90').then(()=>false,()=>true);
  const stopping=link.stop();
  port.unblock();
  await Promise.all([inFlight,stopping]);
  assert.equal(await pending,true); assert.equal(await pendingServo,true);
  assert.deepEqual(port.writes,['etat\n','m 5\n','stop\n']);
  await link.disconnect(false);
});

test('aucune injection multiligne, aucune ecriture apres deconnexion',async()=>{
  const port=fakePort(); const link=new UsbLink({requestPort:async()=>port});
  await link.connect();
  await assert.rejects(link.send('m 5\nm 100'));
  await link.disconnect();
  await assert.rejects(link.send('m 5'));
  assert.deepEqual(port.writes,['etat\n','stop\n']);
});

test('erreur de port occupe ne conserve pas une connexion active',async()=>{
  const port=fakePort(); port.open=async()=>{throw new Error('port occupe');};
  const link=new UsbLink({requestPort:async()=>port});
  await assert.rejects(link.connect(),/port occupe/);
  assert.equal(link.active,false); assert.equal(link.port,null);
  assert.deepEqual(port.writes,[]);
});
