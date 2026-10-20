// Transport Web Serial ; jamais de commande de mouvement a la connexion.
export class UsbLink {
  constructor(serialApi, callbacks = {}) {
    this.api = serialApi; this.cb = callbacks; this.port = null; this.writer = null;
    this.reader = null; this.active = false; this.queue = []; this.draining = null;
    this.closing = null; this.readTask = null;
  }
  async connect() {
    if (this.active || this.port) throw new Error('Une connexion est déjà ouverte.');
    if (!this.api) throw new Error('Utilise Chrome ou Edge pour la connexion USB.');
    this.port = await this.api.requestPort({filters:[{usbVendorId:0x303a},{usbVendorId:0x2886}]});
    try {
      await this.port.open({baudRate:115200, flowControl:'none', bufferSize:16384});
      await this.port.setSignals({dataTerminalReady:false, requestToSend:false});
      this.writer = this.port.writable.getWriter();
      this.active = true;
      this.cb.connection?.(true);
      this.readTask = this.readLines();
      await this.send('etat');
    } catch (error) {
      await this.disconnect(false).catch(() => {});
      throw error;
    }
  }
  send(command, priority = false) {
    if (!this.active || !this.writer) return Promise.reject(new Error('ESP déconnecté.'));
    if (!/^[a-z_]+(?: [-+0-9.]+)?$/.test(command)) return Promise.reject(new Error('Commande non autorisée.'));
    if (priority) this.cancelPending();
    const result = new Promise((resolve,reject) => this.queue.push({command,resolve,reject}));
    this.pump();
    return result;
  }
  stop() { return this.send('stop', true); }
  cancelPending() {
    for (const item of this.queue.splice(0)) item.reject(new Error('Commande annulée par STOP.'));
  }
  pump() {
    if (this.draining) return;
    this.draining = (async () => {
      while (this.active && this.queue.length) {
        const item = this.queue.shift();
        try {
          await this.writer.write(new TextEncoder().encode(item.command+'\n'));
          this.cb.sent?.(item.command); item.resolve();
        } catch (error) {
          item.reject(error); this.cancelPending();
          this.cb.error?.(error);
          queueMicrotask(() => this.disconnect(false).catch(() => {}));
          break;
        }
      }
    })().finally(() => { this.draining = null; if(this.active && this.queue.length) this.pump(); });
  }
  async readLines() {
    const decoder = new TextDecoder(); let buffer = '';
    this.reader = this.port.readable.getReader();
    try {
      while (this.active) {
        const {value,done} = await this.reader.read();
        if (done) break;
        buffer += decoder.decode(value,{stream:true});
        const lines = buffer.split(/\r?\n/); buffer = lines.pop();
        for (const line of lines) if(line.trim()) this.cb.line?.(line);
        if(buffer.length>8192) buffer='';
      }
    } catch(error) { if(this.active) this.cb.error?.(error); }
    finally {
      this.reader?.releaseLock(); this.reader=null;
      if(this.active) queueMicrotask(() => this.disconnect().catch(() => {}));
    }
  }
  disconnect(sendStop = true) {
    if(this.closing) return this.closing;
    this.closing = this.close(sendStop).finally(() => { this.closing=null; });
    return this.closing;
  }
  async close(sendStop) {
    if(sendStop && this.active) {
      let timeout;
      try { await Promise.race([this.stop(),new Promise((_,reject)=>{timeout=setTimeout(()=>reject(new Error('Arrêt USB non confirmé.')),700);})]); }
      catch(error) { this.cb.error?.(error); }
      finally { clearTimeout(timeout); }
    }
    this.active=false; this.cancelPending(); this.cb.connection?.(false);
    if(this.reader) await this.reader.cancel().catch(()=>{});
    await this.readTask?.catch(()=>{});
    if(this.writer) {
      if(this.draining) await this.writer.abort().catch(()=>{});
      await this.draining?.catch(()=>{});
      this.writer.releaseLock(); this.writer=null;
    }
    if(this.port) await this.port.close().catch(()=>{});
    this.port=null; this.readTask=null;
  }
}
