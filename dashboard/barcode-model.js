(function(root) {
  'use strict';
  const types = ['barcode_start','barcode_edges','barcode_samples','barcode_progress','barcode_end','barcode_state'];
  const states = ['Disabled / unconfigured','Waiting for leading quiet zone','Capturing','Checking trailing quiet zone','Accepted','Rejected / interrupted','Suspended by motion or tracking state'];
  const reasons = ['none','valid','invalid format','interrupted','reverse motion','sample spacing too large','stalled','capture limit'];
  const formats = ['ok','element count','element widths','framing markers','unsupported symbol','quiet zone'];
  const patterns = [0x109,0x049,0x148,0x019,0x118,0x058,0x00d,0x10c,0x04c,0x01c,0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,0x181,0x0c1,0x1c0,0x091,0x190,0x0d0];
  const integer=(n,min,max)=>Number.isInteger(n)&&n>=min&&n<=max;
  const u32=n=>integer(n,0,0xffffffff), signed=n=>integer(n,-2147483647,2147483647);
  const array=(row,n)=>Array.isArray(row)&&row.length===n&&row.every(x=>integer(x,-2147483647,0xffffffff));
  function edge(row) { return array(row,5)&&integer(row[0],0,28)&&u32(row[1])&&row[2]>0&&signed(row[2])&&signed(row[3])&&signed(row[4]); }
  function sample(row) { return array(row,8)&&integer(row[0],0,511)&&u32(row[1])&&integer(row[2],0,4095)&&integer(row[3],0,1000)&&integer(row[7],0,1)&&[4,5,6].every(i=>signed(row[i])); }
  function valid(d) {
    if (!d||d.v!==1||!types.includes(d.type)||!['boot','seq','scan','ts_ms','dropped','lag_ms'].every(k=>u32(d[k]))) return false;
    if(d.type==='barcode_start') return array(d.config,11)&&integer(d.config[0],0,4095)&&integer(d.config[1],0,4095)&&
      integer(d.config[2],0,1)&&integer(d.config[3],0,1000)&&integer(d.config[4],0,1000)&&d.config[3]<d.config[4]&&d.config[5]>0&&signed(d.leading)&&d.leading>=0;
    if(d.type==='barcode_edges'||d.type==='barcode_samples') return Array.isArray(d.rows)&&d.rows.length>0&&d.rows.length<=4&&
      d.rows.every(d.type==='barcode_edges'?edge:sample)&&new Set(d.rows.map(r=>r[0])).size===d.rows.length;
    if(!integer(d.state,0,6)||!integer(d.count,0,29)||!integer(d.samples,0,512)||!u32(d.sample_drops)||!signed(d.partial)||!integer(d.blocked,0,31)) return false;
    if(d.type==='barcode_state'&&d.config!==undefined&&(!array(d.config,11)||!d.config.every(signed)||
       (d.state!==0&&(d.config[5]<=0||d.config[3]>=d.config[4]))))return false;
    if(d.type==='barcode_end') return integer(d.reason,1,7)&&integer(d.format,0,5)&&integer(d.reversed,0,1)&&
      ['leading','trailing','narrow','ratio','max_gap'].every(k=>signed(d[k]))&&
      (d.reason===1 ? (d.state===4&&integer(d.symbol,65,90)&&d.count===29) : (d.state===5&&d.symbol===0));
    return true;
  }
  const clone=x=>JSON.parse(JSON.stringify(x));
  function reference(letter, reversed=false, ratio=3) {
    if(!/^[A-Z]$/.test(letter)||![2,3].includes(ratio)) return [];
    const out=[];
    [0x094,patterns[letter.charCodeAt(0)-65],0x094].forEach((p,g)=>{
      if(g)out.push(1);
      for(let i=8;i>=0;i--)out.push(p&(1<<i)?ratio:1);
    });
    return reversed?out.reverse():out;
  }
  function newFrame(d) {
    return {boot:d.boot,id:d.scan,updated:d.ts_ms,lag_ms:d.lag_ms,meta:null,progress:null,end:null,
      edges:Array(29).fill(null),samples:Array(512).fill(null),gap:false,conflict:false};
  }
  function estimate(frame) {
    if(frame.edges.slice(0,9).some(r=>!r))return null;
    const sorted=frame.edges.slice(0,9).map(r=>r[2]/100).sort((a,b)=>a-b);
    return {narrow:sorted.slice(0,6).reduce((a,b)=>a+b,0)/6,wide:sorted.slice(6).reduce((a,b)=>a+b,0)/3};
  }
  class BarcodeHistory {
    constructor() { this.frames=[];this.last=null;this.retired=[];this.pendingGap=false;this.state=0;this.receivedAt=0;this.evicted=0; }
    unavailable(){this.pendingGap=true;this.state=null;}
    accept(input,now=Date.now()) {
      if(!valid(input))return false;
      const d=clone(input);let gap=this.pendingGap||(!this.last&&d.dropped>0);
      if(this.last&&this.last.boot!==d.boot) {
        if(this.retired.includes(d.boot))return false;
        this.retired.push(this.last.boot);this.retired=this.retired.slice(-8);gap=true;
        this.frames.filter(f=>!f.end).forEach(f=>f.gap=true);
      } else if(this.last) {
        const advance=(d.seq-this.last.seq)>>>0;
        if(!advance||advance>=0x80000000)return false;
        if(advance>1||d.dropped!==this.last.dropped)gap=true;
      }
      if(gap)this.frames.filter(f=>!f.end).forEach(f=>f.gap=true);
      this.pendingGap=false;this.last=d;this.receivedAt=now;
      if(d.type==='barcode_state'){this.state=d.lag_ms>500?null:d.state;return true;}
      let f=this.frames.find(f=>f.boot===d.boot&&f.id===d.scan);
      if(!f){f=newFrame(d);this.frames.push(f);if(this.frames.length>20){this.frames.shift();++this.evicted;}}
      if(gap)f.gap=true;
      f.updated=d.ts_ms;f.lag_ms=d.lag_ms;
      if(d.type==='barcode_start')f.meta=d;
      if(d.type==='barcode_edges'||d.type==='barcode_samples') {
        const target=d.type==='barcode_edges'?f.edges:f.samples;
        for(const row of d.rows) {
          if(target[row[0]]&&JSON.stringify(target[row[0]])!==JSON.stringify(row)){f.conflict=true;f.gap=true;}
          else target[row[0]]=row;
        }
      }
      if(d.type==='barcode_progress')f.progress=d;
      if(d.type==='barcode_end')f.end=d;
      this.state=d.lag_ms>500?null:d.type==='barcode_start'?2:d.state??this.state;
      return true;
    }
    export(frames=this.frames){return JSON.stringify({format:'rover-barcode-v1',evicted:this.evicted,scans:frames},null,2);}
    static read(text) {
      if(typeof text!=='string'||text.length>4000000)throw new Error('Scan export exceeds 4 MB');
      const file=JSON.parse(text);
      if(file.format!=='rover-barcode-v1'||!u32(file.evicted)||!Array.isArray(file.scans)||file.scans.length>20)throw new Error('Invalid barcode export');
      const ids=new Set();
      for(const f of file.scans) {
        if(!f||!u32(f.boot)||!u32(f.id)||!u32(f.updated)||!u32(f.lag_ms)||typeof f.gap!=='boolean'||typeof f.conflict!=='boolean'||
           !Array.isArray(f.edges)||f.edges.length!==29||!Array.isArray(f.samples)||f.samples.length!==512||
           !f.edges.every((r,i)=>r===null||(edge(r)&&r[0]===i))||!f.samples.every((r,i)=>r===null||(sample(r)&&r[0]===i)))throw new Error('Invalid scan measurements');
        for(const [key,type] of [['meta','barcode_start'],['progress','barcode_progress'],['end','barcode_end']])
          if(f[key]!==null&&(!valid(f[key])||f[key].type!==type||f[key].boot!==f.boot||f[key].scan!==f.id))throw new Error('Invalid scan metadata');
        const id=`${f.boot}:${f.id}`;if(ids.has(id))throw new Error('Duplicate scan');ids.add(id);
      }
      return clone(file.scans);
    }
  }
  root.RoverBarcode={BarcodeHistory,valid,reference,estimate,states,reasons,formats};
  if(typeof module!=='undefined'&&module.exports)module.exports=root.RoverBarcode;
})(globalThis);
