(function(root) {
  'use strict';
  const integer=(n,min,max)=>Number.isInteger(n)&&n>=min&&n<=max;
  const u32=n=>integer(n,0,0xffffffff), signed=n=>integer(n,-2147483647,2147483647);
  const gaps=[null,'missing_samples','sampling_gap','boot'];
  const clone=x=>JSON.parse(JSON.stringify(x));
  function validRow(r) {
    return Array.isArray(r)&&r.length===11&&u32(r[0])&&u32(r[1])&&
      integer(r[2],0,4095)&&integer(r[3],0,1000)&&integer(r[4],0,1)&&
      signed(r[5])&&signed(r[6])&&integer(r[7],0,63)&&
      integer(r[8],0,4095)&&integer(r[9],0,4095)&&integer(r[10],0,1);
  }
  function valid(d) {
    return d&&d.v===1&&d.type==='ir_stream'&&u32(d.boot)&&u32(d.ts_ms)&&u32(d.lag_ms)&&u32(d.dropped)&&
      d.period_ms===10&&d.white===140&&d.black===210&&Array.isArray(d.rows)&&d.rows.length>0&&d.rows.length<=4&&
      d.rows.every((r,i)=>validRow(r)&&(!i||((r[0]-d.rows[i-1][0])>>>0)===1))&&d.ts_ms===d.rows[d.rows.length-1][1];
  }
  class IRStreamHistory {
    constructor(limit=6000) {
      this.limit=limit;this.records=[];this.last=null;this.boot=null;this.retired=[];
      this.live=null;this.receivedAt=0;this.evicted=0;this.dropped=0;
    }
    unavailable(){this.live=null;}
    accept(d,now=Date.now()) {
      if(!valid(d)||this.retired.includes(d.boot))return false;
      let reboot=this.boot!==null&&this.boot!==d.boot;
      if(reboot){this.retired.push(this.boot);this.retired=this.retired.slice(-8);this.last=null;}
      this.boot=d.boot;let accepted=0;
      for(const input of d.rows) {
        let gap=reboot?'boot':null;
        if(this.last) {
          const advance=(input[0]-this.last.row[0])>>>0;
          if(!advance||advance>=0x80000000)continue;
          const elapsed=(input[1]-this.last.row[1])>>>0;
          if(elapsed>=0x80000000)continue; // Old clock value, not a firmware reboot.
          if(advance>1)gap='missing_samples';
          else if(elapsed>20)gap='sampling_gap';
        }
        const record={boot:d.boot,row:input.slice(),gap,dropped:d.dropped};
        this.records.push(record);this.last=record;reboot=false;++accepted;
      }
      if(!accepted)return false;
      while(this.records.length>this.limit){this.records.shift();++this.evicted;}
      this.dropped=d.dropped;this.receivedAt=now;
      this.live=d.lag_ms>500?null:this.last;
      return true;
    }
    export(records=this.records){return JSON.stringify({format:'rover-ir-stream-v1',evicted:this.evicted,records},null,2);}
    static read(text) {
      if(typeof text!=='string'||text.length>4000000)throw new Error('IR recording exceeds 4 MB');
      const file=JSON.parse(text);
      if(file.format!=='rover-ir-stream-v1'||!u32(file.evicted)||!Array.isArray(file.records)||file.records.length>6000)
        throw new Error('Not a valid continuous IR recording');
      const check=new IRStreamHistory();
      for(const r of file.records) {
        if(!r||!u32(r.boot)||!u32(r.dropped)||!validRow(r.row)||!gaps.includes(r.gap))throw new Error('Invalid IR observation');
        if(!check.accept({v:1,type:'ir_stream',boot:r.boot,ts_ms:r.row[1],lag_ms:0,dropped:r.dropped,period_ms:10,white:140,black:210,rows:[r.row]}))
          throw new Error('IR observations are duplicated or out of order');
        if(check.records.length>1&&check.last.gap&&r.gap!==check.last.gap)throw new Error('IR recording conceals a data gap');
      }
      return clone(file.records);
    }
  }
  function viewport(records,index,windowMs) {
    if(!records.length)return null;
    index=Math.max(0,Math.min(index??records.length-1,records.length-1));
    const selected=records[index],end=selected.row[1];
    const points=records.slice(0,index+1).filter(r=>r.boot===selected.boot).map(r=>({...r,time:-((end-r.row[1])>>>0)}));
    const start=Math.max(-windowMs,points[0].time),span=Math.max(1,-start),segments=[];
    for(let i=0;i<points.length-1;i++) {
      const a=Math.max(start,points[i].time),b=points[i+1].time;
      if(b<=a)continue;
      const reason=points[i+1].gap,value=reason?null:points[i].row[4],last=segments[segments.length-1];
      if(last&&last.value===value&&last.reason===reason&&last.to===a)last.to=b;
      else segments.push({from:a,to:b,value,reason});
    }
    return {start,span,segments,points:points.filter(p=>p.time>=start),selected};
  }
  root.RoverIRStream={IRStreamHistory,valid,viewport};
  if(typeof module!=='undefined'&&module.exports)module.exports=root.RoverIRStream;
})(globalThis);
