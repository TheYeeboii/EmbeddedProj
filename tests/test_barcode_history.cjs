'use strict';
const assert=require('node:assert/strict');
const {BarcodeHistory,valid,reference,estimate}=require('../dashboard/barcode-model.js');
const envelope=(type,seq,extra={})=>({v:1,type,boot:1,seq,scan:1,ts_ms:seq*10,dropped:0,lag_ms:0,...extra});
const start=(seq=1,extra={})=>envelope('barcode_start',seq,{leading:3000,config:[2000,2500,0,400,600,300,-600,1500,650,3870,3870],...extra});
const edge=(seq,index,width=500)=>envelope('barcode_edges',seq,{rows:[[index,seq*10,width,10,10]]});
const end=(seq,extra={})=>envelope('barcode_end',seq,{state:4,count:29,samples:0,sample_drops:0,partial:0,blocked:0,reason:1,format:0,symbol:70,reversed:0,leading:3000,trailing:3000,narrow:500,ratio:300,max_gap:50,...extra});
for(let c=65;c<=90;c++)for(const ratio of [2,3]){
 const forward=reference(String.fromCharCode(c),false,ratio),backward=reference(String.fromCharCode(c),true,ratio);
 assert.equal(forward.length,29);assert.deepEqual(backward,forward.toReversed());
 for(const offset of [0,10,20])assert.equal(forward.slice(offset,offset+9).filter(x=>x===ratio).length,3);
}
// Independent printed-character fixtures exercise scan assembly/render inputs,
// rather than merely round-tripping the production reference table.
for(const [letter,pattern] of Object.entries({F:'001011000',Y:'110010000',A:'100001001',Z:'011010000'}))
 for(const scale of [0.5,0.75,1,1.5,2])for(const reversed of [false,true]) {
  let widths=[];
  for(const [i,p] of ['010010100',pattern,'010010100'].entries()) {
   if(i)widths.push(100*scale);
   widths.push(...[...p].map(c=>(c==='1'?300:100)*scale));
  }
  if(reversed)widths.reverse();
  const history=new BarcodeHistory();history.accept(start());
  widths.forEach((w,i)=>assert(history.accept(edge(i+2,i,w))));
  assert(history.accept(end(31,{symbol:letter.charCodeAt(0),reversed:Number(reversed)})));
  assert.deepEqual(history.frames[0].edges.map(r=>r[2]),widths);
  assert(Math.abs(estimate(history.frames[0]).narrow-scale)<0.001);
  assert.deepEqual(reference(letter,reversed).map(n=>n*100*scale),widths);
 }
const h=new BarcodeHistory();assert(h.accept(start()));
const widths=reference('F').map(x=>x*500);
for(let i=0;i<29;i++)assert(h.accept(edge(i+2,i,widths[i])));
assert(h.accept(end(31)));assert.equal(h.frames[0].end.symbol,70);
assert(estimate(h.frames[0]).narrow===5);assert.equal(h.frames[0].gap,false);
assert.deepEqual(BarcodeHistory.read(h.export()),h.frames);
assert(!h.accept(edge(30,1)));assert(!h.accept(end(31)));
assert(h.accept(start(33,{scan:2})));assert(h.frames[1].gap);
assert(h.accept(envelope('barcode_samples',34,{scan:2,rows:[[2,100,2500,1000,10,10,500,1]]})));
assert.equal(h.frames[1].samples[0],null);assert(h.frames[1].samples[2]);
assert(h.accept(edge(35,0,500))); // independent scan 1 conflicting update
assert(h.accept(edge(36,0,600)));assert(h.frames[0].conflict);
h.unavailable();assert.equal(h.state,null);
assert(h.accept(start(37,{scan:3,lag_ms:1000})));assert.equal(h.state,null);
assert(h.accept(start(1,{boot:2})));assert(!h.accept(start(99,{boot:1})));
assert(!valid(envelope('barcode_samples',1,{rows:[[512,1,1,1,1,1,1,1]]})));
assert(!valid(envelope('barcode_samples',1,{rows:[[0,1,4096,1,1,1,1,1]]})));
assert(!valid(edge(1,0,-1)));assert(!valid({...end(1),symbol:1}));
assert(!valid({...start(),config:[]}));
assert.throws(()=>BarcodeHistory.read('x'.repeat(4000001)));
assert.throws(()=>BarcodeHistory.read(JSON.stringify({format:'rover-barcode-v1',evicted:0,scans:[{...h.frames[0],edges:[]}]})));
for(let i=2;i<=25;i++)h.accept(start(i,{boot:2,scan:i}));assert.equal(h.frames.length,20);assert(h.evicted>0);
console.log('PASS: reference A-Z/orientation/ratio, scan assembly, unknown samples, scale, gaps, conflict/reorder/reboot, invalid input, export/import and bounded scan history');
