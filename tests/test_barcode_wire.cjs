'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs');
const {BarcodeHistory,valid}=require('../dashboard/barcode-model.js');
const h=new BarcodeHistory();
for(const line of fs.readFileSync(process.argv[2],'utf8').trim().split('\n')){
 const packet=JSON.parse(line);assert(valid(packet),`Firmware packet fails schema: ${line}`);assert(h.accept(packet),`Firmware order rejected: ${line}`);
}
assert.equal(h.frames.length,3);
assert.equal(h.frames[0].end.symbol,70);assert.equal(h.frames[0].end.reversed,0);
assert.equal(h.frames[1].end.symbol,70);assert.equal(h.frames[1].end.reversed,1);
assert.equal(h.frames[2].end.state,5);assert(h.frames[2].end.count>0);
for(const f of h.frames){assert.equal(f.edges.filter(Boolean).length,f.end.count);assert.equal(f.samples.filter(Boolean).length,f.end.samples);assert(!f.gap);}
assert.deepEqual(BarcodeHistory.read(h.export()),h.frames);
console.log('PASS: real firmware JSON -> dashboard model, both orientations, rejected partial trace and lossless replay');
