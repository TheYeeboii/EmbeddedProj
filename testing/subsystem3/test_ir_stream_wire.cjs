'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs');
const {IRStreamHistory,valid,viewport}=require('../../dashboard/ir-stream-model.js');
const W=require('../../dashboard/rover-wire.js');
/* Each line is one packed IR batch from the firmware, as hex. */
const unhex=h=>Uint8Array.from(h.match(/../g).map(x=>parseInt(x,16)));
const h=new IRStreamHistory();
for(const line of fs.readFileSync(process.argv[2],'utf8').trim().split('\n')){const m=W.decode(unhex(line));assert.equal(m.kind,'ir',line);const packet=m.data;assert(valid(packet),line);assert.equal(packet.period_ms,10);
const legacy=W.decode(unhex(line).slice(0,-1));
assert.equal(legacy.kind,'ir');assert.equal(legacy.data.period_ms,10);
assert(h.accept(packet));}
assert.equal(h.records.length,8);assert(h.records.some(r=>r.row[4]===1&&r.row[7]===17));
assert(h.records.some(r=>r.row[4]===1&&r.row[7]===32));
assert(h.records.every(r=>r.row[5]===0&&r.row[6]===0));
assert(viewport(h.records,null,5000).segments.some(s=>s.value===1));
assert(viewport(h.records,null,5000).segments.some(s=>s.value===null));
assert.deepEqual(IRStreamHistory.read(h.export()),h.records);
console.log('PASS: actual firmware packed IR batches -> black/white history while decoder blocked, no motion, plus truthful gaps and replay');
