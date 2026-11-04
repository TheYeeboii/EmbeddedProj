'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs');
const {IRStreamHistory,valid,viewport}=require('../dashboard/ir-stream-model.js');
const h=new IRStreamHistory();
for(const line of fs.readFileSync(process.argv[2],'utf8').trim().split('\n')){const packet=JSON.parse(line);assert(valid(packet),line);assert(h.accept(packet));}
assert.equal(h.records.length,8);assert(h.records.some(r=>r.row[4]===1&&r.row[7]===17));
assert(h.records.some(r=>r.row[4]===1&&r.row[7]===32));
assert(h.records.every(r=>r.row[5]===0&&r.row[6]===0));
assert(viewport(h.records,null,5000).segments.some(s=>s.value===1));
assert(viewport(h.records,null,5000).segments.some(s=>s.value===null));
assert.deepEqual(IRStreamHistory.read(h.export()),h.records);
console.log('PASS: actual firmware continuous JSON -> black/white history while decoder blocked, no motion, plus truthful gaps and replay');
