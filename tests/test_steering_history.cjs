'use strict';
const assert = require('node:assert/strict');
const { SteeringHistory, valid } = require('../dashboard/steering-view.js');
const event = (seq, direction = 1, extra = {}) => ({
  v:1, type:'steering', boot:42, seq, ts_ms:seq*10, lag_ms:0, dropped:0,
  owner:2, phase:1, direction, requested:[1000,2000], applied:[400,800],
  encoder_ms:seq*10, distance:[10,20], speed:[50,60], faults:0,
  sample_ms:seq*10, ir:[100,3000,0,900,550,100,3100,100,3100,3], has_input:1, ...extra
});
const h = new SteeringHistory(3);
assert(h.accept(event(1),0));
assert(h.accept(event(2,2),10));
assert(h.accept(event(3,0),20));
assert.equal(h.records[1].data.ts_ms-h.records[0].data.ts_ms,10); // short pulse retained
assert.equal(h.live.direction,0);
assert(!h.accept(event(2))); assert(!h.accept(event(3))); // no invented reboot
assert(h.accept(event(3,0,{type:'steering_state'}),30));
assert.equal(h.records.length,3);
const frozen = h.records.slice();
assert(h.accept(event(5,1),40));
assert(h.records[2].gap); assert.equal(h.evicted,1); assert.equal(frozen[0].data.seq,1);
assert(h.accept(event(6,2,{dropped:1}),50)); assert(h.records[2].gap);
h.unavailable(); assert.equal(h.live,null);
assert(h.accept(event(6,2,{type:'steering_state',dropped:1}),60)); assert(h.records[2].gap);
assert(h.accept(event(1,1,{boot:43}),70));
assert(!h.accept(event(99,2,{boot:42}))); // delayed previous boot
assert(h.accept(event(2,1,{boot:43,lag_ms:1000}),80)); assert.equal(h.live,null);
assert(!valid({...event(1),ir:[1,2]}));
assert(!valid({...event(1),requested:[Infinity,0]}));
assert(!valid({...event(1),owner:'<img>'}));
assert(!valid({...event(1),ir:[0,0,1001,0,550,0,0,0,0,3]}));
const replay = new SteeringHistory();
replay.accept(event(1)); replay.accept(event(2,2)); replay.accept(event(3,0));
assert.deepEqual(SteeringHistory.read(replay.export()),replay.records);
assert.throws(()=>SteeringHistory.read('{"format":"wrong"}'));
assert.throws(()=>SteeringHistory.read('x'.repeat(2000001)));
assert.throws(()=>SteeringHistory.read(JSON.stringify({format:'rover-steering-v1',evicted:0,records:[{data:event(2),gap:false},{data:event(1),gap:false}]})));
const wrap = new SteeringHistory();
assert(wrap.accept(event(0xffffffff,1,{ts_ms:0xfffffff0,encoder_ms:0xfffffff0,sample_ms:0xfffffff0})));
assert(wrap.accept(event(0,2,{ts_ms:5})));
console.log('PASS: short pulses, causal records, snapshot restore, gaps, freeze, eviction, export/replay, reorder/duplicates, reboot, backlog, invalid data and wrap');
