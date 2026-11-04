'use strict';
const assert = require('node:assert/strict');
const RoverActionLog = require('../dashboard/action-log.js');

class Element {
  constructor(document) { this.ownerDocument=document; this.children=[]; this.dataset={}; this.className=''; this.textContent=''; this.scrollTop=0; this.clientHeight=100; }
  append(child) { child.parent=this; this.children.push(child); }
  replaceChildren() { this.children=[]; }
  remove() { if(this.parent) this.parent.children=this.parent.children.filter(e=>e!==this); }
  querySelector(cls) { return this.children.find(c=>'.'+c.className===cls)||null; }
  get firstElementChild() { return this.children[0]; }
  get childElementCount() { return this.children.length; }
  get scrollHeight() { return this.children.length*18; }
}
const doc={createElement:()=>new Element(doc)};
const log=new Element(doc), badge=new Element(doc);
const view=new RoverActionLog(log,badge,{limit:3,clock:()=>'12:00:00'});
const message=(action,seq,direction,type='action',dropped=0)=>({
  type,action,seq,ts_ms:seq*10,direction,dropped,source:'pwm',left_pwm:1000,right_pwm:2000
});
// Routine steering never adds text rows, even on rapid direction changes.
assert(view.handle(message('turn_left',1,'left')));
assert(view.handle(message('stop_turn_left',2,'right')));
assert(view.handle(message('turn_right',3,'right')));
assert(view.handle(message('stop_turn_right',4,'none')));
assert.equal(log.childElementCount,0);
assert.equal(badge.textContent,'Action stream connected');
view.handle(message(null,4,'none','action_state'));
assert.equal(log.childElementCount,0);
assert(view.handle(message('turn_right',10,'right','action',2)));
assert.equal(log.childElementCount,1);assert.equal(log.children[0].dataset.action,'gap');
view.clear();assert(log.querySelector('.console-empty'));

const diagnostic=(seq,owner=2,faults=0,extra={})=>({type:'steering',boot:42,seq,ts_ms:seq*10,owner,faults,phase:1,...extra});
assert(view.diagnostic(diagnostic(1)));
assert(log.children[0].textContent.includes('Control owner: Tracking'));
const count=log.childElementCount;
assert(view.diagnostic(diagnostic(2)));
assert(view.diagnostic(diagnostic(3)));
assert.equal(log.childElementCount,count); // No text on each correction.
assert(view.diagnostic(diagnostic(4,1)));
assert(log.children[1].textContent.includes('Tracking → Manual'));
assert(view.diagnostic(diagnostic(5,0,0,{phase:9})));
assert(log.children[2].textContent.includes('Stop commanded: all sensors black'));
assert(view.diagnostic(diagnostic(6,0,1)));
assert.equal(log.childElementCount,3); // Bounded log retains newest events.
assert(log.children[2].textContent.includes('Encoder fault: left'));
assert(view.diagnostic(diagnostic(7,0,1)));
assert.equal(log.childElementCount,3);
assert(view.diagnostic(diagnostic(8,0,0)));
assert(log.children[2].textContent.includes('Encoder feedback restored'));
const before=log.children.map(x=>x.textContent);
view.diagnostic(diagnostic(9,5),true);
assert.deepEqual(log.children.map(x=>x.textContent),before);
assert.equal(badge.textContent,'Control: Avoidance');
view.unavailable();assert.equal(badge.textContent,'Diagnostics unavailable');
view.diagnostic(diagnostic(9,5,0,{type:'steering_state'}));
assert.equal(badge.textContent,'Control: Avoidance');
assert.deepEqual(log.children.map(x=>x.textContent),before);
assert(!view.diagnostic(diagnostic(8,2)));
assert(!view.diagnostic(diagnostic(10,'bad')));
assert(!view.diagnostic(diagnostic(10,2,4)));
assert(view.diagnostic(diagnostic(1,2,0,{boot:43})));
assert(log.children[2].textContent.includes('Rover restarted'));
assert(view.diagnostic(diagnostic(2,2,0,{boot:43,lag_ms:1000})));
assert.equal(badge.textContent,'Buffered diagnostics');
assert.equal(view.handle(message('<img src=x onerror=alert(1)>',11,'left')),false);
assert.equal(view.handle({...message('turn_left',11,'left'),seq:'11'}),false);
assert.equal(view.handle({...message('turn_left',11,'left'),dropped:-1}),false);
view.handle(message('turn_left',1,'left'));assert.equal(view.lastDropped,0);
view.clear();assert(log.querySelector('.console-empty'));
console.log('PASS: quiet corrections, retained stops/faults/handovers/gaps, deduplication, pause/clear, snapshot restore, bounded history and malformed input');
