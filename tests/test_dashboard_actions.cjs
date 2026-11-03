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
assert(view.handle(message('turn_left',1,'left')));
assert(log.children[0].textContent.includes('Trying to turn left'));
assert.equal(badge.textContent,'Left turn commanded');
assert(view.handle(message('stop_turn_left',2,'right')));
assert(view.handle(message('turn_right',3,'right')));
assert(log.children[1].textContent.includes('Stopped turning left'));
assert(log.children[2].textContent.includes('Trying to turn right'));
view.handle(message('stop_turn_right',4,'none'));
assert.equal(log.childElementCount,3);
assert(log.children[2].textContent.includes('Stopped turning right'));
const count=log.childElementCount;
view.handle(message(null,4,'none','action_state'));
assert.equal(log.childElementCount,count); /* Snapshots never invent action rows. */
view.handle(message('turn_left',5,'left'),true);
assert.equal(log.childElementCount,count);
assert.equal(badge.textContent,'Left turn commanded');
view.clear(); assert(log.querySelector('.console-empty'));
view.handle(message('stop_turn_left',6,'none'));
assert.equal(log.childElementCount,1);
assert(view.handle(message('turn_right',10,'right','action',2)));
assert(log.children.some(e=>e.dataset.action==='gap'));
assert.equal(view.handle(message('<img src=x onerror=alert(1)>',11,'left')),false);
assert.equal(view.handle({...message('turn_left',11,'left'),seq:'11'}),false);
assert.equal(view.handle({...message('turn_left',11,'left'),dropped:-1}),false);
view.unavailable(); assert.equal(badge.textContent,'Command state unavailable');
view.handle(message('turn_left',1,'left')); /* Reboot starts a new sequence. */
assert.equal(badge.textContent,'Left turn commanded');
assert.equal(view.lastDropped,0);
console.log('PASS: dashboard start/stop labels, command badge, snapshots, pause/clear, bounded history, gaps, reboot, malformed-message rejection');
