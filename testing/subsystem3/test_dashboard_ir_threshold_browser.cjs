const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({headless:true,executablePath:'/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'});
 const page=await browser.newPage({viewport:{width:1280,height:1000}});
 const errors=[];page.on('pageerror',error=>errors.push(error.message));
 await page.route('https://**/*',route=>{
  const url=route.request().url();
  if(url.includes('mqtt.min.js')) return route.fulfill({contentType:'application/javascript',body:'window.handlers={};window.sent=[];window.mqtt={connect:()=>({connected:true,on:(k,v)=>window.handlers[k]=v,subscribe:()=>{},publish:(t,b)=>window.sent.push([...b])})};'});
  if(url.includes('lucide')) return route.fulfill({contentType:'application/javascript',body:'window.lucide={createIcons:()=>{}}'});
  return route.abort();
 });
 await page.goto(process.env.DASHBOARD_URL || 'http://127.0.0.1:8765');
 await page.locator('[data-tab="buddy3"]').click();
 const ids=[null,'ir-threshold-editor-left','ir-threshold-editor-right'];
 assert.equal(await page.locator('#ir-threshold-editor').count(),0); // barcode sensor is digital: no editor
 const report=async(sensor,low,high,raw,inverted=false,enabled=true)=>page.evaluate(({sensor,low,high,raw,inverted,enabled})=>{
  const writer=new RoverWire.Writer(11);
  writer.put(0xc1,8).put(1000,32).put(low,12).put(high,12).put(raw,12).put(enabled?1:0,1).put(inverted?1:0,1).put(1,1).put(sensor,2);
  window.handlers.message('rover/debug',writer.bytes);
 },{sensor,low,high,raw,inverted,enabled});
 for(let sensor=0;sensor<3;sensor++) await report(sensor,0,0,200+sensor*100,false,false);
 await page.waitForTimeout(300);
 for(let sensor=1;sensor<3;sensor++) {
  const form=page.locator('#'+ids[sensor]);
  assert(await form.isVisible());
  assert(await form.locator('[data-threshold="apply"]').isEnabled());
  await form.locator('[data-threshold="low-number"]').fill(String(220+sensor*100));
  await form.locator('[data-threshold="high-number"]').fill(String(260+sensor*100));
 }
 // Applying the left editor does not submit or confirm either other editor.
 const left=page.locator('#'+ids[1]);
 await left.locator('[data-threshold="apply"]').click();
 const bytes=await page.evaluate(()=>window.sent[0]);
 assert.equal(bytes[0]>>4,0);
 assert.equal((bytes[0]>>2)&3,1);
 await report(0,320,360,250);
 assert.match(await left.locator('[data-threshold="status"]').innerText(),/Waiting/);
 await report(1,320,360,350);
 assert.match(await left.locator('[data-threshold="status"]').innerText(),/Confirmed/);
 assert.match(await left.locator('[data-threshold="live"]').innerText(),/350.*BLACK/);
 assert.equal(await page.evaluate(()=>window.sent.length),1);
 await left.locator('[data-threshold="low-range"]').focus();
 await page.keyboard.press('ArrowRight');
 assert.equal(await left.locator('[data-threshold="low-number"]').inputValue(),'321');
 assert.equal(await page.locator('#'+ids[2]+' [data-threshold="low-number"]').inputValue(),'420');
 assert.equal(await page.evaluate(()=>window.sent.length),1);
 const slider=left.locator('[data-threshold="high-range"]');
 const bounds=await slider.boundingBox();
 await page.mouse.move(bounds.x+360/4095*bounds.width,bounds.y+bounds.height/2);
 await page.mouse.down();await page.mouse.move(bounds.x+bounds.width*.5,bounds.y+bounds.height/2);await page.mouse.up();
 assert(Number(await left.locator('[data-threshold="high-number"]').inputValue())>1000);
 await left.locator('[data-threshold="invert"]').check();
 await page.locator('.ir-threshold-panels').screenshot({path:'/private/tmp/ir-three-desktop.png'});
 await page.setViewportSize({width:390,height:844});
 assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
 await page.locator('.ir-threshold-panels').screenshot({path:'/private/tmp/ir-three-mobile.png'});
 // Each status has independent freshness.
 await report(2,420,460,450);
 await page.waitForTimeout(1100);
 await report(2,420,460,450);
 await page.waitForTimeout(650);
 assert(await page.locator('#'+ids[2]+' [data-threshold="apply"]').isEnabled());
 assert(await left.locator('[data-threshold="apply"]').isDisabled());
 assert.match(await left.locator('[data-threshold="live"]').innerText(), /Delayed.*last Raw 350/i);
 assert.match(await left.locator('[data-threshold="status"]').innerText(), /Last report.*old/);
 // A queued report cannot confirm a pending setting or become fresh on arrival.
 await report(1,320,360,350);
 await left.locator('[data-threshold="apply"]').click();
 await page.evaluate(()=>{
   const pending = irThresholdViews[0].pending;
   const status = new RoverWire.Writer(13).put(0xc1,8).put(2000,32)
     .put(pending.low,12).put(pending.high,12).put(350,12).put(1,1)
     .put(pending.inverted ? 1 : 0,1).put(1,1)
     .put(1,2).put(0,1).put(0,5).put(0,1).put(3000,16).bytes;
   window.handlers.message('rover/debug',status);
 });
 assert(await left.locator('[data-threshold="apply"]').isDisabled());
 assert.match(await left.locator('[data-threshold="live"]').innerText(), /Delayed.*350/);
 assert(!/Confirmed by/.test(await left.locator('[data-threshold="status"]').innerText()));
 // Let the request expire while reports are delayed.
 await page.evaluate(()=>{irThresholdViews[0].pending.at = Date.now()-5000;irThresholdViews[0].refresh();});
 assert.equal(await page.evaluate(()=>irThresholdViews[0].pending),null);
 await report(1,320,360,350);
 assert(await left.locator('[data-threshold="apply"]').isEnabled());
 assert(!/Delayed/.test(await left.locator('[data-threshold="live"]').innerText()));
 // New firmware reports the digital mode and its hardware pin/polarity.
 await page.evaluate(()=>{
  const status = new RoverWire.Writer(11).put(0xc1,8).put(2000,32)
    .put(0,12).put(0,12).put(1,12).put(0,1).put(0,1).put(1,1)
    .put(0,2).put(1,1).put(28,5).put(1,1).bytes;
  window.handlers.message('rover/debug',status);
 });
 await report(1,320,360,350);
 await page.waitForFunction(()=>!document.querySelector('#ir-threshold-editor-left [data-threshold="apply"]').disabled);
 assert(await page.locator('[data-ir-levels="barcode"] [data-ir-apply]').isDisabled());
 assert(await page.locator('[data-ir-levels="left"] [data-ir-apply]').isEnabled());
 assert(await left.locator('[data-threshold="low-number"]').isEnabled());
 assert(await left.locator('[data-threshold="apply"]').isEnabled());
 await page.evaluate(()=>{
  const rows=[0,1,1,0].map((value,index)=>[index+1,index*5,value,value*1000,value,0,0,1,0,1,0]);
  window.handlers.message('rover/ir',new TextEncoder().encode(JSON.stringify({v:1,type:'ir_stream',boot:99,ts_ms:15,lag_ms:0,dropped:0,period_ms:5,white:140,black:210,rows})));
 });
 await page.waitForTimeout(100);
 assert(!(await page.locator('[data-ir="signal"]').isVisible()));
 assert.match(await page.locator('[data-ir="detail"]').innerText(),/Digital GPIO/);
 assert(await page.locator('[data-ir="strip"] [data-color="black"]').count()>0);
 await page.locator('.ir-threshold-panels').screenshot({path:'/private/tmp/ir-digital-editor.png'});

 assert.deepEqual(errors,[]);
 console.log('PASS: two front-sensor editors (no barcode editor), independent drafts/commands/confirmation/live data/staleness, drag, keyboard safety, polarity and mobile layout');
 await browser.close();
})().catch(error=>{console.error(error);process.exit(1)});
