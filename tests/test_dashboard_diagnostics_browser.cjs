const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({headless:true,...(process.env.PLAYWRIGHT_CHROME_PATH ? {executablePath:process.env.PLAYWRIGHT_CHROME_PATH} : {})});
 const page=await browser.newPage({viewport:{width:1280,height:960}});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.route('https://**/*',route=>{
  if(route.request().url().includes('mqtt.min.js'))return route.fulfill({contentType:'application/javascript',body:`window.mockHandlers={};window.mqtt={connect:()=>({connected:true,on:(name,fn)=>window.mockHandlers[name]=name==='message'?(topic,payload)=>fn(topic,typeof payload==='string'?new TextEncoder().encode(payload):payload):fn,subscribe:()=>{},publish:()=>{}})};`});
  if(route.request().url().includes('lucide'))return route.fulfill({contentType:'application/javascript',body:'window.lucide={createIcons:()=>{}};'});
  return route.abort();
 });
 await page.goto(process.env.DASHBOARD_URL || 'http://127.0.0.1:8765/',{waitUntil:'networkidle'});
 await page.locator('[data-tab="buddy2"]').click();
 await page.locator('#steering-visual .tools-toggle').click();
 await page.evaluate(()=>{
  window.mockHandlers.connect();
  const make=(seq,direction)=>({v:1,type:'steering',boot:42,seq,ts_ms:seq*10,lag_ms:0,dropped:0,owner:2,phase:1,direction,requested:direction===1?[1000,4000]:direction===2?[4000,1000]:[3000,3000],applied:direction===1?[400,1600]:direction===2?[1600,400]:[1200,1200],encoder_ms:seq*10,distance:[100,100],speed:[500,500],faults:0,sample_ms:seq*10,ir:[3000,200,1000,0,500,200,1300,200,3000,0],has_input:1});
  for(let i=1;i<=18;i++)window.mockHandlers.message('rover/steering',JSON.stringify(make(i,i%3)));
 });
 assert.match(await page.locator('[data-steering="live"]').textContent(),/straight/);
 assert.equal(await page.locator('[data-steering="timeline"] rect').count(),18);
 await page.locator('[data-steering="pause"]').click();
 assert.match(await page.locator('[data-steering="notice"]').textContent(),/Frozen/);
 await page.locator('[data-steering="cursor"]').fill('0');
 assert.match(await page.locator('[data-steering="detail"]').textContent(),/event 1/);
 await page.locator('#steering-visual').screenshot({path:'/private/tmp/steering-desktop.png'});
 const downloadPromise=page.waitForEvent('download');
 await page.locator('[data-steering="export"]').click();
 const download=await downloadPromise;await download.saveAs('/private/tmp/steering-replay.json');
 await page.locator('[data-steering="clear"]').click();
 await page.locator('[data-steering="import"]').setInputFiles('/private/tmp/steering-replay.json');
 await page.waitForFunction(()=>document.querySelector('[data-steering="notice"]').textContent.includes('Imported replay'));
 assert.equal(await page.locator('[data-steering="timeline"] rect').count(),18);
 // Live ownership changes still apply while the historical view is frozen.
 await page.evaluate(()=>{
  const sample=steeringView.history.last;
  window.mockHandlers.message('rover/steering',JSON.stringify({...sample,type:'steering',seq:19,ts_ms:190,owner:1,has_input:0}));
  for(const [i,action] of ['turn_left','stop_turn_left','turn_right','stop_turn_right'].entries())
   window.mockHandlers.message('rover/actions',JSON.stringify({type:'action',seq:i+1,ts_ms:190+i,dropped:0,direction:i<2?'left':'right',source:'pwm',action}));
 });
 assert.equal(await page.locator('.steering-arrow:visible').count(),0);
 assert.match(await page.locator('[data-steering="live"]').textContent(),/Manual control/);
 assert.equal(await page.locator('[data-steering="timeline"] rect').count(),18);
 assert(!/Trying to turn|Stopped turning/.test(await page.locator('#action-log').textContent()));
 await page.evaluate(()=>{
  const sample=steeringView.history.last;
  window.mockHandlers.message('rover/steering',JSON.stringify({...sample,seq:20,ts_ms:200,owner:2,direction:2}));
 });
 assert.equal(await page.locator('.steering-arrow:visible').count(),2);
 assert.equal(await page.locator('#steering-visual').getAttribute('data-direction'),'right');
 assert.equal(await page.locator('[data-steering="timeline"] rect').count(),18);
 await page.locator('#left-speed').fill('-1');
 await page.locator('#drive-form button[type="submit"]').click();
 assert.match(await page.locator('#action-log').textContent(),/Dashboard rejected input/);
 await page.evaluate(()=>{
  const sample=steeringView.history.last;
  window.mockHandlers.message('rover/steering',JSON.stringify({...sample,seq:21,ts_ms:210,faults:1}));
  window.mockHandlers.message('rover/steering',JSON.stringify({...sample,seq:22,ts_ms:220,owner:0,direction:0,phase:9,faults:1}));
 });
 assert.match(await page.locator('#action-log').textContent(),/Encoder fault: left/);
 assert.match(await page.locator('#action-log').textContent(),/Stop commanded: all sensors black/);
 await page.evaluate(()=>window.mockHandlers.close());
 assert.equal(await page.locator('#steering-visual').getAttribute('data-direction'),'unknown');
 await page.setViewportSize({width:390,height:844});
 assert(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth));
 await page.locator('#steering-visual').screenshot({path:'/private/tmp/steering-mobile.png'});
 assert.deepEqual(errors,[]);
 console.log('PASS: browser arrows/timeline/freeze/inspection/export/import/disconnect; manual arrows collapse, autonomous restore, quiet turn logs, faults/stops/rejections; desktop/mobile, no page errors');
 await browser.close();
})().catch(e=>{console.error(e);process.exit(1);});
