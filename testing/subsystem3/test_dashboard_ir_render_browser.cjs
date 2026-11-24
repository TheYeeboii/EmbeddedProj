const {chromium}=require('playwright');
const assert=require('node:assert/strict');
(async()=>{
 const browser=await chromium.launch({headless:true,executablePath:'/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'});
 const page=await browser.newPage({viewport:{width:1280,height:960}});
 await page.route('https://**/*',route=>{
  const url=route.request().url();
  if(url.includes('mqtt.min.js')) return route.fulfill({contentType:'application/javascript',body:'window.mqtt={connect:()=>({connected:true,on:()=>{},subscribe:()=>{},publish:()=>{}})};'});
  if(url.includes('lucide')) return route.fulfill({contentType:'application/javascript',body:'window.lucide={createIcons:()=>{}}'});
  return route.abort();
 });
 await page.goto(process.env.DASHBOARD_URL || 'http://127.0.0.1:8765');
 await page.locator('[data-tab="buddy3"]').click();
 const result=await page.evaluate(async()=>{
  let sequence=0;
  function packet() {
   const rows=Array.from({length:4},()=>{
    const seq=++sequence, black=seq%2;
    return [seq,seq*5,black,black*1000,black,0,0,1,0,1,0];
   });
   return {v:1,type:'ir_stream',boot:123,ts_ms:rows.at(-1)[1],lag_ms:0,dropped:0,period_ms:5,white:140,black:210,rows};
  }
  for(let i=0;i<1500;i++) irStreamView.history.accept(packet());
  irStreamView.render();
  let renders=0,total=0,max=0;
  const original=irStreamView.render.bind(irStreamView);
  irStreamView.render=()=>{const start=performance.now();original();const duration=performance.now()-start;renders++;total+=duration;max=Math.max(max,duration);};
  const run=duration=>new Promise(resolve=>{
   const timer=setInterval(()=>irStreamView.handle(packet()),20);
   setTimeout(()=>{clearInterval(timer);resolve();},duration);
  });
  await run(3000);
  await new Promise(resolve=>setTimeout(resolve,150));
  const visible={renders,renderMs:Math.round(total),maxRenderMs:Math.round(max)};
  document.querySelector('[data-tab="buddy4"]').click();
  renders=0;total=0;max=0;
  await run(1000);
  await new Promise(resolve=>setTimeout(resolve,150));
  return {visible,hidden:{renders,renderMs:Math.round(total)},retained:irStreamView.history.records.length,received:sequence,evicted:irStreamView.history.evicted};
 });
 assert(result.visible.renders > 0 && result.visible.renders <= 35);
 assert.equal(result.hidden.renders, 0);
 assert.equal(result.retained, 6000);
 assert.equal(result.received - result.retained, result.evicted);
 console.log('PASS: bounded live redraws, no hidden-tab redraws, full-history retention and eviction accounting', JSON.stringify(result));
 await browser.close();
})().catch(e=>{console.error(e);process.exit(1)});
