(function(root){
  'use strict';
  const {BarcodeHistory,reference,estimate,states,reasons,formats}=root.RoverBarcode;
  const copy=x=>JSON.parse(JSON.stringify(x));
  const mm=x=>(x/100).toFixed(2);
  class BarcodeView {
    constructor(element){
      this.element=element;this.history=new BarcodeHistory();this.frozen=null;this.selected=null;this.elementIndex=0;this.sampleIndex=0;this.scheduled=false;
      this.find=name=>element.querySelector(`[data-barcode="${name}"]`);
      for(let code=65;code<=90;code++){
        const option=document.createElement('option');option.value=option.textContent=String.fromCharCode(code);this.find('expected').append(option);
      }
      this.find('expected').value='F';
      for(const name of ['expected','orientation','ratio'])this.find(name).addEventListener('change',()=>this.render());
      this.find('scan').addEventListener('change',e=>{this.selected=e.target.value;this.elementIndex=0;this.sampleIndex=0;this.render();});
      this.find('pause').addEventListener('click',()=>{this.frozen=this.frozen?null:copy(this.history.frames);this.render();});
      this.find('clear').addEventListener('click',()=>{this.history.frames=[];this.history.evicted=0;this.selected=null;this.frozen=null;this.render();});
      this.find('sample').addEventListener('input',e=>{this.sampleIndex=Number(e.target.value);this.renderSample();});
      this.find('export').addEventListener('click',()=>{
        const blob=new Blob([this.history.export(this.frozen||this.history.frames)],{type:'application/json'});
        const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='rover-barcode-scans.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
      });
      this.find('import').addEventListener('change',async e=>{
        try{const file=e.target.files[0];if(!file)return;if(file.size>4000000)throw new Error('Scan export exceeds 4 MB');
          this.frozen=BarcodeHistory.read(await file.text());this.selected=null;this.render();this.find('notice').textContent='Imported scan replay — live reader state still describes the connected rover.';
        }catch(error){this.find('notice').textContent=error.message;}e.target.value='';
      });
      this.timer=setInterval(()=>{if(this.history.state!==null&&Date.now()-this.history.receivedAt>2500)this.unavailable();else this.renderLive();},1000);
      element.addEventListener('ir-view-change',event=>{if(event.detail.mode==='attempts')this.render();});
      this.render();
    }
    handle(data){
      if(!this.history.accept(data))return false;
      this.renderLive();
      if(!this.frozen&&!this.element.querySelector('[data-ir="attempts-panel"]').hidden&&!this.scheduled){this.scheduled=true;requestAnimationFrame(()=>{this.scheduled=false;this.render();});}
      return true;
    }
    unavailable(){this.history.unavailable();this.renderLive();}
    renderLive(){
      const r=this.history.readiness;
      this.find('readiness').textContent=r?
        `Side-sensor snapshot: raw ${r[0]}, normalized ${r[1]}/1000, ${r[2]?'black':'white'}. White lead-in observed: ${mm(r[3])} mm. Sensor travel this observation: ${mm(r[4])} mm; allowed ≤${mm(r[5])} mm. Oversized steps rejected: ${r[7]} (last ${mm(r[6])} mm).`:
        'Pre-scan sensor/encoder readings are unavailable in this firmware. Rebuild the current barcode branch to report why capture is waiting.';
      const state=this.history.state,blocked=this.history.last?.blocked??0;
      const label=state===null?'Reader state unavailable / buffered history':
        state===6?(blocked&32?'Suspended: sampling too coarse — slow down':blocked&2?'Suspended: encoder fault':blocked&4?'Suspended: excessive turn':
          blocked&8?'Suspended: reverse motion':blocked&1?'Configured — press GP21 to start':blocked&16?'Suspended: junction or recovery':'Configured — reader suspended'):states[state];
      this.find('status').textContent='Decoder: '+label;
      this.renderLetter(state,label);
    }
    renderLetter(state,label){
      const card=document.getElementById('barcode-letter');if(!card)return;
      const pick=name=>card.querySelector(`[data-letter="${name}"]`);
      const reader=pick('reader');
      reader.textContent=state===null?'Reader: offline':state===0?'Reader: off':'Reader: '+label;
      reader.dataset.tone=state===null?'muted':state===0?'warning':state===2||state===3?'accent':'success';
      const ended=this.history.frames.filter(f=>f.end);
      const latest=ended[ended.length-1],good=[...ended].reverse().find(f=>f.end.reason===1);
      if(latest&&`${latest.boot}:${latest.id}`!==this.letterKey){this.letterKey=`${latest.boot}:${latest.id}`;this.letterAt=Date.now();if(latest.end.reason===1)this.goodAt=this.letterAt;}
      const ago=t=>{const s=Math.round((Date.now()-t)/1000);return s<2?'just now':s<60?`${s}s ago`:`${Math.round(s/60)} min ago`;};
      if(good){
        pick('symbol').textContent=String.fromCharCode(good.end.symbol);
        pick('headline').textContent=`Read “${String.fromCharCode(good.end.symbol)}”`;
        pick('detail').textContent=`${good.end.reversed?'Print was flipped (read in reverse)':'Read forward'} · scan ${good.id}${this.goodAt?` · ${ago(this.goodAt)}`:''}`;
      }else{
        pick('symbol').textContent='–';
        pick('headline').textContent=latest?'No letter decoded yet':'No barcode read yet';
        pick('detail').textContent=state===0||state===null?
          'Reader off. Enable it in IR / barcode → Tools → Reader setup.':
          'Press GP21, then drive slowly over the whole barcode.';
      }
      const rejected=latest&&latest.end.reason!==1;
      pick('latest').textContent=rejected?`Latest attempt (scan ${latest.id}, ${ago(this.letterAt)}) rejected: ${reasons[latest.end.reason]}${latest.end.reason===2?` / ${formats[latest.end.format]}`:''}`:
        state===2||state===3?'Scanning now…':'';
      card.dataset.result=state===2||state===3?'scanning':good&&!rejected?'ok':rejected?'rejected':'none';
    }
    frames(){return this.frozen||this.history.frames;}
    frame(){const frames=this.frames();return frames.find(f=>`${f.boot}:${f.id}`===this.selected)||frames[frames.length-1];}
    render(){
      this.renderLive();const frames=this.frames(),select=this.find('scan');select.replaceChildren();
      for(const f of frames){const option=document.createElement('option');option.value=`${f.boot}:${f.id}`;option.textContent=`Scan ${f.id} · boot ${f.boot} · ${f.end?states[f.end.state]:'capture / incomplete'}`;select.append(option);}
      const f=this.frame();select.disabled=!f;if(f)select.value=`${f.boot}:${f.id}`;
      this.find('pause').textContent=this.frozen?'Resume live history':'Freeze scans';this.find('pause').setAttribute('aria-pressed',String(!!this.frozen));
      this.find('notice').textContent=`${frames.length}/20 scans · ${this.history.evicted} evicted · ${this.frozen?'Frozen; rover is still running':'Live capture'} · expected letter is your independent reference`;
      if(!f){
        this.find('outcome').textContent='Waiting for a scan. Configure the distance reader while stopped, then press GP21. Reading remains observe-only.';
        for(const name of ['strip','reference','signal','elements'])this.find(name).replaceChildren();
        this.find('summary').textContent='Bars and spaces will appear as they are measured. No exact barcode width is assumed.';
        this.find('detail').textContent='Select a measured element to inspect its width and encoder references.';
        this.find('sample-detail').textContent='No raw samples yet.';this.find('sample').disabled=true;return;
      }
      const end=f.end,progress=end||f.progress;
      const edges=f.edges.filter(Boolean).length,samples=f.samples.filter(Boolean).length;
      const missingEdges=end?end.count-edges:0,missingSamples=end?end.samples-samples:0;
      const incomplete=!f.meta||f.gap||f.conflict||missingEdges>0||missingSamples>0||(progress?.sample_drops??0)>0;
      this.find('outcome').textContent=end?
        (end.reason===1?`Firmware decoded ${String.fromCharCode(end.symbol)} (${end.reversed?'reverse':'forward'}) — observe only`:`Rejected: ${reasons[end.reason]}${end.reason===2?` / ${formats[end.format]}`:''}`):
        (f.progress?states[f.progress.state]:'Capturing candidate; no validated letter yet');
      if(end?.blocked)this.find('outcome').textContent+=' · '+[[1,'tracking stopped/unconfigured'],[2,'encoder fault'],[4,'excessive heading change'],[8,'reverse wheel travel'],[16,'junction/recovery phase'],[32,'sample spacing too large']].filter(([bit])=>end.blocked&bit).map(([,label])=>label).join(', ');
      if(incomplete)this.find('outcome').textContent+=' · diagnostic evidence incomplete';
      const cal=f.meta?.config;
      this.find('summary').textContent=`${edges}/${end?.count??29} completed elements received · ${samples}/${end?.samples??512} raw samples · boot ${f.boot}, scan ${f.id}, rover ${f.updated} ms; queue age ${f.lag_ms} ms.\n`+
        `Leading white: ${f.meta?mm(f.meta.leading):'unknown'} mm; trailing white: ${end?mm(end.trailing):'not confirmed'} mm. `+
        `${incomplete?'Missing/uncertain telemetry is marked unknown; no bars are invented. ':''}Raw capture drops: ${progress?.sample_drops??0}; trace frame drops: ${end?.dropped??f.meta?.dropped??0}.\n`+
        (cal?`Side ADC2 · raw endpoints ${cal[0]}–${cal[1]}, inversion ${cal[2]}; white ≤${cal[3]}, black ≥${cal[4]}; minimum resolvable element ${mm(cal[5])} mm, offset left ${cal[6]/10} mm.\nWheel diameter ${cal[8]/10} mm, track ${cal[7]/10} mm, encoder edges/rev L ${cal[9]/10}, R ${cal[10]/10}.`:'Calibration metadata unavailable.');
      this.drawBars(f);this.drawSignal(f);this.renderElements(f);this.renderSample();
    }
    add(svg,tag,attrs={},text){
      const node=document.createElementNS('http://www.w3.org/2000/svg',tag);
      for(const [k,v]of Object.entries(attrs))node.setAttribute(k,String(v));
      if(text!==undefined)node.textContent=text;svg.append(node);return node;
    }
    referenceFor(f){
      const choice=this.find('orientation').value;
      const reversed=choice==='reverse'||(choice==='validated'&&!!f.end?.reversed);
      return reference(this.find('expected').value,reversed,Number(this.find('ratio').value));
    }
    drawBars(f){
      const observed=this.find('strip'),expected=this.find('reference');observed.replaceChildren();expected.replaceChildren();
      const est=estimate(f),scale=est?.narrow||1,ref=this.referenceFor(f);
      let prefix=0,closed=0;
      for(const row of f.edges){if(!row)break;prefix+=row[2]/100;++closed;}
      const partial=(!f.end&&f.progress)?Math.max(0,f.progress.partial/100):f.end?.reason!==1?Math.max(0,(f.end?.partial||0)/100):0;
      const extent=Math.max(ref.reduce((a,b)=>a+b,0)*scale,prefix+partial,1);
      const x=distance=>5+distance/extent*990;
      let position=0;
      ref.forEach((width,i)=>{this.add(expected,'rect',{x:x(position),y:6,width:Math.max(.5,width*scale/extent*990),height:62,fill:i%2===0?'#111':'#fff'});position+=width*scale;});
      position=0;
      for(let i=0;i<closed;++i){
        const row=f.edges[i],width=row[2]/100;
        const rect=this.add(observed,'rect',{x:x(position),y:6,width:Math.max(.5,width/extent*990),height:62,fill:i%2===0?'#111':'#fff',class:i===this.elementIndex?'barcode-selected':''});
        const title=document.createElementNS('http://www.w3.org/2000/svg','title');title.textContent=`Element ${i+1}, ${i%2===0?'black':'white'}, ${width.toFixed(2)} mm`;rect.append(title);
        rect.addEventListener('click',()=>{this.elementIndex=i;this.render();});position+=width;
      }
      if(partial>0)this.add(observed,'rect',{x:x(position),y:6,width:Math.max(1,partial/extent*990),height:62,class:'barcode-partial','data-black':String(closed%2===0)});
      const expectedCount=f.end?.count??f.progress?.count??closed;
      if(closed<expectedCount)this.add(observed,'rect',{x:x(position),y:6,width:Math.max(1,1000-x(position)),height:62,class:'barcode-unknown'});
      this.find('scale').textContent=est?
        `Shared distance scale: ${extent.toFixed(2)} mm across. Estimated narrow ${est.narrow.toFixed(2)} mm, wide ${est.wide.toFixed(2)} mm. Dashed outline = unfinished observed black/white segment; orange = unknown. White quiet zones are reported above.`:
        'Reference is normalized until nine measured elements establish scale. Observed widths remain in mm; incomplete/missing elements are not reconstructed.';
    }
    renderElements(f){
      const list=this.find('elements');list.replaceChildren();const ref=this.referenceFor(f);
      const count=Math.max(f.end?.count??f.progress?.count??0,f.edges.reduce((n,r,i)=>r?i+1:n,0));
      for(let i=0;i<count;++i){
        const row=f.edges[i],button=document.createElement('button');button.type='button';button.className='barcode-element';button.dataset.selected=String(i===this.elementIndex);
        let classification='?';
        if(i===9||i===19)classification=row?'gap':'?';
        else {
          const group=Math.floor(i/10)*10,all=f.edges.slice(group,group+9);
          if(row&&all.length===9&&all.every(Boolean)){
            const sorted=all.map(r=>r[2]).sort((a,b)=>a-b);classification=row[2]>(sorted[5]+sorted[6])/2?'wide':'narrow';
          }
        }
        const mismatch=row&&['wide','narrow'].includes(classification)&&((classification==='wide')!==(ref[i]>1));
        button.dataset.mismatch=String(!!mismatch);
        button.textContent=`${i+1} ${i%2===0?'B':'W'} · ${row?mm(row[2])+' mm':'unknown'} · ${classification}${mismatch?' ≠ reference':''}`;
        button.addEventListener('click',()=>{this.elementIndex=i;this.render();});list.append(button);
      }
      const row=f.edges[this.elementIndex];
      this.find('detail').textContent=row?`Element ${row[0]+1} · ${row[0]%2===0?'black bar':'white space'} · measured ${mm(row[2])} mm\nEdge detected at sample ${row[1]} ms. Wheel observations at that detection: L ${row[3]/10} mm / R ${row[4]/10} mm.\nWidths are sensor-travel estimates; wheel positions are observations at detection, not exact optical-edge coordinates. A reference mismatch alone cannot identify the cause.`:'Selected element was not received or not completed. No width/class is guessed.';
    }
    drawSignal(f){
      const svg=this.find('signal');svg.replaceChildren();const samples=f.samples.filter(Boolean);if(!samples.length)return;
      const start=samples[0][1],span=Math.max(1,(samples[samples.length-1][1]-start)>>>0);
      let raw='',norm='',previous=-2;
      for(const row of samples){const move=row[0]===previous+1?'L':'M',x=5+(((row[1]-start)>>>0)/span)*990;
        raw+=`${move}${x},${94-row[2]/4095*88} `;norm+=`${move}${x},${94-row[3]/1000*88} `;previous=row[0];}
      this.add(svg,'path',{d:raw,class:'barcode-raw-signal'});this.add(svg,'path',{d:norm,class:'barcode-norm-signal'});
      if(f.meta)for(const threshold of f.meta.config.slice(3,5))this.add(svg,'line',{x1:0,x2:1000,y1:94-threshold/1000*88,y2:94-threshold/1000*88,class:'barcode-threshold'});
      const lastIndex=samples[samples.length-1][0];this.find('sample').max=String(lastIndex);this.find('sample').disabled=false;
      this.sampleIndex=Math.min(this.sampleIndex,lastIndex);this.find('sample').value=String(this.sampleIndex);
    }
    renderSample(){
      const f=this.frame(),r=f?.samples[this.sampleIndex];
      this.find('sample-detail').textContent=r?`Sample ${r[0]+1} · rover ${r[1]} ms · raw ${r[2]} / normalized ${r[3]} · hysteresis state ${r[7]?'black':'white'}\nObserved wheel positions L ${r[4]/10} mm, R ${r[5]/10} mm; estimated local sensor travel ${mm(r[6])} mm. Stationary flicker changes signals, not spatial widths.`:'No received sample at this index. A diagnostic gap is not a white barcode space.';
    }
  }
  root.RoverBarcodeView=BarcodeView;
})(globalThis);
