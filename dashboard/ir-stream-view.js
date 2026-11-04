(function(root){
  'use strict';
  const {IRStreamHistory,viewport}=root.RoverIRStream;
  class IRStreamView {
    constructor(element) {
      this.element=element;this.history=new IRStreamHistory();this.frozen=null;this.cursor=null;this.scheduled=false;
      this.find=name=>element.querySelector(`[data-ir="${name}"]`);
      this.tabs=[...element.querySelectorAll('[data-ir-mode]')];
      for(const [i,tab] of this.tabs.entries()) {
        tab.addEventListener('click',()=>this.mode(tab.dataset.irMode));
        tab.addEventListener('keydown',e=>{
          if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','Home','End'].includes(e.key))return;
          e.preventDefault();e.stopPropagation();
          const next=e.key==='Home'?0:e.key==='End'?this.tabs.length-1:
            (i+(['ArrowLeft','ArrowUp'].includes(e.key)?-1:1)+this.tabs.length)%this.tabs.length;
          this.mode(this.tabs[next].dataset.irMode);this.tabs[next].focus();
        });
      }
      this.find('freeze').addEventListener('click',()=>{this.frozen=this.frozen?null:this.history.records.slice();this.cursor=null;this.render();});
      this.find('clear').addEventListener('click',()=>{this.history.records=[];this.history.evicted=0;this.frozen=null;this.cursor=null;this.render();});
      this.find('window').addEventListener('change',()=>this.render());
      this.find('cursor').addEventListener('input',e=>{if(!this.frozen)this.frozen=this.history.records.slice();this.cursor=Number(e.target.value);this.render();});
      this.find('export').addEventListener('click',()=>{
        const url=URL.createObjectURL(new Blob([this.history.export(this.frozen||this.history.records)],{type:'application/json'}));
        const a=document.createElement('a');a.href=url;a.download='rover-continuous-ir.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
      });
      this.find('import').addEventListener('change',async e=>{
        try{const file=e.target.files[0];if(!file)return;if(file.size>4000000)throw new Error('IR recording exceeds 4 MB');
          this.frozen=IRStreamHistory.read(await file.text());this.cursor=null;this.render();this.find('notice').textContent='Imported continuous IR recording. Live indicator still describes the connected rover.';
        }catch(error){this.find('notice').textContent=error.message;}e.target.value='';
      });
      this.timer=setInterval(()=>{if(this.history.live&&Date.now()-this.history.receivedAt>500)this.unavailable();},250);
      this.mode('continuous');
    }
    mode(name) {
      if(!['continuous','attempts'].includes(name))return;
      for(const tab of this.tabs){const selected=tab.dataset.irMode===name;tab.setAttribute('aria-selected',String(selected));tab.tabIndex=selected?0:-1;}
      this.find('continuous-panel').hidden=name!=='continuous';this.find('attempts-panel').hidden=name!=='attempts';
      this.element.dispatchEvent(new CustomEvent('ir-view-change',{detail:{mode:name}}));
      if(name==='continuous')this.render();
    }
    handle(d) {
      if(!this.history.accept(d))return false;
      this.live();
      if(!this.frozen&&!this.find('continuous-panel').hidden&&!this.scheduled){this.scheduled=true;requestAnimationFrame(()=>{this.scheduled=false;this.render();});}
      return true;
    }
    unavailable(){this.history.unavailable();this.live();}
    live() {
      const sample=this.history.live;
      this.find('live').textContent=sample?(sample.row[4]?'Black observed':'White observed'):'Sensor state unavailable';
      this.find('live').dataset.color=sample?(sample.row[4]?'black':'white'):'unknown';
    }
    add(svg,tag,attrs,text){const node=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const [k,v]of Object.entries(attrs))node.setAttribute(k,String(v));if(text!==undefined)node.textContent=text;svg.append(node);return node;}
    render() {
      this.live();const records=this.frozen||this.history.records;
      const index=this.cursor===null?records.length-1:Math.min(this.cursor,records.length-1);
      const slider=this.find('cursor');slider.disabled=!records.length;slider.max=String(Math.max(0,records.length-1));slider.value=String(Math.max(0,index));
      this.find('freeze').textContent=this.frozen?'Resume IR history':'Freeze IR history';this.find('freeze').setAttribute('aria-pressed',String(!!this.frozen));
      this.find('notice').textContent=`${records.length}/6000 observations · ${this.history.evicted} evicted · firmware drops ${this.history.dropped} · ${this.frozen?'Frozen; sensing continues':'Continuous capture, independent of decoder'}`;
      const strip=this.find('strip'),signal=this.find('signal');strip.replaceChildren();signal.replaceChildren();
      const view=viewport(records,index,Number(this.find('window').value));
      if(!view){this.find('axis').textContent='No continuous IR samples received. Empty is unknown, not white. Rebuild/reflash the current branch to enable rover/ir.';this.find('detail').textContent='Continuous capture does not need barcode configuration, a quiet zone or wheel movement.';return;}
      const x=t=>5+(t-view.start)/view.span*990;
      for(const segment of view.segments) {
        const rect=this.add(strip,'rect',{x:x(segment.from),y:5,width:Math.max(.5,(segment.to-segment.from)/view.span*990),height:65,fill:segment.value===null?'#94a3b8':segment.value?'#111':'#fff','data-color':segment.value===null?'unknown':segment.value?'black':'white'});
        const title=document.createElementNS('http://www.w3.org/2000/svg','title');title.textContent=`${segment.to-segment.from} ms · ${segment.reason|| (segment.value?'black':'white')}`;rect.append(title);
      }
      // A current sample is a point, not a fabricated future bar width.
      this.add(strip,'line',{x1:995,x2:995,y1:0,y2:75,stroke:view.selected.row[4]?'#111':'#fff','stroke-width':3});
      let raw='',normalized='',previous=null;
      // Retain bucket extrema and every gap boundary when zoomed out.
      // All 6000 observations remain available for inspection/export.
      const stride=Math.max(1,Math.ceil(view.points.length/400));
      for(let i=0;i<view.points.length;i+=stride) {
        const bucket=view.points.slice(i,i+stride);
        const chosen=new Set([bucket[0],bucket[bucket.length-1],bucket.reduce((a,b)=>a.row[2]<b.row[2]?a:b),bucket.reduce((a,b)=>a.row[2]>b.row[2]?a:b),bucket.reduce((a,b)=>a.row[3]<b.row[3]?a:b),bucket.reduce((a,b)=>a.row[3]>b.row[3]?a:b)]);
        bucket.forEach((p,i)=>{if(p.gap){chosen.add(p);if(i)chosen.add(bucket[i-1]);}});
        for(const p of bucket.filter(p=>chosen.has(p))) {
          const move=!previous||p.gap||((p.row[0]-previous.row[0])>>>0)>stride*2?'M':'L';
          raw+=`${move}${x(p.time)},${94-p.row[2]/4095*88} `;normalized+=`${move}${x(p.time)},${94-p.row[3]/1000*88} `;previous=p;
        }
      }
      this.add(signal,'path',{d:raw,class:'barcode-raw-signal'});this.add(signal,'path',{d:normalized,class:'barcode-norm-signal'});
      for(const threshold of [400,600])this.add(signal,'line',{x1:0,x2:1000,y1:94-threshold/1000*88,y2:94-threshold/1000*88,class:'barcode-threshold'});
      const r=view.selected.row;
      this.find('axis').textContent=`${Math.max(0,-view.start)} ms of time history ending at rover ${r[1]} ms · grey = unknown/missing observations · bar widths here represent time, not printed millimetres.`;
      this.find('detail').textContent=`Boot ${view.selected.boot} · sample ${r[0]} · rover ${r[1]} ms · ${r[4]?'BLACK':'WHITE'}\nRaw ${r[2]}/4095 · normalized ${r[3]}/1000 · thresholds white ≤400 / black ≥600\nWheel positions L ${r[5]/10} mm / R ${r[6]/10} mm · decoder blocked flags ${r[7]} (do not block this view)\nRaw calibration ${r[8]}–${r[9]}, inverted ${r[10]}. ${view.selected.gap?'Gap before sample: '+view.selected.gap+'. ':''}This is sensor evidence, not a decoded barcode.`;
    }
  }
  root.RoverIRStreamView=IRStreamView;
})(globalThis);
