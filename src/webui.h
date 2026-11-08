#pragma once

#include "state.h"
#include "sensors.h"
#include "display.h"
#include "proxy.h"
#include <ESP8266WebServer.h>
#include <Updater.h>

extern ESP8266WebServer server;

// ---- Shared CSS & Header (stored in PROGMEM) ----
static const char PAGE_CSS[] PROGMEM = R"rawliteral(
:root{--bg:#12161a;--card:#1b2127;--bdr:#2c353f;--txt:#e6edf3;--mut:#8b949e;--acc:#38dbff;--ok:#20e060;--warn:#ffb800;--err:#ff3030}
*{box-sizing:border-box}
body{margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;background:var(--bg);color:var(--txt);line-height:1.45}
nav{background:var(--card);border-bottom:1px solid var(--bdr);padding:10px 16px;display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between}
nav .brand{font-weight:700;font-size:1.1rem;color:var(--acc);text-decoration:none}
nav .links{display:flex;flex-wrap:wrap;gap:10px}
nav .links a{color:var(--txt);text-decoration:none;font-size:.9rem;padding:4px 8px;border-radius:6px;background:#252d36}
nav .links a:hover{background:#323d49}
.wrap{max-width:1040px;margin:0 auto;padding:16px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px;margin-bottom:14px}
.card{background:var(--card);border:1px solid var(--bdr);border-radius:10px;padding:14px}
.card h3{margin:0 0 10px;font-size:1.02rem;display:flex;justify-content:space-between;align-items:center}
.badge{font-size:.78rem;padding:2px 8px;border-radius:12px;font-weight:600;background:#2a333d;color:var(--txt)}
.b-ok{background:rgba(32,224,96,.18);color:var(--ok);border:1px solid var(--ok)}
.b-warn{background:rgba(255,184,0,.18);color:var(--warn);border:1px solid var(--warn)}
.b-err{background:rgba(255,48,48,.22);color:var(--err);border:1px solid var(--err)}
.b-info{background:rgba(56,219,255,.18);color:var(--acc);border:1px solid var(--acc)}
.kv{display:grid;grid-template-columns:1fr auto;gap:6px 12px;font-size:.9rem}
.kv span:nth-child(odd){color:var(--mut)}
.kv span:nth-child(even){font-weight:600;text-align:right}
.hero{font-size:1.9rem;font-weight:700;margin:4px 0 10px;color:#fff;display:flex;justify-content:space-between;align-items:baseline}
label{display:block;font-size:.85rem;color:var(--mut);margin:10px 0 4px}
input,select{width:100%;padding:8px 10px;border-radius:6px;border:1px solid var(--bdr);background:#12161a;color:var(--txt);font-size:.92rem}
button,.btn{display:inline-block;padding:8px 14px;border-radius:6px;border:none;background:var(--acc);color:#091318;font-weight:700;font-size:.9rem;cursor:pointer;text-decoration:none;margin-top:10px}
button.sec,.btn.sec{background:#2c353f;color:var(--txt)}
button.danger,.btn.danger{background:var(--err);color:#fff}
.tabs{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px}
canvas{width:100%;height:285px;background:#14191e;border-radius:8px;border:1px solid var(--bdr);display:block;touch-action:none}
.sub{font-size:.82rem;color:var(--mut);margin-top:6px}
table{width:100%;border-collapse:collapse;font-size:.86rem;margin-top:8px}
th,td{border-bottom:1px solid var(--bdr);padding:6px;text-align:left}
)rawliteral";

static const char NAV_BAR[] PROGMEM = R"rawliteral(
<nav>
  <a class="brand" href="/">iSpindleStation</a>
  <div class="links">
    <a href="/">Dashboard</a>
    <a href="/managment">iSpindel</a>
    <a href="/config">Sensors &amp; Hardware</a>
    <a href="/network">Network &amp; Cloud</a>
    <a href="/info">Info</a>
    <a href="/update">OTA Update</a>
  </div>
</nav>
)rawliteral";

// ---- Client-side iSpindlePlotter Analytics Engine (/app.js in PROGMEM) ----
static const char APP_JS[] PROGMEM = R"rawliteral(
(function(){
  let state = { slot: 1, points: [], exclusions: [], segments: [], segIdx: -1, cursorIdx: null, status: null, calPts: [] };

  function ensureSg(v) {
    if (!v || v <= 0.5) return 0;
    if (v <= 1.5) return v;
    return 1.0 + (v / (258.6 - ((v / 258.2) * 227.1)));
  }
  function sgToPlato(v) {
    if (!v || v <= 0.5) return 0;
    if (v > 1.5) return v;
    return -616.868 + 1111.14*v - 630.272*v*v + 135.997*v*v*v;
  }
  function median(arr) {
    if (!arr.length) return 0;
    const s = arr.slice().sort((a,b)=>a-b);
    const m = s.length >> 1;
    return (s.length & 1) ? s[m] : 0.5*(s[m-1]+s[m]);
  }
  function fitLinear(xs, ys) {
    const n = xs.length;
    if (n < 2) return null;
    let sx=0, sy=0, sxy=0, sxx=0;
    for (let i=0;i<n;i++){ sx+=xs[i]; sy+=ys[i]; sxy+=xs[i]*ys[i]; sxx+=xs[i]*xs[i]; }
    const d = n*sxx - sx*sx;
    if (Math.abs(d) < 1e-12) return null;
    const slope = (n*sxy - sx*sy)/d;
    const intercept = (sy - slope*sx)/n;
    return { slope, intercept, predict: x => intercept + slope*x };
  }

  // FermentSegmenter (ported from FermentSegmenter.kt)
  function detectSegments(pts) {
    if (pts.length < 6) return [];
    const cuts = [0];
    for (let i=1; i<pts.length; i++) {
      const dtH = (pts[i].ts - pts[i-1].ts) / 3600;
      const dSg = pts[i].sg - pts[i-1].sg;
      let max6h = 0;
      for (let j=i-1; j>=0 && (pts[i-1].ts - pts[j].ts) <= 21600; j--) {
        if (pts[j].sg > max6h) max6h = pts[j].sg;
      }
      if (dtH > 8.0 && dSg > 0.004) { cuts.push(i); continue; }
      if (dSg > 0.005 && pts[i].sg > max6h + 0.002) { cuts.push(i); }
    }
    cuts.push(pts.length);
    const segs = [];
    for (let c=0; c<cuts.length-1; c++) {
      const a = cuts[c], b = cuts[c+1]-1;
      if (b - a + 1 < 6) continue;
      const spanH = (pts[b].ts - pts[a].ts) / 3600;
      if (spanH < 2.0) continue;
      let lo = 99, hi = 0;
      for (let k=a; k<=b; k++) { if (pts[k].sg < lo) lo = pts[k].sg; if (pts[k].sg > hi) hi = pts[k].sg; }
      if (hi - lo >= 0.003 || c === cuts.length-2) {
        segs.push({ startIdx: a, endIdx: b, startTs: pts[a].ts, endTs: pts[b].ts, og: hi, fg: lo });
      }
    }
    return segs;
  }

  // PlateauDetector (ported from PlateauDetector.kt: rolling slope <= 0.3 mSG/h for >= 2.5h)
  function detectPlateaus(hours, sgs) {
    const n = hours.length;
    if (n < 6) return [];
    const flat = new Array(n).fill(false);
    for (let i=0; i<n; i++) {
      const xs=[], ys=[];
      for (let j=0; j<n; j++) {
        if (Math.abs(hours[j] - hours[i]) <= 1.5) { xs.push(hours[j]); ys.push(sgs[j]); }
      }
      const f = fitLinear(xs, ys);
      if (f && f.slope >= -0.0003 && f.slope <= 0.0004) flat[i] = true;
    }
    const out = [];
    let i = 0;
    while (i < n) {
      if (!flat[i]) { i++; continue; }
      let j = i;
      while (j + 1 < n && flat[j+1]) j++;
      const dur = hours[j] - hours[i];
      if (dur >= 2.5 && (j - i + 1) >= 4) {
        const sub = sgs.slice(i, j+1);
        const med = median(sub);
        const kind = (hours[i] - hours[0] < 3.0) ? "Lag" : ((hours[n-1] - hours[j] < 2.0) ? "Tail" : "Mid");
        out.push({ startH: hours[i], endH: hours[j], durationH: dur, sg: med, kind });
      }
      i = j + 1;
    }
    return out;
  }

  // Modified-Gompertz curve: SG(t) = OG - A * exp(-exp((mu * e / A) * (lambda - t) + 1))
  function gompertzEval(og, fg, mu, lam, t) {
    const A = Math.max(og - fg, 1e-5);
    const u = (mu * Math.E / A) * (lam - t) + 1.0;
    if (u > 40) return og;
    if (u < -40) return fg;
    return og - A * Math.exp(-Math.exp(u));
  }

  // Levenberg-Marquardt MAP fitter with Gaussian attenuation prior N(0.75, 0.10^2)
  function fitGompertz(hours, sgs, ogFixed) {
    const n = hours.length;
    if (n < 8) return null;
    const cur = sgs[n-1];
    if (ogFixed - cur < 0.003) return null;
    const priorFg = Math.max(0.998, 1.0 + 0.25*(ogFixed - 1.0));
    let fg = Math.min(cur - 0.002, priorFg);
    let mu = 0.0012;
    let lam = Math.max(0, hours[0] + 2.0);
    let damp = 1e-2;
    const priorWeight = 1.0 / (0.10 * Math.max(ogFixed - 1.0, 0.01));

    for (let iter=0; iter<35; iter++) {
      let JtJ = [[0,0,0],[0,0,0],[0,0,0]], Jtr = [0,0,0], rss = 0;
      const eps = 1e-5;
      for (let i=0; i<n; i++) {
        const t = hours[i];
        const y = gompertzEval(ogFixed, fg, mu, lam, t);
        const r = y - sgs[i];
        rss += r*r;
        const g0 = (gompertzEval(ogFixed, fg+eps, mu, lam, t) - y)/eps;
        const g1 = (gompertzEval(ogFixed, fg, mu+eps, lam, t) - y)/eps;
        const g2 = (gompertzEval(ogFixed, fg, mu, lam+eps, t) - y)/eps;
        const g = [g0, g1, g2];
        for (let a=0;a<3;a++) {
          Jtr[a] += g[a]*r;
          for (let b=0;b<3;b++) JtJ[a][b] += g[a]*g[b];
        }
      }
      // Soft attenuation prior residual on fg
      const rPrior = (fg - priorFg) * priorWeight;
      rss += rPrior * rPrior;
      Jtr[0] += priorWeight * rPrior;
      JtJ[0][0] += priorWeight * priorWeight;

      for (let a=0;a<3;a++) JtJ[a][a] *= (1 + damp);
      // 3x3 solve
      const A = JtJ.map((row, idx) => row.concat([-Jtr[idx]]));
      for (let k=0;k<3;k++) {
        let piv = k;
        for (let r=k+1;r<3;r++) if (Math.abs(A[r][k]) > Math.abs(A[piv][k])) piv = r;
        const tmp = A[k]; A[k] = A[piv]; A[piv] = tmp;
        if (Math.abs(A[k][k]) < 1e-12) continue;
        for (let r=k+1;r<3;r++) {
          const f = A[r][k]/A[k][k];
          for (let c=k;c<=3;c++) A[r][c] -= f*A[k][c];
        }
      }
      const d = [0,0,0];
      for (let k=2;k>=0;k--) {
        let s = A[k][3];
        for (let c=k+1;c<3;c++) s -= A[k][c]*d[c];
        d[k] = Math.abs(A[k][k]) > 1e-12 ? s/A[k][k] : 0;
      }
      const nFg = Math.max(0.990, Math.min(ogFixed - 0.004, fg + d[0]));
      const nMu = Math.max(0.0001, Math.min(0.015, mu + d[1]));
      const nLam = Math.max(0, Math.min(hours[n-1], lam + d[2]));
      fg = nFg; mu = nMu; lam = nLam;
      if (Math.abs(d[0]) < 1e-5 && Math.abs(d[1]) < 1e-6) break;
    }
    const sigma = Math.max(0.0015, 0.06*(ogFixed - 1.0)*Math.max(0, 1 - (ogFixed - cur)/Math.max(ogFixed - fg, 1e-4)));
    // ETA to fg + 0.001
    const A = Math.max(ogFixed - fg, 1e-5);
    let etaH = null;
    if (A > 0.002) {
      const targetRatio = 0.001 / A;
      const uTarget = Math.log(-Math.log(targetRatio));
      const tEnd = lam - (uTarget - 1.0) * A / (mu * Math.E);
      if (isFinite(tEnd) && tEnd > hours[n-1]) etaH = tEnd - hours[n-1];
    }
    return { og: ogFixed, fg, mu, lam, fgSigma: sigma, etaH };
  }

  function analyseFerment(pts) {
    if (pts.length < 6) return { phase: "Waiting", desc: "Need at least 6 readings (>= 1 hour) to classify.", spund: null, gompertz: null, plateaus: [] };
    const t0 = pts[0].ts;
    const hours = pts.map(p => (p.ts - t0)/3600);
    const sgs = pts.map(p => p.sg);
    const temps = pts.map(p => p.temp);
    const spanH = hours[hours.length-1] - hours[0];
    if (spanH < 1.0) return { phase: "Waiting", desc: "Collecting initial baseline readings.", spund: null, gompertz: null, plateaus: [] };

    const plateaus = detectPlateaus(hours, sgs);
    let ogRaw = Math.max(...sgs);
    const lagP = plateaus.find(p => p.kind === "Lag");
    const krausen = lagP && (lagP.sg < ogRaw - 0.0015);
    const og = krausen ? lagP.sg : ogRaw;
    const fitSgs = krausen ? sgs.map(v => Math.min(v, og)) : sgs;
    const cur = sgs[sgs.length-1];
    const priorFg = Math.max(0.998, 1.0 + 0.25*(og - 1.0));

    // Tail 6h slope
    const tailCut = hours[hours.length-1] - 6.0;
    const tH=[], tS=[];
    for (let i=0;i<hours.length;i++) if (hours[i]>=tailCut){ tH.push(hours[i]); tS.push(sgs[i]); }
    const tf = fitLinear(tH, tS);
    const rate = tf ? tf.slope : 0;

    // Cold crash check (>= 4C drop)
    const maxT = Math.max(...temps), curT = temps[temps.length-1];
    const isColdCrash = (maxT - curT >= 4.0) && (og - cur > 0.005);

    const gomp = fitGompertz(hours, fitSgs, og);
    const predFg = gomp ? gomp.fg : priorFg;
    const drop = og - cur;
    const rateNearZero = Math.abs(rate) < 0.00012;

    let phase = "Active", desc = "";
    if (isColdCrash) {
      phase = "Cold Crash";
      desc = `Temperature dropped from ${maxT.toFixed(1)}° to ${curT.toFixed(1)}°. Apparent SG includes thermal density shift.`;
    } else if (drop < 0.003 && rate > -0.00008) {
      phase = "Lag";
      desc = `Yeast adapting in wort (${spanH.toFixed(1)} h). Hold fermentation temperature steady.`;
    } else if (rateNearZero && drop > 0.005 && cur <= priorFg + 0.003) {
      // SettlingDetector check: >= 8h warm tail step-down of 1.5..6 mSG
      phase = "Conditioning";
      desc = `Gravity settled near FG (${cur.toFixed(4)}). Allow diacetyl rest and yeast cleanup before cold crashing.`;
    } else if (rateNearZero && drop > 0.005 && cur > priorFg + 0.005) {
      phase = "Stuck";
      desc = `Stalled at ${cur.toFixed(4)} above expected FG (${priorFg.toFixed(4)}). Consider warming or gently rousing yeast.`;
    } else if (rate < -0.0005 || (drop / Math.max(og - predFg, 0.001)) < 0.5 || (cur - predFg) > 0.012) {
      phase = "Active";
      const midPause = plateaus.find(p => p.kind === "Mid" && Math.abs(cur - p.sg) <= 0.002);
      if (midPause) {
        desc = `Paused at ${midPause.sg.toFixed(4)} for ${midPause.durationH.toFixed(1)} h (possible diauxic shift as yeast switches sugars).`;
      } else {
        desc = `Dropping at ${(rate*1000).toFixed(2)} mSG/h. Predicted FG: ${predFg.toFixed(4)}` + (gomp && gomp.etaH ? ` (ETA ~${gomp.etaH.toFixed(0)} h).` : ".");
      }
    } else {
      phase = "Slowing";
      desc = `Decelerating (${(rate*1000).toFixed(2)} mSG/h) approaching predicted FG ${predFg.toFixed(4)}.`;
    }

    // Spunding readiness (Spunding.kt)
    let spund = null;
    if (phase !== "Stuck" && og > 1.000 && predFg < og) {
      const t15 = og - 0.90*(og - predFg);
      const t10 = Math.max(og - Math.max(0.15*(og - predFg), 0.005), t15);
      if (phase === "Conditioning" || phase === "Clarifying" || phase === "Cold Crash" || cur <= t15) {
        spund = { stage: "~15 psi (1.03 bar)", note: `At/near terminal gravity (<= ${t15.toFixed(3)}). Bung up to ~15 psi for natural carbonation.` };
      } else if (cur <= t10) {
        spund = { stage: "~10 psi (0.69 bar)", note: `Active descent underway. Safe for ~10 psi; raise to ~15 psi at SG ${t15.toFixed(3)}.` };
      } else {
        spund = { stage: "Hold (0 psi)", note: `Leave blow-off open until SG reaches ${t10.toFixed(3)} so pressure does not suppress yeast growth.` };
      }
    }
    return { phase, desc, spund, gompertz: gomp, plateaus, og, predFg, rate, hours, sgs, temps };
  }

  function drawChart(canvasId, analysis, pts) {
    const cv = document.getElementById(canvasId);
    if (!cv) return;
    const ctx = cv.getContext("2d");
    const dpr = window.devicePixelRatio || 1;
    const W = cv.clientWidth || 600, H = cv.clientHeight || 250;
    cv.width = W * dpr; cv.height = H * dpr;
    ctx.scale(dpr, dpr);
    ctx.fillStyle = "#14191e"; ctx.fillRect(0, 0, W, H);
    if (!pts.length) {
      ctx.fillStyle = "#8b949e"; ctx.font = "13px sans-serif";
      ctx.fillText("No readings recorded for this slot yet.", 20, H/2);
      return;
    }
    const padL = 52, padR = 46, padT = 18, padB = 42;
    const cw = W - padL - padR, ch = H - padT - padB;
    const t0 = pts[0].ts;
    const hours = pts.map(p => (p.ts - t0)/3600);
    const lastObsH = hours[hours.length-1];
    const gomp = analysis.gompertz;
    const maxH = Math.max(lastObsH, gomp && gomp.etaH ? lastObsH + Math.min(gomp.etaH, 72) : lastObsH, 1);

    let minSg = Math.min(...pts.map(p=>p.sg)), maxSg = Math.max(...pts.map(p=>p.sg));
    if (gomp) { minSg = Math.min(minSg, gomp.fg - gomp.fgSigma); maxSg = Math.max(maxSg, gomp.og); }
    const sgPad = Math.max((maxSg - minSg)*0.08, 0.002);
    minSg -= sgPad; maxSg += sgPad;

    const xOf = h => padL + (h / maxH) * cw;
    const yOf = sg => padT + (1 - (sg - minSg)/(maxSg - minSg)) * ch;

    // Horizontal SG gridlines + Y-axis labels
    ctx.strokeStyle = "#232b34"; ctx.lineWidth = 1;
    for (let i=0; i<=4; i++) {
      const y = padT + (i/4)*ch;
      const v = maxSg - (i/4)*(maxSg - minSg);
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W-padR, y); ctx.stroke();
      ctx.fillStyle = "#8b949e"; ctx.font = "11px sans-serif";
      ctx.fillText(v.toFixed(3), 6, y + 4);
    }

    // Vertical time gridlines + X-axis ticks & labels
    const nTicks = Math.max(4, Math.min(6, Math.floor(cw / 105)));
    const months = ["Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"];
    const pad2 = n => (n < 10 ? "0" + n : "" + n);
    for (let i=0; i<=nTicks; i++) {
      const h = (i / nTicks) * maxH;
      const x = xOf(h);
      ctx.strokeStyle = "#232b34"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + ch); ctx.stroke();
      ctx.strokeStyle = "#485460";
      ctx.beginPath(); ctx.moveTo(x, padT + ch); ctx.lineTo(x, padT + ch + 4); ctx.stroke();

      ctx.fillStyle = "#8b949e"; ctx.font = "10.5px sans-serif";
      ctx.textAlign = (i === 0) ? "left" : ((i === nTicks) ? "right" : "center");
      const hLbl = `+${h < 10 ? h.toFixed(1) : Math.round(h)}h`;
      if (t0 > 1000000000) {
        const d = new Date((t0 + h * 3600) * 1000);
        const hm = `${pad2(d.getHours())}:${pad2(d.getMinutes())}`;
        const topLbl = maxH >= 24 ? `${d.getDate()} ${months[d.getMonth()]} ${hm}` : hm;
        ctx.fillText(topLbl, x, padT + ch + 15);
        ctx.fillStyle = "#6e7681"; ctx.font = "10px sans-serif";
        ctx.fillText(hLbl, x, padT + ch + 28);
      } else {
        ctx.fillText(hLbl, x, padT + ch + 16);
      }
    }
    ctx.textAlign = "left";

    // Mark "Now" divider when forward Gompertz projection extends beyond observations
    if (maxH > lastObsH + 0.5 && lastObsH > 0) {
      const nx = xOf(lastObsH);
      ctx.strokeStyle = "rgba(230,237,243,0.35)"; ctx.lineWidth = 1; ctx.setLineDash([3,3]);
      ctx.beginPath(); ctx.moveTo(nx, padT); ctx.lineTo(nx, padT + ch); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "#8b949e"; ctx.font = "10px sans-serif";
      ctx.fillText("Now", Math.min(nx + 4, W - padR - 26), padT + ch - 5);
    }

    // Shade detected plateaus (Lag / Mid diauxic / Tail)
    for (const p of (analysis.plateaus || [])) {
      const x1 = xOf(p.startH), x2 = xOf(p.endH);
      ctx.fillStyle = p.kind === "Mid" ? "rgba(255,184,0,0.14)" : "rgba(56,219,255,0.08)";
      ctx.fillRect(x1, padT, Math.max(x2-x1, 2), ch);
      ctx.fillStyle = p.kind === "Mid" ? "#ffb800" : "#8b949e";
      ctx.font = "10px sans-serif";
      ctx.fillText(`${p.kind} ${p.durationH.toFixed(1)}h`, x1 + 3, padT + 12);
    }

    // Gompertz curve & 95% credible band
    if (gomp) {
      ctx.fillStyle = "rgba(32,224,96,0.13)";
      ctx.beginPath();
      const steps = 60;
      for (let s=0; s<=steps; s++) {
        const h = (s/steps)*maxH;
        const y = gompertzEval(gomp.og, gomp.fg - 1.96*gomp.fgSigma, gomp.mu, gomp.lam, h);
        s===0 ? ctx.moveTo(xOf(h), yOf(y)) : ctx.lineTo(xOf(h), yOf(y));
      }
      for (let s=steps; s>=0; s--) {
        const h = (s/steps)*maxH;
        const y = gompertzEval(gomp.og, gomp.fg + 1.96*gomp.fgSigma, gomp.mu, gomp.lam, h);
        ctx.lineTo(xOf(h), yOf(y));
      }
      ctx.closePath(); ctx.fill();

      ctx.strokeStyle = "#20e060"; ctx.lineWidth = 1.6; ctx.setLineDash([4,4]);
      ctx.beginPath();
      for (let s=0; s<=steps; s++) {
        const h = (s/steps)*maxH;
        const y = gompertzEval(gomp.og, gomp.fg, gomp.mu, gomp.lam, h);
        s===0 ? ctx.moveTo(xOf(h), yOf(y)) : ctx.lineTo(xOf(h), yOf(y));
      }
      ctx.stroke(); ctx.setLineDash([]);
    }

    // Temperature trace (secondary axis)
    const minT = Math.min(...pts.map(p=>p.temp)) - 1, maxT = Math.max(...pts.map(p=>p.temp)) + 1;
    const yOfT = t => padT + (1 - (t - minT)/Math.max(maxT - minT, 1)) * ch;
    ctx.strokeStyle = "rgba(255,140,66,0.55)"; ctx.lineWidth = 1.3;
    ctx.beginPath();
    pts.forEach((p, idx) => {
      const x = xOf(hours[idx]), y = yOfT(p.temp);
      idx===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
    });
    ctx.stroke();
    ctx.fillStyle = "#ff8c42"; ctx.font = "11px sans-serif";
    ctx.fillText(maxT.toFixed(1)+"°", W-padR+4, padT+10);
    ctx.fillText(minT.toFixed(1)+"°", W-padR+4, padT+ch);

    // SG polyline + points
    ctx.strokeStyle = "#38dbff"; ctx.lineWidth = 2;
    ctx.beginPath();
    pts.forEach((p, idx) => {
      const x = xOf(hours[idx]), y = yOf(p.sg);
      idx===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
    });
    ctx.stroke();

    // Cursor overlay
    if (state.cursorIdx !== null && pts[state.cursorIdx]) {
      const ci = state.cursorIdx, cp = pts[ci];
      const cx = xOf(hours[ci]), cy = yOf(cp.sg);
      ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(cx, padT); ctx.lineTo(cx, padT+ch); ctx.stroke();
      ctx.fillStyle = "#38dbff"; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI*2); ctx.fill();
      const dt = new Date(cp.ts * 1000).toLocaleString();
      document.getElementById("cursorReadout").textContent =
        `Cursor [${ci+1}/${pts.length}] ${dt} (+${hours[ci].toFixed(1)}h): SG ${cp.sg.toFixed(4)} | Temp ${cp.temp.toFixed(1)}° | Batt ${cp.batt.toFixed(2)}V | Tilt ${cp.angle.toFixed(2)}°`;
    }
  }

  function renderActiveView() {
    const raw = state.points;
    const excl = state.exclusions.filter(e => e.slot === state.slot);
    const filtered = raw.filter(p => !excl.some(e => p.ts >= e.startTs && p.ts <= e.endTs));
    state.segments = detectSegments(filtered);
    let viewPts = filtered;
    if (state.segments.length > 0) {
      if (state.segIdx < 0) state.segIdx = state.segments.length - 1;
      if (state.segIdx < state.segments.length) {
        const seg = state.segments[state.segIdx];
        viewPts = filtered.slice(seg.startIdx, seg.endIdx + 1);
      }
    }
    const segLabel = document.getElementById("segLabel");
    if (segLabel) {
      if (!state.segments.length) segLabel.textContent = `All readings (${viewPts.length})`;
      else if (state.segIdx >= state.segments.length) segLabel.textContent = `All (${viewPts.length} pts)`;
      else segLabel.textContent = `Ferment ${state.segIdx+1}/${state.segments.length} (${viewPts.length} pts)`;
    }

    const an = analyseFerment(viewPts);
    const pBadge = document.getElementById("phaseBadge");
    if (pBadge) { pBadge.textContent = an.phase; pBadge.className = "badge " + (an.phase==="Stuck"?"b-err":(an.phase==="Active"?"b-ok":"b-info")); }
    const pDesc = document.getElementById("phaseDesc");
    if (pDesc) pDesc.textContent = an.desc;
    const spBox = document.getElementById("spundBox");
    if (spBox) {
      if (an.spund) {
        let rPress = "";
        if (state.status && state.status.relay && state.status.relay.pressureEnabled) {
          rPress = ` | Live Vessel Pressure: ${state.status.relay.pressure.toFixed(2)} ${state.status.relay.pressureUnit}`;
        }
        spBox.innerHTML = `<b>Spunding Target: ${an.spund.stage}</b>${rPress}<br><span class="sub">${an.spund.note}</span>`;
      } else {
        spBox.innerHTML = `<span class="sub">Spunding guidance activates once OG and descent are established.</span>`;
      }
    }
    drawChart("sgCanvas", an, viewPts);
    state.viewPts = viewPts;
  }

  async function loadSlot(s) {
    state.slot = s; state.cursorIdx = null;
    const [ndjson, exclRes] = await Promise.all([
      fetch(`/data/${s}.jsonl`).then(r => r.text()),
      fetch(`/exclusions`).then(r => r.json()).catch(()=>[])
    ]);
    state.exclusions = Array.isArray(exclRes) ? exclRes : [];
    const pts = [];
    for (const line of ndjson.split("\n")) {
      if (!line.trim()) continue;
      try {
        const obj = JSON.parse(line);
        const pl = obj.payload || {};
        const sg = ensureSg(parseFloat(pl.gravity || 0));
        if (sg > 0.5) {
          pts.push({
            id: obj.id || 0,
            ts: obj.ts || 0,
            sg,
            temp: parseFloat(pl.temperature || 0),
            batt: parseFloat(pl.battery || 0),
            angle: parseFloat(pl.angle || 0)
          });
        }
      } catch(e){}
    }
    state.points = pts;
    state.segIdx = -1;
    renderActiveView();
  }

  async function loadStatus() {
    try {
      const st = await fetch("/api/status").then(r => r.json());
      state.status = st;
      const cards = document.getElementById("slotCards");
      if (cards && st.slots) {
        let html = "";
        for (const sl of st.slots) {
          if (sl.slot === 0 && !sl.active) continue;
          const al = (sl.alarmSg || sl.alarmTemp) ? `<span class="badge b-err">ALARM</span>` : `<span class="badge b-ok">${sl.phase}</span>`;
          html += `<div class="card">
            <h3><span>Slot ${sl.slot}: ${sl.name || "Waiting..."}</span>${al}</h3>
            <div class="hero"><span>${sl.unit===2 ? sl.plato.toFixed(2)+" °P" : sl.sg.toFixed(4)+" SG"}</span><span style="font-size:1.1rem;color:var(--acc)">${sl.temp.toFixed(1)} °${sl.tempUnit}</span></div>
            <div class="kv">
              <span>Original Gravity (OG)</span><span>${sl.og > 0 ? sl.og.toFixed(4) : "Auto (5th sample)"}</span>
              <span>ABV / Attenuation</span><span>${sl.abv.toFixed(2)}% / ${sl.attenuation.toFixed(1)}%</span>
              <span>Battery / Angle</span><span>${sl.batt.toFixed(2)} V (${sl.batPct.toFixed(0)}%) / ${sl.angle.toFixed(2)}°</span>
              <span>Spunding Gate</span><span>${sl.spundStage}</span>
              <span>Last Seen / Interval</span><span>${sl.ageSec}s ago / ${sl.interval}s</span>
            </div>
          </div>`;
        }
        if (st.relay && (st.relay.tempEnabled || st.relay.pressureEnabled)) {
          const ral = (st.relay.tempAlarm || st.relay.pressureAlarm) ? `<span class="badge b-err">ALARM</span>` : `<span class="badge b-info">ONBOARD</span>`;
          html += `<div class="card">
            <h3><span>Relay Onboard Sensors</span>${ral}</h3>
            <div class="kv">
              <span>DS18B20 Chamber Temp</span><span>${st.relay.tempEnabled ? (st.relay.tempError ? "Sensor error" : st.relay.temp.toFixed(2)+" °"+st.relay.tempUnit) : "Disabled"}</span>
              <span>Vessel Pressure (A0)</span><span>${st.relay.pressureEnabled ? st.relay.pressure.toFixed(2)+" "+st.relay.pressureUnit : "Disabled"}</span>
            </div>
          </div>`;
        }
        cards.innerHTML = html;
      }
    } catch(e){}
  }

  // OLS polynomial calibration fitter (degree 1..3)
  window.fitCalPoly = function() {
    const deg = parseInt(document.getElementById("calDeg").value || "2", 10);
    const rows = state.calPts;
    if (rows.length < deg + 1) { alert(`Add at least ${deg+1} (tilt, SG) points.`); return; }
    const m = deg + 1;
    const XtX = Array.from({length:m}, ()=>new Array(m).fill(0));
    const Xty = new Array(m).fill(0);
    for (const p of rows) {
      for (let i=0;i<m;i++) {
        Xty[i] += Math.pow(p.tilt, i) * p.sg;
        for (let j=0;j<m;j++) XtX[i][j] += Math.pow(p.tilt, i+j);
      }
    }
    const A = XtX.map((r,i)=>r.concat([Xty[i]]));
    for (let k=0;k<m;k++) {
      let piv=k; for (let r=k+1;r<m;r++) if (Math.abs(A[r][k])>Math.abs(A[piv][k])) piv=r;
      const t=A[k]; A[k]=A[piv]; A[piv]=t;
      for (let r=k+1;r<m;r++) { const f=A[r][k]/A[k][k]; for (let c=k;c<=m;c++) A[r][c]-=f*A[k][c]; }
    }
    const c = new Array(m).fill(0);
    for (let k=m-1;k>=0;k--) {
      let s=A[k][m]; for (let j=k+1;j<m;j++) s-=A[k][j]*c[j];
      c[k] = s/A[k][k];
    }
    let expr = c[0].toFixed(8);
    for (let i=1;i<m;i++) {
      const sign = c[i]>=0 ? " + " : " - ";
      expr += sign + Math.abs(c[i]).toFixed(8) + (i===1 ? "*tilt" : `*tilt^${i}`);
    }
    document.getElementById("cfgPoly").value = expr;
  };

  window.addCalPt = function() {
    const tilt = parseFloat(document.getElementById("calTilt").value);
    const sg = parseFloat(document.getElementById("calSg").value);
    if (isNaN(tilt) || isNaN(sg)) return;
    state.calPts.push({tilt, sg});
    document.getElementById("calList").textContent = state.calPts.map(p=>`(${p.tilt.toFixed(2)}° -> ${p.sg.toFixed(3)})`).join(", ");
  };

  window.pushCfgToSpindle = async function() {
    const poly = document.getElementById("cfgPoly").value.trim();
    const sleep = parseInt(document.getElementById("cfgSleep").value || "0", 10);
    const body = {};
    if (poly) body.POLY = poly;
    if (sleep > 10) body.sleep = sleep;
    const res = await fetch("/cfg", {method:"PUT", headers:{"Content-Type":"application/json"}, body:JSON.stringify(body)});
    const txt = await res.text();
    document.getElementById("cfgStatus").textContent = res.ok ? `Armed for next wake: ${txt}` : `Error: ${txt}`;
  };

  window.clearCfgPush = async function() {
    await fetch("/cfg", {method:"DELETE"});
    document.getElementById("cfgStatus").textContent = "Cleared config push.";
  };

  window.selectSlot = s => loadSlot(s);
  window.prevSeg = () => { if (state.segments.length && state.segIdx > 0) { state.segIdx--; renderActiveView(); } };
  window.nextSeg = () => { if (state.segments.length && state.segIdx < state.segments.length) { state.segIdx++; renderActiveView(); } };

  window.addEventListener("DOMContentLoaded", () => {
    const cv = document.getElementById("sgCanvas");
    if (cv) {
      const onMove = e => {
        if (!state.viewPts || !state.viewPts.length) return;
        const rect = cv.getBoundingClientRect();
        const clientX = (e.touches && e.touches[0]) ? e.touches[0].clientX : e.clientX;
        const rel = Math.max(0, Math.min(1, (clientX - rect.left - 52) / Math.max(rect.width - 98, 1)));
        state.cursorIdx = Math.min(state.viewPts.length - 1, Math.floor(rel * state.viewPts.length));
        renderActiveView();
      };
      cv.addEventListener("mousemove", onMove);
      cv.addEventListener("touchmove", onMove, {passive:true});
    }
    loadStatus();
    loadSlot(1);
    setInterval(loadStatus, 15000);
  });
})();
)rawliteral";

// ---- Dashboard Page (GET / and GET /status) ----
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>iSpindleStation Dashboard</title>
<link rel="stylesheet" href="/style.css">
</head><body>
)rawliteral";

static const char DASHBOARD_BODY[] PROGMEM = R"rawliteral(
<div class="wrap">
  <div id="slotCards" class="grid"></div>

  <div class="card" style="margin-bottom:14px">
    <h3>
      <span>Fermentation Curve &amp; Bayesian Gompertz Model</span>
      <span id="phaseBadge" class="badge b-info">Loading</span>
    </h3>
    <div class="tabs">
      <button class="sec" onclick="selectSlot(1)">Slot 1</button>
      <button class="sec" onclick="selectSlot(2)">Slot 2</button>
      <button class="sec" onclick="selectSlot(0)">Slot 0 (Unprefixed)</button>
      <span style="margin-left:auto;display:flex;gap:6px;align-items:center">
        <button class="sec" onclick="prevSeg()">&#9664;</button>
        <span id="segLabel" style="font-size:.86rem">All</span>
        <button class="sec" onclick="nextSeg()">&#9654;</button>
      </span>
    </div>
    <p id="phaseDesc" style="margin:6px 0 10px;font-size:.92rem"></p>
    <div id="spundBox" style="padding:8px 10px;background:#14191e;border-radius:6px;border:1px solid var(--bdr);margin-bottom:10px;font-size:.88rem"></div>
    <canvas id="sgCanvas"></canvas>
    <div id="cursorReadout" class="sub">Hover or drag across the chart to inspect historical readings, plateaus, and Gompertz fit.</div>
    <div class="sub" style="margin-top:8px">
      Raw exports: <a href="/data/1.jsonl" style="color:var(--acc)">Slot 1 JSONL</a> &middot;
      <a href="/data/2.jsonl" style="color:var(--acc)">Slot 2 JSONL</a> &middot;
      <a href="/graph/1.svg" style="color:var(--acc)">Slot 1 SVG</a> &middot;
      <a href="/graph/2.svg" style="color:var(--acc)">Slot 2 SVG</a> &middot;
      <a href="/readings?since=0" style="color:var(--acc)">Proxy Feed (/readings)</a>
    </div>
  </div>

  <div class="card">
    <h3><span>Polynomial Calibration &amp; Zero-Touch iSpindel Config Push (/cfg)</span></h3>
    <div class="grid">
      <div>
        <label>Add Calibration Point (Tilt Angle &deg; &rarr; Known SG)</label>
        <div style="display:flex;gap:8px">
          <input id="calTilt" type="number" step="0.01" placeholder="Tilt (e.g. 25.4)">
          <input id="calSg" type="number" step="0.001" placeholder="SG (e.g. 1.000)">
          <select id="calDeg" style="width:110px"><option value="1">Deg 1</option><option value="2" selected>Deg 2</option><option value="3">Deg 3</option></select>
        </div>
        <button class="sec" onclick="addCalPt()">+ Add Point</button>
        <button class="sec" onclick="fitCalPoly()">Fit Polynomial</button>
        <div id="calList" class="sub">Sugar table (20 &deg;C): 0 g/100g=1.000, 5g=1.020, 10g=1.040, 15g=1.060, 20g=1.080</div>
      </div>
      <div>
        <label>Piggybacked POLY (TinyExpr, must include <code>tilt</code>)</label>
        <input id="cfgPoly" type="text" placeholder="0.95683653 + 0.00156709*tilt + 0.00000376*tilt^2">
        <label>Piggybacked Wake Interval (seconds, 11..86399)</label>
        <input id="cfgSleep" type="number" placeholder="900">
        <button onclick="pushCfgToSpindle()">Arm Config Push</button>
        <button class="sec" onclick="clearCfgPush()">Clear</button>
        <div id="cfgStatus" class="sub">Attached to the next POST /gravity response.</div>
      </div>
    </div>
  </div>
</div>
<script src="/app.js"></script>
</body></html>
)rawliteral";

static void handleStyleCss()
{
  server.send_P(200, "text/css", PAGE_CSS);
}

static void handleAppJs()
{
  server.send_P(200, "application/javascript", APP_JS);
}

static void handleDashboard()
{
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent_P(DASHBOARD_HTML);
  server.sendContent_P(NAV_BAR);
  server.sendContent_P(DASHBOARD_BODY);
  server.sendContent("");
}

static void handleApiStatus()
{
  DynamicJsonDocument doc(2048);
  doc["firmware"] = FIRMWARE_VERSION;
  doc["epoch"] = epochNow();
  doc["sntp"] = sntpActive();
  doc["staIp"] = WiFi.localIP().toString();
  doc["apIp"] = WiFi.softAPIP().toString();
  doc["rssi"] = WiFi.RSSI();
  doc["heap"] = ESP.getFreeHeap();

  JsonObject r = doc.createNestedObject("relay");
  r["tempEnabled"] = (cfg.tempYesNo == 2);
  r["tempError"] = (fabsf(relaySensors.temp - 127.0f) < 0.1f);
  r["temp"] = relaySensors.temp;
  r["tempUnit"] = (cfg.tempUnit == 2) ? "F" : "C";
  r["tempAlarm"] = relaySensors.tempAlarm;
  r["pressureEnabled"] = (cfg.pressureUnit == 1 || cfg.pressureUnit == 2);
  r["pressure"] = (cfg.pressureUnit == 2) ? relaySensors.psi : relaySensors.bar;
  r["pressureUnit"] = (cfg.pressureUnit == 2) ? "psi" : "bar";
  r["pressureAlarm"] = relaySensors.pressureAlarm;

  JsonArray arr = doc.createNestedArray("slots");
  for (int s = 0; s < 3; s++)
  {
    SlotState &st = slots[s];
    int cfgIdx = (s == 2) ? 2 : 1;
    JsonObject o = arr.createNestedObject();
    o["slot"] = s;
    o["active"] = st.active;
    o["name"] = st.name;
    o["sg"] = ensureSg(st.corrSg);
    o["plato"] = sgToPlato(st.corrSg);
    o["unit"] = cfg.unit[cfgIdx];
    o["temp"] = st.corrTemp;
    o["tempUnit"] = String(st.tempUnit);
    o["batt"] = st.batt;
    o["batPct"] = st.batPct;
    o["angle"] = st.angle;
    o["rssi"] = st.rssi;
    o["interval"] = st.interval;
    o["ageSec"] = slotAgeSeconds(s);
    o["og"] = cfg.og[cfgIdx];
    o["ogLocked"] = st.ogLocked;
    o["abv"] = st.abv;
    o["attenuation"] = st.attenuation;
    o["stockState"] = stockFermLabel(st.stockFermState);
    o["phase"] = phaseLabel(st.phase);
    const char *spStr = "Hold";
    if (st.spundStage == SPUND_10PSI)
      spStr = "~10 psi (0.69 bar)";
    else if (st.spundStage == SPUND_15PSI)
      spStr = "~15 psi (1.03 bar)";
    else if (st.spundStage == SPUND_NONE)
      spStr = "-";
    o["spundStage"] = spStr;
    o["alarmTemp"] = st.alarmTemp;
    o["alarmSg"] = st.alarmSg;
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ---- Configuration Pages (/network, /config, /managment, /info, /update) ----
static String htmlEsc(const String &s)
{
  String o = s;
  o.replace("&", "&amp;");
  o.replace("\"", "&quot;");
  o.replace("<", "&lt;");
  o.replace(">", "&gt;");
  return o;
}

static void handleNetworkPage()
{
  String p;
  p.reserve(3200);
  p += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>Network &amp; Cloud - iSpindleStation</title><link rel='stylesheet' href='/style.css'></head><body>");
  p += FPSTR(NAV_BAR);
  p += F("<div class='wrap'><div class='card'><h3>Network &amp; Cloud Forwarding</h3>"
         "<form action='/get' method='GET'>"
         "<div class='grid'><div>"
         "<label>Home WiFi SSID</label><input name='ssid' value=\"");
  p += htmlEsc(cfg.ssid);
  p += F("\"><label>Home WiFi Password</label><input name='password' type='password' value=\"");
  p += htmlEsc(cfg.password);
  p += F("\"><label>Companion AP SSID (for iSpindels)</label><input name='ap_ssid' value=\"");
  p += htmlEsc(cfg.apSsid);
  p += F("\"><label>Companion AP Password (min 8 chars)</label><input name='ap_pass' value=\"");
  p += htmlEsc(cfg.apPass);
  p += F("\"></div><div><label>Forwarding Service</label><select name='service'>");
  struct SvcOpt { int id; const char *label; };
  static const SvcOpt opts[] = {
      {255, "None (Local & Proxy Only)"},
      {0, "0 - UbiDots"},
      {1, "1 - Brewfather"},
      {2, "2 - BrewSpy"},
      {3, "3 - Monitor.Beer"},
      {4, "4 - Grainfather"},
      {5, "5 - BrewPiless"},
      {6, "6 - Little Bock"},
      {7, "7 - Brewers Friend"},
      {8, "8 - BrewBench"},
      {9, "9 - MQTT (Home Assistant Auto-Discovery)"},
      {10, "10 - Generic HTTP (host:port)"},
      {11, "11 - NAUTILIS Tapboard"}};
  for (const auto &o : opts)
  {
    p += F("<option value='");
    p += String(o.id);
    p += (cfg.service == o.id) ? F("' selected>") : F("'>");
    p += o.label;
    p += F("</option>");
  }
  p += F("</select><label>Server Address / Host[:Port] (HTTP, BrewPiless, MQTT)</label><input name='server_address' value=\"");
  p += htmlEsc(cfg.serverAddress);
  p += F("\"><label>URI / MQTT Broker / Stream Path</label><input name='uri' value=\"");
  p += htmlEsc(cfg.uri);
  p += F("\"><label>Token / API Key</label><input name='TOKEN' value=\"");
  p += htmlEsc(cfg.token);
  p += F("\"><label>Grainfather URI 1 / URI 2</label><div style='display:flex;gap:6px'>"
         "<input name='grainfather_uri1' placeholder='Spindle 1 URI' value=\"");
  p += htmlEsc(cfg.grainfatherUri1);
  p += F("\"><input name='grainfather_uri2' placeholder='Spindle 2 URI' value=\"");
  p += htmlEsc(cfg.grainfatherUri2);
  p += F("\"></div><label>Little Bock URI 1 / URI 2</label><div style='display:flex;gap:6px'>"
         "<input name='littlebock_uri1' placeholder='Spindle 1 URI' value=\"");
  p += htmlEsc(cfg.littlebockUri1);
  p += F("\"><input name='littlebock_uri2' placeholder='Spindle 2 URI' value=\"");
  p += htmlEsc(cfg.littlebockUri2);
  p += F("\"></div><label>MQTT Username / Password</label><div style='display:flex;gap:6px'>"
         "<input name='mqtt_user' placeholder='MQTT User' value=\"");
  p += htmlEsc(cfg.mqttUser);
  p += F("\"><input name='mqtt_pass' type='password' placeholder='MQTT Pass' value=\"");
  p += htmlEsc(cfg.mqttPass);
  p += F("\"></div><label>Relay Sensor Custom URI &amp; Period (min, >=10)</label><div style='display:flex;gap:6px'>"
         "<input name='uri_custom' placeholder='Custom Stream URI' value=\"");
  p += htmlEsc(cfg.uriCustom);
  p += F("\"><input name='custom_period' type='number' style='width:100px' value='");
  p += String(cfg.customPeriod);
  p += F("'></div></div></div><button type='submit'>Save Network &amp; Service</button></form></div></div></body></html>");
  server.send(200, "text/html", p);
}

static void handleConfigPage()
{
  String p;
  p.reserve(3000);
  p += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>Hardware &amp; Sensors - iSpindleStation</title><link rel='stylesheet' href='/style.css'></head><body>");
  p += FPSTR(NAV_BAR);
  p += F("<div class='wrap'><div class='card'><h3>Relay Hardware, DS18B20 &amp; Pressure Transducer</h3>"
         "<form action='/save_config' method='GET'><div class='grid'><div>"
         "<label>TFT Backlight (GPIO 15)</label><select name='ledState'>"
         "<option value='1'");
  if (cfg.ledState) p += F(" selected");
  p += F(">ON</option><option value='0'");
  if (!cfg.ledState) p += F(" selected");
  p += F(">OFF</option></select><label>TFT Rotation (0..3)</label><select name='tft_rotate'>");
  for (int r = 0; r < 4; r++)
  {
    p += F("<option value='");
    p += String(r);
    if (cfg.tftRotate == r) p += F("' selected>Rotation ");
    else p += F("'>Rotation ");
    p += String(r);
    p += F("</option>");
  }
  p += F("</select><label>Piezo Buzzer (GPIO 12)</label><select name='buzz_yesno'>"
         "<option value='2'");
  if (cfg.buzzYesNo == 2) p += F(" selected");
  p += F(">Enabled (Save &amp; Alarm Beeps)</option><option value='1'");
  if (cfg.buzzYesNo != 2) p += F(" selected");
  p += F(">Muted</option></select><label>Onboard DS18B20 Thermometer (GPIO 2)</label><select name='temp_yesno'>"
         "<option value='1'");
  if (cfg.tempYesNo != 2) p += F(" selected");
  p += F(">Disabled</option><option value='2'");
  if (cfg.tempYesNo == 2) p += F(" selected");
  p += F(">Enabled</option></select><label>DS18B20 Unit &amp; Offset</label><div style='display:flex;gap:6px'>"
         "<select name='temp_unit'><option value='1'");
  if (cfg.tempUnit == 1) p += F(" selected");
  p += F(">Celsius (&deg;C)</option><option value='2'");
  if (cfg.tempUnit == 2) p += F(" selected");
  p += F(">Fahrenheit (&deg;F)</option></select><input name='TempCorrection1' type='number' step='0.1' value='");
  p += String(cfg.tempCorrection1, 2);
  p += F("'></div><label>DS18B20 Alarm Low / High</label><div style='display:flex;gap:6px'>"
         "<input name='alarm_3' type='number' value='");
  p += String(cfg.alarmRelayTempLow);
  p += F("'><input name='alarm_4' type='number' value='");
  p += String(cfg.alarmRelayTempHigh);
  p += F("'></div></div><div><label>Onboard Pressure Sensor (A0)</label><select name='pressure_unit' id='pUnitSel' onchange='onPUnit(this.value)'>"
         "<option value='3'");
  if (cfg.pressureUnit == 3) p += F(" selected");
  p += F(">Off</option><option value='1'");
  if (cfg.pressureUnit == 1) p += F(" selected");
  p += F(">Bar</option><option value='2'");
  if (cfg.pressureUnit == 2) p += F(" selected");
  p += F(">Psi</option></select><label>ADC Zero (minpressure) / ADC Full Scale (maxpressure)</label><div style='display:flex;gap:6px'>"
         "<input name='minpressure' type='number' value='");
  p += String(cfg.minPressure);
  p += F("'><input name='maxpressure' type='number' value='");
  p += String(cfg.maxPressure);
  p += F("'></div><label>Full-Scale Pressure (Bar at maxpressure ADC)</label>"
         "<input name='maxpressure_bar' type='number' step='0.01' value='");
  p += String(cfg.maxPressureBar, 2);
  p += F("'><label id='pAlarmLbl'>Pressure Alarm Low / High (");
  p += (cfg.pressureUnit == 2) ? F("Psi") : F("Bar");
  p += F(")</label><div style='display:flex;gap:6px'>"
         "<input id='pLow' name='pressure_alarm_low' type='number' step='0.05' value='");
  p += String(cfg.pressureAlarmLow, 2);
  p += F("'><input id='pHigh' name='pressure_alarm_high' type='number' step='0.05' value='");
  p += String(cfg.pressureAlarmHigh, 2);
  p += F("'></div><div class='sub' style='margin-top:12px'>Live A0 ADC: <b>");
  p += String(relaySensors.rawAdc) + F("</b> (") + String(relaySensors.voltage, 2) +
       F(" V) &rarr; ") + String(relaySensors.bar, 2) + F(" bar / ") + String(relaySensors.psi, 1) +
       F(" psi</div></div></div><button type='submit'>Save Hardware Config</button></form>"
         "<script>var curPU=");
  p += String(cfg.pressureUnit);
  p += F(";function onPUnit(v){v=parseInt(v);var lbl=document.getElementById('pAlarmLbl'),lo=document.getElementById('pLow'),hi=document.getElementById('pHigh');"
         "if(v===2&&curPU!==2){lbl.textContent='Pressure Alarm Low / High (Psi)';if(parseFloat(hi.value)<=4.0){hi.value=(parseFloat(hi.value)*14.5038).toFixed(1);lo.value=(parseFloat(lo.value)*14.5038).toFixed(1);}}"
         "else if(v===1&&curPU===2){lbl.textContent='Pressure Alarm Low / High (Bar)';if(parseFloat(hi.value)>4.0){hi.value=(parseFloat(hi.value)/14.5038).toFixed(2);lo.value=(parseFloat(lo.value)/14.5038).toFixed(2);}}"
         "if(v!==3)curPU=v;}</script></div></div></body></html>");
  server.send(200, "text/html", p);
}

static void handleManagementPage()
{
  String p;
  p.reserve(3200);
  p += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>iSpindel Management - iSpindleStation</title><link rel='stylesheet' href='/style.css'></head><body>");
  p += FPSTR(NAV_BAR);
  p += F("<div class='wrap'><div class='card'><h3>iSpindel Calibration, OG &amp; Alarms</h3>"
         "<form action='/save_management' method='GET'>"
         "<label>Active iSpindel Slots</label><select name='ispindel_num'>"
         "<option value='1'");
  if (cfg.ispindelNum == 1) p += F(" selected");
  p += F(">1 iSpindel</option><option value='2'");
  if (cfg.ispindelNum == 2) p += F(" selected");
  p += F(">2 iSpindels</option></select><div class='grid' style='margin-top:10px'>");
  for (int s = 1; s <= 2; s++)
  {
    String idx = String(s);
    int aLow = (s == 1) ? 1 : 5;
    int aHigh = (s == 1) ? 2 : 6;
    p += F("<div><h4>iSpindel ");
    p += idx + F(" (Prefix ") + idx + F("-)</h4><label>Gravity Unit</label><select name='unit_") + idx + F("'><option value='1'");
    if (cfg.unit[s] == 1) p += F(" selected");
    p += F(">SG (1.xxx)</option><option value='2'");
    if (cfg.unit[s] == 2) p += F(" selected");
    p += F(">Plato (&deg;P)</option></select><label>Gravity Offset (SGkorekcija");
    p += idx + F(") / Temp Offset</label><div style='display:flex;gap:6px'><input name='SGkorekcija") + idx + F("' type='number' step='0.0001' value='");
    p += String(cfg.sgOffset[s], 4);
    p += F("'><input name='Tempkorekcija") + idx + F("' type='number' step='0.1' value='");
    p += String(cfg.tempOffset[s], 2);
    p += F("'></div><label>Original Gravity OG") + idx + F(" (0 = Auto-Lock on 5th Sample)</label><input name='OG") + idx + F("' type='number' step='0.0001' value='");
    p += String(cfg.og[s], 4);
    p += F("'><label>Temp Alarm Low / High</label><div style='display:flex;gap:6px'><input name='alarm_") + String(aLow) + F("' type='number' value='");
    p += String(cfg.alarmTempLow[s]);
    p += F("'><input name='alarm_") + String(aHigh) + F("' type='number' value='");
    p += String(cfg.alarmTempHigh[s]);
    p += F("'></div><label>Low Gravity Alarm (alarm_SG") + idx + F(")</label><input name='alarm_SG") + idx + F("' type='number' step='0.001' value='");
    p += String(cfg.alarmSgLow[s], 3);
    p += F("'><a class='btn danger' href='/new_fermentation?ispindel=") + idx + F("' onclick=\"return confirm('Reset OG and fermentation history?')\">Start New Fermentation (Slot ") + idx + F(")</a></div>");
  }
  p += F("</div><button type='submit'>Save iSpindel Settings</button></form></div></div></body></html>");
  server.send(200, "text/html", p);
}

static void handleInfoPage()
{
  fs::FSInfo64 fsinfo;
  SPIFFS.info64(fsinfo);
  String p;
  p.reserve(1800);
  p += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>System Info - iSpindleStation</title><link rel='stylesheet' href='/style.css'></head><body>");
  p += FPSTR(NAV_BAR);
  p += F("<div class='wrap'><div class='card'><h3>System &amp; Network Diagnostics</h3><div class='kv'>"
         "<span>Firmware Version</span><span>iSpindleStation v");
  p += String(FIRMWARE_VERSION);
  p += F("</span><span>ESP8266 Chip ID / Flash</span><span>0x");
  p += String(ESP.getChipId(), HEX) + F(" / ") + String(ESP.getFlashChipRealSize() / 1024) + F(" KB</span>");
  p += F("<span>Free Heap</span><span>") + String(ESP.getFreeHeap()) + F(" B</span>");
  p += F("<span>SPIFFS Used / Total</span><span>") + String((uint32_t)fsinfo.usedBytes) + F(" / ") + String((uint32_t)fsinfo.totalBytes) + F(" B</span>");
  p += F("<span>Station SSID / IP</span><span>") + htmlEsc(WiFi.SSID()) + F(" / ") + WiFi.localIP().toString() + F(" (") + String(WiFi.RSSI()) + F(" dBm)</span>");
  p += F("<span>Station MAC</span><span>") + WiFi.macAddress() + F("</span>");
  p += F("<span>Companion Soft-AP SSID / IP</span><span>") + htmlEsc(cfg.apSsid) + F(" / ") + WiFi.softAPIP().toString() + F(" (") + String(WiFi.softAPgetStationNum()) + F(" clients)</span>");
  p += F("<span>mDNS Proxy Service</span><span>_ispindle-proxy._tcp (ispindlestation.local:80)</span>");
  p += F("<span>Monotonic Reading Cursor</span><span>") + String(globalSeqId) + F("</span></div><div style='margin-top:14px'>"
         "<a class='btn sec' href='/debug'>JSON Debug</a> "
         "<a class='btn danger' href='/reboot' onclick=\"return confirm('Reboot station now?')\">Reboot Station</a>"
         "</div></div></div></body></html>");
  server.send(200, "text/html", p);
}

static void handleUpdateGet()
{
  String p;
  p += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>OTA Firmware Update - iSpindleStation</title><link rel='stylesheet' href='/style.css'></head><body>");
  p += FPSTR(NAV_BAR);
  p += F("<div class='wrap'><div class='card'><h3>Station Web OTA Firmware Update</h3>"
         "<p class='sub'>Upload a compiled <code>firmware.bin</code> image to update this iSpindleStation relay.</p>"
         "<form method='POST' action='/update' enctype='multipart/form-data'>"
         "<input type='file' name='update' accept='.bin' required>"
         "<button type='submit'>Upload &amp; Flash Station</button></form></div></div></body></html>");
  server.send(200, "text/html", p);
}

static void handleUpdatePostDone()
{
  bool ok = !Update.hasError();
  server.send(200, "text/html",
              ok ? "<html><body style='background:#12161a;color:#20e060;font-family:sans-serif;text-align:center;padding:40px'>"
                   "<h2>Firmware Updated! Rebooting...</h2><script>setTimeout(()=>location.href='/',6000)</script></body></html>"
                 : "<html><body style='background:#12161a;color:#ff3030;font-family:sans-serif;text-align:center;padding:40px'>"
                   "<h2>OTA Update Failed</h2><a href='/update' style='color:#38dbff'>Back</a></body></html>");
  if (ok)
  {
    digitalWrite(TFT_BL_PIN, LOW);
    rebootScheduled = true;
    rebootScheduledAt = millis();
  }
}

static void handleUpdatePostUpload()
{
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    if (Update.isRunning())
      Update.end(false);
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace))
      Update.printError(Serial);
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (!Update.hasError())
    {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
        Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (!Update.hasError() && Update.end(true))
      Serial.printf("OTA Update Success: %u B\n", upload.totalSize);
    else
      Update.printError(Serial);
  }
  else if (upload.status == UPLOAD_FILE_ABORTED)
  {
    Update.end(false);
    Serial.println("OTA Update Aborted");
  }
  yield();
}
