#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Browser-based visual tuner for the ROS1 camera line follower.

This node avoids Tkinter/DISPLAY problems.  It serves a HTML control panel on
http://127.0.0.1:<port>/ and writes ROS parameters in real time.  The panel also
streams the /line_follower/debug_image topic as JPEG so ROI, contour, centroid
and error can be checked while tuning.
"""
from __future__ import annotations

import json
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, String
from std_srvs.srv import Empty as EmptySrv

try:
    import cv2
    from cv_bridge import CvBridge, CvBridgeError
except Exception:
    cv2 = None
    CvBridge = None
    CvBridgeError = Exception


PARAM_DEFAULTS = {
    "enabled": True,
    "control_mode": "pid",
    "kp": 32.0,
    "ki": 0.0,
    "kd": 6.0,
    "pos_out_limit": 60.0,
    "pos_int_limit": 2.0,
    "base_speed_pct": 24.0,
    "base_min_pct": 12.0,
    "curve_factor": 0.55,
    "max_linear": 0.45,
    "max_angular": 2.6,
    "turn_sign": -1.0,
    "threshold_mode": "otsu",
    "roi_top_ratio": 0.42,
    "roi_bottom_ratio": 0.90,
    "roi_left_ratio": 0.05,
    "roi_right_ratio": 0.95,
    "black_threshold": 90,
    "adaptive_block_size": 31,
    "adaptive_c": 7,
    "blur_kernel": 5,
    "morph_kernel": 5,
    "min_area": 180.0,
    "min_black_ratio": 0.0015,
    "max_black_ratio": 0.85,
    "max_contour_area_ratio": 0.75,
    "reject_round_blob": True,
    "round_blob_min_area_ratio": 0.035,
    "error_filter_alpha": 0.60,
    "use_weighted_centroid": True,
    "publish_mask": True,
    "stop_when_lost": False,
    "lost_strategy": "arc_recovery",
    "lost_grace_time": 0.20,
    "lost_reverse_after": 0.75,
    "lost_reverse_duration": 0.55,
    "lost_scan_period": 1.20,
    "lost_forward_pct": 8.0,
    "lost_reverse_pct": -7.0,
    "lost_turn_pct": 30.0,
    "lost_spin_turn_pct": 26.0,
    "lost_safe_stop_after": 2.0,
    "lost_auto_reset_after": 4.5,
    "out_of_bounds_enable": True,
    "out_of_bounds_action": "reset",
    "x_abs_limit": 4.0,
    "y_abs_limit": 3.0,
}

PRESETS = {
    "default": {
        "control_mode": "pd", "kp": 32.0, "ki": 0.0, "kd": 6.0,
        "base_speed_pct": 24.0, "base_min_pct": 12.0, "curve_factor": 0.60,
        "threshold_mode": "otsu", "roi_top_ratio": 0.42, "roi_bottom_ratio": 0.90,
        "roi_left_ratio": 0.05, "roi_right_ratio": 0.95,
        "min_area": 180.0, "min_black_ratio": 0.0015, "max_black_ratio": 0.85, "max_contour_area_ratio": 0.75, "error_filter_alpha": 0.60,
        "blur_kernel": 5, "morph_kernel": 5,
        "lost_strategy": "arc_recovery", "lost_forward_pct": 8.0, "lost_turn_pct": 30.0,
        "lost_scan_period": 1.20, "lost_safe_stop_after": 2.0, "lost_auto_reset_after": 4.5,
    },
    "slow": {
        "control_mode": "pd", "kp": 28.0, "ki": 0.0, "kd": 6.0,
        "base_speed_pct": 20.0, "base_min_pct": 12.0, "curve_factor": 0.65,
        "threshold_mode": "otsu", "roi_top_ratio": 0.44, "roi_bottom_ratio": 0.90,
        "min_area": 140.0, "min_black_ratio": 0.0015, "max_black_ratio": 0.85, "max_contour_area_ratio": 0.75, "error_filter_alpha": 0.70,
        "lost_forward_pct": 8.0, "lost_turn_pct": 28.0,
    },
    "fast": {
        "control_mode": "pd", "kp": 48.0, "ki": 0.0, "kd": 7.5,
        "base_speed_pct": 46.0, "base_min_pct": 20.0, "curve_factor": 0.50,
        "threshold_mode": "otsu", "roi_top_ratio": 0.50, "roi_bottom_ratio": 0.88,
        "min_area": 200.0, "error_filter_alpha": 0.45,
        "lost_forward_pct": 12.0, "lost_turn_pct": 36.0,
    },
    "curve": {
        "control_mode": "pd", "kp": 34.0, "ki": 0.0, "kd": 8.0,
        "base_speed_pct": 26.0, "base_min_pct": 13.0, "curve_factor": 0.80,
        "threshold_mode": "otsu", "roi_top_ratio": 0.42, "roi_bottom_ratio": 0.90,
        "min_area": 150.0, "error_filter_alpha": 0.68,
        "lost_forward_pct": 9.0, "lost_turn_pct": 34.0,
    },
    "recovery": {
        "control_mode": "pd", "kp": 32.0, "ki": 0.0, "kd": 6.0,
        "base_speed_pct": 22.0, "base_min_pct": 12.0, "curve_factor": 0.60,
        "threshold_mode": "otsu", "roi_top_ratio": 0.40, "roi_bottom_ratio": 0.92,
        "roi_left_ratio": 0.00, "roi_right_ratio": 1.00,
        "min_area": 100.0, "min_black_ratio": 0.0010, "max_black_ratio": 0.85, "max_contour_area_ratio": 0.75, "error_filter_alpha": 0.62,
        "lost_strategy": "arc_recovery", "lost_forward_pct": 10.0, "lost_reverse_pct": -9.0,
        "lost_turn_pct": 40.0, "lost_scan_period": 0.90, "lost_safe_stop_after": 2.5, "lost_auto_reset_after": 5.0,
    },
    "fixed_threshold": {
        "threshold_mode": "fixed", "black_threshold": 105,
        "blur_kernel": 5, "morph_kernel": 5,
        "roi_top_ratio": 0.44, "roi_bottom_ratio": 0.90,
        "min_area": 130.0, "error_filter_alpha": 0.65,
    },
}

SLIDERS = [
    {"name":"kp","label":"Kp","min":0,"max":80,"step":0.1,"group":"pid"},
    {"name":"ki","label":"Ki","min":0,"max":5,"step":0.01,"group":"pid"},
    {"name":"kd","label":"Kd","min":0,"max":15,"step":0.01,"group":"pid"},
    {"name":"pos_out_limit","label":"转向限幅 %","min":10,"max":80,"step":1,"group":"pid"},
    {"name":"base_speed_pct","label":"基础速度 %","min":0,"max":80,"step":1,"group":"pid"},
    {"name":"base_min_pct","label":"弯道最低 %","min":0,"max":40,"step":1,"group":"pid"},
    {"name":"curve_factor","label":"弯道降速","min":0,"max":1.5,"step":0.01,"group":"pid"},
    {"name":"max_linear","label":"最大线速度","min":0.1,"max":1.2,"step":0.01,"group":"pid"},
    {"name":"max_angular","label":"最大角速度","min":0.5,"max":5.0,"step":0.01,"group":"pid"},
    {"name":"roi_top_ratio","label":"ROI 上边界","min":0.20,"max":0.90,"step":0.01,"group":"vision"},
    {"name":"roi_bottom_ratio","label":"ROI 下边界","min":0.40,"max":1.00,"step":0.01,"group":"vision"},
    {"name":"roi_left_ratio","label":"ROI 左边界","min":0.00,"max":0.40,"step":0.01,"group":"vision"},
    {"name":"roi_right_ratio","label":"ROI 右边界","min":0.60,"max":1.00,"step":0.01,"group":"vision"},
    {"name":"black_threshold","label":"固定阈值","min":0,"max":255,"step":1,"group":"vision"},
    {"name":"adaptive_block_size","label":"自适应块大小","min":3,"max":99,"step":2,"group":"vision"},
    {"name":"adaptive_c","label":"自适应 C","min":-20,"max":30,"step":1,"group":"vision"},
    {"name":"min_area","label":"最小轮廓面积","min":20,"max":1200,"step":1,"group":"vision"},
    {"name":"min_black_ratio","label":"最小黑色占比","min":0,"max":0.03,"step":0.0005,"group":"vision"},
    {"name":"max_black_ratio","label":"最大黑色占比","min":0.05,"max":0.90,"step":0.01,"group":"vision"},
    {"name":"max_contour_area_ratio","label":"最大轮廓占比","min":0.05,"max":0.95,"step":0.01,"group":"vision"},
    {"name":"round_blob_min_area_ratio","label":"圆形遮挡最小占比","min":0.005,"max":0.20,"step":0.005,"group":"vision"},
    {"name":"error_filter_alpha","label":"误差滤波","min":0,"max":0.95,"step":0.01,"group":"vision"},
    {"name":"blur_kernel","label":"模糊核","min":1,"max":15,"step":2,"group":"vision"},
    {"name":"morph_kernel","label":"形态学核","min":1,"max":15,"step":2,"group":"vision"},
    {"name":"lost_grace_time","label":"前弧搜索时间","min":0,"max":1.5,"step":0.01,"group":"recovery"},
    {"name":"lost_reverse_after","label":"倒车触发时间","min":0.1,"max":2.5,"step":0.01,"group":"recovery"},
    {"name":"lost_reverse_duration","label":"倒车持续时间","min":0,"max":1.5,"step":0.01,"group":"recovery"},
    {"name":"lost_scan_period","label":"左右扫描周期","min":0.3,"max":3.0,"step":0.01,"group":"recovery"},
    {"name":"lost_forward_pct","label":"丢线前进 %","min":0,"max":30,"step":1,"group":"recovery"},
    {"name":"lost_reverse_pct","label":"丢线倒车 %","min":-25,"max":0,"step":1,"group":"recovery"},
    {"name":"lost_turn_pct","label":"丢线转向 %","min":5,"max":60,"step":1,"group":"recovery"},
    {"name":"lost_spin_turn_pct","label":"原地旋转 %","min":5,"max":60,"step":1,"group":"recovery"},
    {"name":"lost_safe_stop_after","label":"丢线安全停车 s","min":0,"max":8,"step":0.1,"group":"recovery"},
    {"name":"lost_auto_reset_after","label":"丢线自动回起点 s","min":0,"max":12,"step":0.1,"group":"recovery"},
    {"name":"x_abs_limit","label":"X 边界 m","min":2.5,"max":8.0,"step":0.1,"group":"recovery"},
    {"name":"y_abs_limit","label":"Y 边界 m","min":2.0,"max":6.0,"step":0.1,"group":"recovery"},
]

HTML = r'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>三轮小车摄像头巡线 ROS 调参面板</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}body{font-family:'Segoe UI','Microsoft YaHei',sans-serif;background:#1a1d23;color:#e0e0e0;min-height:100vh}header{background:linear-gradient(135deg,#1e3a5f,#0d2137);padding:12px 22px;border-bottom:2px solid #2d5a87;display:flex;justify-content:space-between;gap:12px;align-items:center}h1{font-size:1.25rem;color:#64b5f6}.sub{font-size:.78rem;color:#90a4ae;margin-top:3px}.wrap{display:grid;grid-template-columns:minmax(560px,1.2fr) 440px;gap:14px;padding:14px;max-width:1420px;margin:auto}.card{background:#232830;border:1px solid #333840;border-radius:10px;padding:13px;margin-bottom:12px}h3{font-size:.92rem;color:#64b5f6;border-bottom:1px solid #2a3039;padding-bottom:7px;margin-bottom:9px}.summary{display:grid;grid-template-columns:repeat(5,1fr);gap:7px}.box{background:#1a1d23;border-radius:7px;padding:8px;text-align:center}.box .label{font-size:.68rem;color:#78909c}.box .num{font-family:Consolas,monospace;font-size:1.1rem;font-weight:800;margin-top:3px}.pid{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:8px}.imgbox{background:#0f1217;border-radius:8px;padding:8px;text-align:center}.imgbox img{width:100%;max-height:430px;object-fit:contain;border-radius:6px;background:#111}.controls{display:flex;gap:8px;flex-wrap:wrap}.btn{border:0;border-radius:6px;padding:8px 13px;font-weight:700;cursor:pointer;color:#fff;background:#1565c0}.btn:hover{filter:brightness(1.15)}.green{background:#2e7d32}.orange{background:#f57f17}.gray{background:#546e7a}.purple{background:#4527a0}.teal{background:#00695c}.red{background:#b71c1c}.row{display:grid;grid-template-columns:112px 1fr 62px;gap:8px;align-items:center;margin:7px 0}.row label,.selectrow label{color:#b0bec5;font-size:.8rem}input[type=range]{width:100%;accent-color:#42a5f5}.pval{font-family:Consolas,monospace;text-align:right;font-weight:700}.selectrow{display:grid;grid-template-columns:116px 1fr;gap:8px;align-items:center;margin:8px 0}select{background:#1a1d23;color:#d0d7de;border:1px solid #455a64;border-radius:5px;padding:6px}.checkrow{display:flex;align-items:center;gap:8px;color:#b0bec5;font-size:.84rem;margin:8px 0}input[type=checkbox]{accent-color:#66bb6a}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}.hint{font-size:.75rem;color:#90a4ae;line-height:1.55}.state{font-size:.78rem;color:#90a4ae;text-align:right}.err{color:#ff7043}.steer{color:#ffa726}.ok{color:#66bb6a}.bad{color:#ef5350}.blue{color:#42a5f5}.small{font-size:.72rem;color:#78909c;margin-top:5px}@media(max-width:1100px){.wrap{grid-template-columns:1fr}header{flex-direction:column;align-items:flex-start}.summary{grid-template-columns:repeat(2,1fr)}}
</style>
</head>
<body>
<header><div><h1>📷 三轮小车摄像头巡线 — ROS 可视化交互调参</h1><div class="sub">Gazebo 相机 → ROI → 黑线分割 → 质心误差 → PID → 差速驱动；浏览器面板避免 Tkinter / DISPLAY 崩溃</div></div><div class="state" id="conn">连接中...</div></header>
<div class="wrap">
  <main>
    <section class="card"><h3>🎥 摄像头调试图像 /line_follower/debug_image</h3><div class="imgbox"><img id="debugImg" src="/debug.jpg" alt="等待 debug_image..."/></div><div class="small" id="vision">vision: --</div></section>
    <section class="card"><h3>📌 状态摘要</h3><div class="summary">
      <div class="box"><div class="label">在线状态</div><div class="num" id="online">--</div></div>
      <div class="box"><div class="label">error</div><div class="num err" id="error">--</div></div>
      <div class="box"><div class="label">steer</div><div class="num steer" id="steer">--</div></div>
      <div class="box"><div class="label">base</div><div class="num blue" id="base">--</div></div>
      <div class="box"><div class="label">模式</div><div class="num" id="mode">--</div></div>
    </div><div class="pid">
      <div class="box"><div class="label">P 分量</div><div class="num err" id="pout">--</div></div>
      <div class="box"><div class="label">I 分量</div><div class="num ok" id="iout">--</div></div>
      <div class="box"><div class="label">D 分量</div><div class="num blue" id="dout">--</div></div>
    </div><div class="hint" style="margin-top:8px" id="wheel">左轮: -- ｜ 右轮: -- ｜ controller: --</div></section>
    <section class="card"><h3>🕹️ 操作</h3><div class="controls">
      <button class="btn green" onclick="action('start')">▶ 启动巡线</button>
      <button class="btn orange" onclick="action('pause')">⏸ 暂停停车</button>
      <button class="btn gray" onclick="action('reset_world')">🔄 重置仿真</button>
      <button class="btn teal" onclick="action('reset_pid')">🧹 清空 PID</button>
    </div><div class="hint" style="margin-top:8px">重置仿真调用 /gazebo/reset_world；清空 PID 会发布 /line_follower/reset_pid。</div></section>
  </main>
  <aside>
    <section class="card"><h3>🎛️ 模式与开关</h3>
      <div class="selectrow"><label>控制模式</label><select id="control_mode" onchange="setParam('control_mode',this.value)"><option>p</option><option>pd</option><option>pi</option><option>pid</option></select></div>
      <div class="selectrow"><label>阈值模式</label><select id="threshold_mode" onchange="setParam('threshold_mode',this.value)"><option>otsu</option><option>fixed</option><option>adaptive</option></select></div>
      <div class="selectrow"><label>丢线恢复</label><select id="lost_strategy" onchange="setParam('lost_strategy',this.value)"><option>arc_recovery</option><option>spin</option><option>stop</option></select></div>
      <div class="selectrow"><label>越界动作</label><select id="out_of_bounds_action" onchange="setParam('out_of_bounds_action',this.value)"><option>reset</option><option>stop</option></select></div>
      <label class="checkrow"><input type="checkbox" id="enabled" onchange="setParam('enabled',this.checked)">启用巡线</label>
      <label class="checkrow"><input type="checkbox" id="out_of_bounds_enable" onchange="setParam('out_of_bounds_enable',this.checked)">启用越界保护</label>
      <label class="checkrow"><input type="checkbox" id="use_weighted_centroid" onchange="setParam('use_weighted_centroid',this.checked)">使用加权质心</label>
          <label class="checkrow"><input type="checkbox" id="reject_round_blob" onchange="setParam('reject_round_blob',this.checked)">过滤近圆形自车遮挡</label>
      <label class="checkrow"><input type="checkbox" id="publish_mask" onchange="setParam('publish_mask',this.checked)">发布 mask 图像</label>
    </section>
    <section class="card"><h3>📋 参数预设</h3><div class="grid2">
      <button class="btn purple" onclick="preset('default')">默认稳态</button>
      <button class="btn purple" onclick="preset('slow')">慢速保守</button>
      <button class="btn purple" onclick="preset('fast')">快速激进</button>
      <button class="btn purple" onclick="preset('curve')">弯道稳定</button>
      <button class="btn purple" onclick="preset('recovery')">找线强化</button>
      <button class="btn purple" onclick="preset('fixed_threshold')">固定阈值</button>
    </div></section>
    <section class="card"><h3>① PID 与速度</h3><div id="pidSliders"></div></section>
    <section class="card"><h3>② 摄像头识别</h3><div id="visionSliders"></div></section>
    <section class="card"><h3>③ 丢线恢复</h3><div id="recoverySliders"></div></section>
    <section class="card"><h3>调参提示</h3><div class="hint">
      1. 先点“慢速保守”，确认 debug 图像里 ROI 能框住黑线。<br>
      2. 黑线识别不稳：先调 ROI，再调阈值模式、min_area 和“最大轮廓占比”。<br>
      2.1 如果画面中心有大黑圆，那是相机拍到自车部件，应调小 ROI 下边界或使用本修正版的前置相机。<br>
      3. 弯道冲出去：降低基础速度，增大 Kd 或弯道降速。<br>
      4. 来回摆动：降低 Kp 或增大误差滤波。<br>
      5. 丢线找不回：点“找线强化”，但保留“丢线安全停车/自动回起点”，防止小车跑出 Gazebo 视野。
    </div></section>
  </aside>
</div>
<script>
const sliders = __SLIDERS__;
let params = {};
function fmt(v){ if(typeof v==='boolean') return v?'true':'false'; if(typeof v==='number') return Math.abs(v)<2 ? v.toFixed(3) : v.toFixed(2); return v; }
function makeSlider(s){return `<div class="row"><label>${s.label}</label><input id="${s.name}" type="range" min="${s.min}" max="${s.max}" step="${s.step}" oninput="slide('${s.name}',this.value)"><div class="pval" id="${s.name}_v">--</div></div>`}
function build(){
  document.getElementById('pidSliders').innerHTML=sliders.filter(s=>s.group==='pid').map(makeSlider).join('');
  document.getElementById('visionSliders').innerHTML=sliders.filter(s=>s.group==='vision').map(makeSlider).join('');
  document.getElementById('recoverySliders').innerHTML=sliders.filter(s=>s.group==='recovery').map(makeSlider).join('');
}
function slide(name,value){ let v=parseFloat(value); setParam(name,v); const el=document.getElementById(name+'_v'); if(el) el.textContent=fmt(v); }
async function setParam(name,value){
  try{ await fetch('/api/set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name,value})}); }
  catch(e){ console.error(e); }
}
async function preset(name){ await fetch('/api/preset',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name})}); await refresh(); }
async function action(name){ await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name})}); await refresh(); }
function setText(id,t){ const e=document.getElementById(id); if(e) e.textContent=t; }
function setParamWidgets(p){
  params=p;
  for(const s of sliders){ const e=document.getElementById(s.name), val=document.getElementById(s.name+'_v'); if(e && p[s.name]!==undefined){ e.value=p[s.name]; if(val) val.textContent=fmt(Number(p[s.name])); } }
  ['control_mode','threshold_mode','lost_strategy','out_of_bounds_action'].forEach(k=>{const e=document.getElementById(k); if(e && p[k]!==undefined)e.value=p[k];});
  ['enabled','use_weighted_centroid','reject_round_blob','publish_mask','out_of_bounds_enable'].forEach(k=>{const e=document.getElementById(k); if(e && p[k]!==undefined)e.checked=!!p[k];});
}
async function refresh(){
  try{
    const r=await fetch('/api/status?ts='+Date.now()); const d=await r.json();
    setParamWidgets(d.params || {});
    const enabled=!!(d.params && d.params.enabled);
    setText('conn','已连接  '+new Date().toLocaleTimeString());
    setText('online', enabled ? (d.online?'● 在线':'✗ 丢线') : '⏸ 暂停');
    document.getElementById('online').className='num '+(enabled?(d.online?'ok':'bad'):'steer');
    setText('error',(d.error>=0?'+':'')+Number(d.error||0).toFixed(3));
    setText('steer',(d.steer>=0?'+':'')+Number(d.steer||0).toFixed(1));
    let w=d.wheel||[0,0,0], pid=d.pid||[0,0,0]; while(w.length<3)w.push(0); while(pid.length<3)pid.push(0);
    setText('base',Number(w[2]||0).toFixed(1)+'%'); setText('mode',params.control_mode||'--');
    setText('pout',(pid[0]>=0?'+':'')+Number(pid[0]||0).toFixed(2));
    setText('iout',(pid[1]>=0?'+':'')+Number(pid[1]||0).toFixed(2));
    setText('dout',(pid[2]>=0?'+':'')+Number(pid[2]||0).toFixed(2));
    setText('wheel',`左轮: ${Number(w[0]||0).toFixed(1)}% ｜ 右轮: ${Number(w[1]||0).toFixed(1)}% ｜ controller: ${d.state||'--'}`);
    setText('vision','vision: '+(d.vision||'--'));
  }catch(e){ setText('conn','连接失败：'+e); }
}
function refreshImg(){ const img=document.getElementById('debugImg'); img.src='/debug.jpg?ts='+Date.now(); }
build(); refresh(); setInterval(refresh,250); setInterval(refreshImg,350);
</script>
</body>
</html>'''

HTML = HTML.replace("__SLIDERS__", json.dumps(SLIDERS, ensure_ascii=False))


class CameraWebTuner(object):
    def __init__(self):
        rospy.init_node("camera_tuner_web_node", anonymous=False)
        self.port = int(rospy.get_param("~port", 8091))
        self.host = str(rospy.get_param("~host", "0.0.0.0"))
        self.open_browser = bool(rospy.get_param("~open_browser", False))
        self.lock = threading.Lock()
        self.latest = {"online": False, "error": 0.0, "steer": 0.0, "pid": [0.0, 0.0, 0.0], "wheel": [0.0, 0.0, 0.0], "state": "waiting...", "vision": "waiting..."}
        self.jpeg_lock = threading.Lock()
        self.last_jpeg = None
        self.bridge = CvBridge() if CvBridge is not None else None

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.reset_pid_pub = rospy.Publisher("/line_follower/reset_pid", Empty, queue_size=1)
        rospy.Subscriber("/line_follower/online", Bool, lambda m: self._set("online", bool(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/error", Float32, lambda m: self._set("error", float(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/steer", Float32, lambda m: self._set("steer", float(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/pid_terms", Float32MultiArray, lambda m: self._set("pid", list(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/wheel_pwm", Float32MultiArray, lambda m: self._set("wheel", list(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/state", String, lambda m: self._set("state", str(m.data)), queue_size=1)
        rospy.Subscriber("/line_follower/vision_state", String, lambda m: self._set("vision", str(m.data)), queue_size=1)
        if self.bridge is not None and cv2 is not None:
            rospy.Subscriber("/line_follower/debug_image", Image, self.debug_image_cb, queue_size=1, buff_size=2**24)
        else:
            rospy.logwarn("cv_bridge/cv2 unavailable: web image preview disabled, parameters still work")
        self.server = None

    def _set(self, key, value):
        with self.lock:
            self.latest[key] = value

    def debug_image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, enc = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 72])
            if ok:
                with self.jpeg_lock:
                    self.last_jpeg = enc.tobytes()
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "debug image conversion failed: %s", exc)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "debug image JPEG encode failed: %s", exc)

    def params(self):
        return {k: rospy.get_param("/" + k, v) for k, v in PARAM_DEFAULTS.items()}

    def status(self):
        with self.lock:
            data = dict(self.latest)
        data["params"] = self.params()
        data["time"] = time.time()
        return data

    def set_param(self, name, value):
        if name not in PARAM_DEFAULTS:
            raise ValueError("unknown param: %s" % name)
        default = PARAM_DEFAULTS[name]
        if isinstance(default, bool):
            value = bool(value)
        elif isinstance(default, int) and not isinstance(default, bool):
            value = int(round(float(value)))
            if name in ("blur_kernel", "morph_kernel"):
                value = max(1, value)
                if value > 1 and value % 2 == 0:
                    value += 1
            if name == "adaptive_block_size":
                value = max(3, value)
                if value % 2 == 0:
                    value += 1
        elif isinstance(default, float):
            value = float(value)
        else:
            value = str(value)
        rospy.set_param("/" + name, value)
        return value

    def apply_preset(self, name):
        if name not in PRESETS:
            raise ValueError("unknown preset: %s" % name)
        for k, v in PRESETS[name].items():
            self.set_param(k, v)
        self.reset_pid_pub.publish(Empty())
        return self.params()

    def action(self, name):
        if name == "start":
            self.set_param("enabled", True)
            return "started"
        if name == "pause":
            self.set_param("enabled", False)
            self.cmd_pub.publish(Twist())
            return "paused"
        if name == "reset_pid":
            self.reset_pid_pub.publish(Empty())
            return "pid reset requested"
        if name == "reset_world":
            rospy.wait_for_service("/gazebo/reset_world", timeout=2.0)
            srv = rospy.ServiceProxy("/gazebo/reset_world", EmptySrv)
            srv()
            self.set_param("enabled", True)
            self.reset_pid_pub.publish(Empty())
            return "world reset requested"
        raise ValueError("unknown action: %s" % name)

    def make_handler(self):
        parent = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                rospy.logdebug("web_tuner: " + fmt, *args)
            def _send(self, code, content, ctype="application/json; charset=utf-8"):
                if isinstance(content, str):
                    content = content.encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(content)
            def _json(self, code, obj):
                self._send(code, json.dumps(obj, ensure_ascii=False), "application/json; charset=utf-8")
            def _read_json(self):
                n = int(self.headers.get("Content-Length", "0"))
                if n <= 0:
                    return {}
                return json.loads(self.rfile.read(n).decode("utf-8"))
            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/":
                    return self._send(200, HTML, "text/html; charset=utf-8")
                if path == "/api/status":
                    return self._json(200, parent.status())
                if path == "/debug.jpg":
                    with parent.jpeg_lock:
                        data = parent.last_jpeg
                    if data is None:
                        return self._send(404, "debug image not ready", "text/plain; charset=utf-8")
                    return self._send(200, data, "image/jpeg")
                return self._send(404, "not found", "text/plain; charset=utf-8")
            def do_POST(self):
                try:
                    path = urlparse(self.path).path
                    body = self._read_json()
                    if path == "/api/set":
                        name = body.get("name")
                        value = parent.set_param(name, body.get("value"))
                        return self._json(200, {"ok": True, "name": name, "value": value})
                    if path == "/api/preset":
                        return self._json(200, {"ok": True, "params": parent.apply_preset(str(body.get("name", "")))})
                    if path == "/api/action":
                        return self._json(200, {"ok": True, "result": parent.action(str(body.get("name", "")))})
                    return self._json(404, {"ok": False, "error": "not found"})
                except Exception as exc:
                    rospy.logwarn("web tuner request failed: %s", exc)
                    return self._json(400, {"ok": False, "error": str(exc)})
        return Handler

    def serve(self):
        self.server = ThreadingHTTPServer((self.host, self.port), self.make_handler())
        rospy.loginfo("HTML-style camera line follower tuner is running: http://127.0.0.1:%d/", self.port)
        if self.open_browser:
            threading.Timer(1.0, lambda: webbrowser.open("http://127.0.0.1:%d/" % self.port)).start()
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()
        rospy.spin()
        try:
            self.server.shutdown()
        except Exception:
            pass


def main():
    CameraWebTuner().serve()


if __name__ == "__main__":
    main()
