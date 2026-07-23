/* UAV_yolo 地面站前端：輪詢狀態 + 四個分頁 */

const $ = (sel) => document.querySelector(sel);
const api = async (path, opts) => {
  const res = await fetch(path, opts);
  return res.json();
};
const post = (path, body) =>
  api(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });

/* ---------------- 分頁切換 ---------------- */
document.querySelectorAll(".tab").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".view").forEach((v) => v.classList.remove("active"));
    btn.classList.add("active");
    $("#view-" + btn.dataset.view).classList.add("active");
    if (btn.dataset.view === "checklist") loadChecklist();
    if (btn.dataset.view === "settings") loadSettings();
    if (btn.dataset.view === "calib") refreshCalibStatus();
  });
});

/* ---------------- 儀表板：狀態輪詢 ---------------- */
let lastStatus = null;

function fmt(v, digits = 1) {
  return v === null || v === undefined ? "—" : Number(v).toFixed(digits);
}

function kvRow(k, v, cls = "") {
  return `<span class="k">${k}</span><span class="v ${cls}">${v}</span>`;
}

async function poll() {
  try {
    const st = await api("/api/status");
    lastStatus = st;
    renderStatus(st);
  } catch (e) {
    $("#badge-state").textContent = "連線中斷";
    $("#badge-state").className = "badge st-LOST";
  }
  setTimeout(poll, 500);
}

function renderStatus(st) {
  if (!st.ready) {
    $("#badge-state").textContent = "引擎啟動中…";
    return;
  }
  const modeBadge = $("#badge-mode");
  modeBadge.textContent = st.sim ? "模擬模式" : "實機模式";
  modeBadge.className = "badge" + (st.sim ? " sim" : "");

  const stateBadge = $("#badge-state");
  const stateNames = { SEARCH: "搜尋中", TRACK: "追蹤中", COAST: "盲區外推", LOST: "目標丟失" };
  stateBadge.textContent =
    (st.airframe === "fixedwing" ? "定翼" : "旋翼") + "・" + (stateNames[st.state] || st.state);
  stateBadge.className = "badge st-" + st.state;

  // 導引開關
  const btn = $("#btn-guidance");
  btn.textContent = st.guidance_enabled ? "導引：啟用中（點擊關閉）" : "導引：關閉（點擊啟用）";
  btn.className = "big-btn " + (st.guidance_enabled ? "on" : "off");

  // 閘門
  const gates = $("#gate-list");
  if (st.guidance_enabled && st.gates.length === 0) {
    gates.innerHTML = `<div class="gate-item ok">✓ 全部閘門通過，指令發送中</div>`;
  } else if (st.gates.length) {
    gates.innerHTML = st.gates.map((g) => `<div class="gate-item">✗ ${g}</div>`).join("");
  } else {
    gates.innerHTML = "";
  }

  // 最後指令
  const lc = st.last_command;
  $("#last-cmd").textContent = lc
    ? `最後指令 ${lc.label}：(${fmt(lc.lat, 6)}, ${fmt(lc.lon, 6)}) 高度 ${fmt(lc.alt_rel, 0)}m` +
      (lc.radius ? ` 半徑 ${fmt(lc.radius, 0)}m` : "") + `（${fmt(lc.age_s, 0)} 秒前）`
    : "尚未發送任何導引指令";

  // 目標
  const t = st.target;
  let tHtml = "";
  if (t.initialized) {
    tHtml += kvRow("座標", `${fmt(t.lat, 6)}, ${fmt(t.lon, 6)}`);
    tHtml += kvRow("速度", `${fmt(t.speed)} m/s`);
    tHtml += kvRow("位置 (N,E)", `${fmt(t.n, 0)}, ${fmt(t.e, 0)} m`);
    tHtml += kvRow("量測時間差", `${fmt(t.age_s)} s`, t.age_s > 2 ? "bad" : "good");
    tHtml += kvRow("不確定度", `±${fmt(t.pos_std)} m`);
  } else {
    tHtml += kvRow("狀態", "尚未鎖定目標", "");
  }
  $("#target-info").innerHTML = tHtml;

  // 偵測列表
  $("#det-list").innerHTML = st.detections.length
    ? st.detections
        .map(
          (d) =>
            `<div class="det-item ${d.locked ? "locked" : ""}" data-id="${d.id}">
               <span>#${d.id}</span><span>${d.cls}</span><span>${(d.conf * 100).toFixed(0)}%</span>
               ${d.locked ? "<span>🔒 已鎖定</span>" : ""}
             </div>`
        )
        .join("")
    : `<div class="mut small">畫面中沒有偵測到目標</div>`;
  document.querySelectorAll(".det-item").forEach((el) =>
    el.addEventListener("click", () => post("/api/lock", { track_id: Number(el.dataset.id) }))
  );

  // 載具
  const v = st.vehicle;
  $("#vehicle-info").innerHTML =
    kvRow("模式", v.mode || "—", v.mode === "AUTO.LOITER" ? "good" : "") +
    kvRow("Arm", v.armed ? "已解鎖" : "未解鎖", v.armed ? "good" : "") +
    kvRow("數傳", v.link_ok ? "正常" : "斷線", v.link_ok ? "good" : "bad") +
    kvRow("高度(相對)", v.rel_alt !== null && v.rel_alt !== undefined ? fmt(v.rel_alt, 0) + " m" : "—") +
    kvRow("Home", v.home_set ? "已取得" : "未取得", v.home_set ? "good" : "bad") +
    kvRow("雲台", st.gimbal.present ? `${st.gimbal.control}${st.gimbal.has_feedback ? "・有回報" : ""}` : "無") +
    kvRow("處理速率", fmt(st.loop_hz, 0) + " Hz");

  // 影像資訊
  $("#video-label").textContent = st.video.device || "影像";
  $("#video-fps").textContent = st.video.connected ? `${fmt(st.video.fps, 0)} FPS` : "未連線";
  const ve = $("#video-error");
  const err = st.video.error || st.mavlink.error;
  ve.hidden = !err;
  if (err) ve.textContent = err;

  drawMap(st);
}

/* ---------------- 俯視圖 ---------------- */
function drawMap(st) {
  const canvas = $("#map");
  const ctx = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  const vp = st.vehicle.path || [];
  const tp = st.target.path || [];
  const pts = [...vp, ...tp];
  if (st.last_command) pts.push([st.last_command.n, st.last_command.e]);
  if (!pts.length) {
    ctx.fillStyle = "#a99c88";
    ctx.font = "15px sans-serif";
    ctx.fillText("等待資料…", W / 2 - 35, H / 2);
    return;
  }

  let minN = Math.min(...pts.map((p) => p[0])), maxN = Math.max(...pts.map((p) => p[0]));
  let minE = Math.min(...pts.map((p) => p[1])), maxE = Math.max(...pts.map((p) => p[1]));
  const pad = 30;
  const spanN = Math.max(maxN - minN, 40), spanE = Math.max(maxE - minE, 40);
  const scale = Math.min((W - 2 * pad) / spanE, (H - 2 * pad) / spanN);
  const cx = (minE + maxE) / 2, cy = (minN + maxN) / 2;
  const X = (e) => W / 2 + (e - cx) * scale;
  const Y = (n) => H / 2 - (n - cy) * scale;

  // 比例尺
  const scaleM = Math.round(50 / scale) * (scale > 1 ? 1 : 1);
  ctx.strokeStyle = "#c9bda6"; ctx.fillStyle = "#a99c88"; ctx.font = "12px sans-serif";
  ctx.beginPath(); ctx.moveTo(10, H - 12); ctx.lineTo(10 + 50, H - 12); ctx.stroke();
  ctx.fillText(`${scaleM} m`, 14, H - 18);

  const drawPath = (path, color) => {
    if (path.length < 2) return;
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
    path.forEach((p, i) => (i ? ctx.lineTo(X(p[1]), Y(p[0])) : ctx.moveTo(X(p[1]), Y(p[0]))));
    ctx.stroke();
  };
  drawPath(vp, "#4a7191");
  drawPath(tp, "#b5493f");

  const drawDot = (n, e, color, r = 5) => {
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(X(e), Y(n), r, 0, Math.PI * 2); ctx.fill();
  };
  if (vp.length) drawDot(vp[vp.length - 1][0], vp[vp.length - 1][1], "#4a7191", 6);
  if (tp.length) drawDot(tp[tp.length - 1][0], tp[tp.length - 1][1], "#b5493f", 6);

  if (st.last_command) {
    const lc = st.last_command;
    drawDot(lc.n, lc.e, "#b8863b", 5);
    if (lc.radius) {
      ctx.strokeStyle = "#b8863b"; ctx.setLineDash([6, 5]); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(X(lc.e), Y(lc.n), lc.radius * scale, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
    }
  }
}

/* ---------------- 導引開關 / 解鎖 ---------------- */
$("#btn-guidance").addEventListener("click", async () => {
  const enabled = lastStatus && lastStatus.guidance_enabled;
  if (!enabled) {
    if (!confirm("確認啟用導引？系統將開始向飛控發送位置指令（載具需在 AUTO.LOITER/Hold 模式）。")) return;
  }
  await post("/api/guidance", { enabled: !enabled });
});
$("#btn-unlock").addEventListener("click", () => post("/api/unlock"));

/* ---------------- 檢查清單 ---------------- */
async function loadChecklist() {
  const data = await api("/api/checklist");
  $("#cl-bar").style.width = data.total ? (100 * data.done) / data.total + "%" : "0";
  $("#cl-count").textContent = `${data.done} / ${data.total}`;

  const groups = {};
  data.items.forEach((it) => (groups[it.group] = groups[it.group] || []).push(it));

  $("#cl-groups").innerHTML = Object.entries(groups)
    .map(
      ([group, items]) => `
      <div class="cl-group">
        <div class="cl-group-title">${group}</div>
        ${items
          .map(
            (it) => `
          <div class="cl-item ${it.done ? "done" : ""}">
            <input type="checkbox" id="cb-${it.id}" ${it.done ? "checked" : ""} data-id="${it.id}">
            <label for="cb-${it.id}">${it.text}</label>
            ${it.custom ? `<button class="cl-del" data-del="${it.id}" title="刪除">✕</button>` : ""}
          </div>`
          )
          .join("")}
        <div class="cl-add">
          <input type="text" placeholder="＋ 新增項目到「${group}」，按 Enter" data-group="${group}">
        </div>
      </div>`
    )
    .join("");

  document.querySelectorAll(".cl-item input[type=checkbox]").forEach((cb) =>
    cb.addEventListener("change", async () => {
      await post("/api/checklist/toggle", { id: cb.dataset.id });
      loadChecklist();
    })
  );
  document.querySelectorAll(".cl-del").forEach((btn) =>
    btn.addEventListener("click", async () => {
      await post("/api/checklist/remove", { id: btn.dataset.del });
      loadChecklist();
    })
  );
  document.querySelectorAll(".cl-add input").forEach((inp) =>
    inp.addEventListener("keydown", async (e) => {
      if (e.key === "Enter" && inp.value.trim()) {
        await post("/api/checklist/add", { group: inp.dataset.group, text: inp.value.trim() });
        loadChecklist();
      }
    })
  );
}
$("#cl-uncheck").addEventListener("click", async () => {
  await post("/api/checklist/uncheck_all");
  loadChecklist();
});
$("#cl-reset").addEventListener("click", async () => {
  if (confirm("還原成預設清單？自訂項目會被刪除。")) {
    await post("/api/checklist/reset");
    loadChecklist();
  }
});

/* ---------------- 系統設定 ---------------- */
const FIELDS = [
  { sec: "運作模式" },
  { path: "system.mode", label: "模式", type: "seg", options: [["sim", "模擬"], ["live", "實機"]],
    hint: "模擬＝無硬體演練（合成影像+合成飛控）；實機＝接採集卡與數傳 ⟳" },
  { sec: "載具" },
  { path: "vehicle.airframe", label: "載體種類", type: "seg",
    options: [["multirotor", "四軸旋翼"], ["fixedwing", "固定翼"]],
    hint: "決定導引方式：旋翼＝跟隨懸停；定翼＝standoff 繞行 ⟳" },
  { sec: "雲台" },
  { path: "gimbal.present", label: "有裝雲台", type: "bool", hint: "C-20T 三軸雲台 ⟳" },
  { path: "gimbal.control", label: "控制方式", type: "select",
    options: [["roi", "ROI（飛控自動指向，建議）"], ["pitchyaw", "直接下角度"], ["none", "不控制（固定安裝）"]],
    hint: "ROI 需 PX4 設 MNT_MODE_IN=4 ⟳" },
  { path: "gimbal.mount_deg.pitch", label: "固定安裝俯角(度)", type: "number",
    hint: "無雲台時相機朝向：-90=垂直朝下（僅 control=none 用） ⟳" },
  { sec: "影像來源" },
  { path: "video.source", label: "來源", type: "select",
    options: [["uvc", "HDMI 採集卡 / USB 相機（建議）"], ["rtsp", "RTSP / IP Cam"], ["obs", "OBS 虛擬相機"], ["file", "影片檔（測試）"]],
    hint: "採集卡直開比 OBS 少一段延遲 ⟳" },
  { path: "video.uvc_name_hint", label: "採集卡名稱關鍵字", type: "text", hint: "留空用索引；可從下方「裝置清單」複製 ⟳" },
  { path: "video.uvc_index", label: "採集卡索引", type: "number", hint: "名稱留空時用 ⟳" },
  { path: "video.rtsp_url", label: "RTSP 網址", type: "text", hint: "例 rtsp://192.168.144.25:8554/main ⟳" },
  { sec: "偵測" },
  { path: "detector.conf", label: "信心閾值", type: "number", step: 0.05, hint: "0.3~0.7，低=多框、高=少框" },
  { path: "detector.lock_mode", label: "鎖定方式", type: "select",
    options: [["auto", "自動（最大目標連續幀）"], ["manual", "手動（UI 點選）"]] },
  { path: "detector.min_lock_frames", label: "鎖定所需連續幀", type: "number" },
  { sec: "導引（旋翼）" },
  { path: "guidance.multirotor.follow_alt_m", label: "跟隨高度 m", type: "number" },
  { path: "guidance.multirotor.standoff_m", label: "水平退距 m", type: "number", hint: "0=目標正上方" },
  { sec: "導引（固定翼）" },
  { path: "guidance.fixedwing.orbit_radius_m", label: "繞行半徑 m", type: "number", hint: "須大於最小轉彎半徑；同步設 PX4 NAV_LOITER_RAD" },
  { path: "guidance.fixedwing.alt_m", label: "繞行高度 m", type: "number" },
  { path: "guidance.fixedwing.lead_time_s", label: "預測前置秒數", type: "number" },
  { sec: "安全" },
  { path: "safety.max_cmd_distance_m", label: "距 Home 圍欄 m", type: "number" },
  { path: "safety.min_cmd_alt_m", label: "指令高度下限 m", type: "number" },
  { path: "safety.max_cmd_alt_m", label: "指令高度上限 m", type: "number" },
  { sec: "MAVLink 數傳" },
  { path: "mavlink.port", label: "COM 埠", type: "text", hint: "例 COM3 ⟳" },
  { path: "mavlink.baud", label: "鮑率", type: "number", hint: "SiK 電台常見 57600 ⟳" },
];

function getPath(obj, path) {
  return path.split(".").reduce((o, k) => (o == null ? undefined : o[k]), obj);
}

async function loadSettings() {
  const data = await api("/api/config");
  const cfg = data.config;
  const form = $("#settings-form");
  form.innerHTML = "";
  let section = null;

  FIELDS.forEach((f) => {
    if (f.sec) {
      section = document.createElement("div");
      section.className = "cfg-section";
      section.innerHTML = `<h3>${f.sec}</h3>`;
      form.appendChild(section);
      return;
    }
    const value = getPath(cfg, f.path);
    const row = document.createElement("div");
    row.className = "cfg-row";
    let ctrl = "";
    if (f.type === "seg") {
      ctrl = `<div class="seg" data-path="${f.path}">${f.options
        .map(([v, l]) => `<button data-val="${v}" class="${v === value ? "sel" : ""}">${l}</button>`)
        .join("")}</div>`;
    } else if (f.type === "select") {
      ctrl = `<select data-path="${f.path}">${f.options
        .map(([v, l]) => `<option value="${v}" ${v === value ? "selected" : ""}>${l}</option>`)
        .join("")}</select>`;
    } else if (f.type === "bool") {
      ctrl = `<input type="checkbox" data-path="${f.path}" ${value ? "checked" : ""}>`;
    } else if (f.type === "number") {
      ctrl = `<input type="number" data-path="${f.path}" value="${value ?? ""}" step="${f.step || "any"}">`;
    } else {
      ctrl = `<input type="text" data-path="${f.path}" value="${value ?? ""}">`;
    }
    row.innerHTML = `<label>${f.label}</label>${ctrl}` + (f.hint ? `<div class="hint">${f.hint}</div>` : "");
    section.appendChild(row);
  });

  document.querySelectorAll(".seg button").forEach((btn) =>
    btn.addEventListener("click", () => {
      btn.parentElement.querySelectorAll("button").forEach((b) => b.classList.remove("sel"));
      btn.classList.add("sel");
    })
  );

  const meta = data.meta;
  $("#cfg-meta").innerHTML =
    `<b>狀態</b>：權重 ${meta.weights_exists ? "✓ weights/best.pt" : "✗ 未找到（將用 COCO 預訓練 yolo26n）"}｜` +
    `相機校正 ${meta.calibrated ? "✓ 已完成" : "✗ 未校正（用 FOV 近似，精度較差）"}<br>` +
    `<b>影像裝置清單</b>：${meta.devices.length ? meta.devices.map((d, i) => `[${i}] ${d}`).join("、") : "（偵測不到 / 非 Windows）"}`;
}

function collectPatch() {
  const patch = {};
  const setPath = (path, value) => {
    const parts = path.split(".");
    let node = patch;
    parts.slice(0, -1).forEach((k) => (node = node[k] = node[k] || {}));
    node[parts[parts.length - 1]] = value;
  };
  document.querySelectorAll("#settings-form [data-path]").forEach((el) => {
    const path = el.dataset.path;
    if (el.classList.contains("seg")) {
      const sel = el.querySelector("button.sel");
      if (sel) setPath(path, sel.dataset.val);
    } else if (el.type === "checkbox") {
      setPath(path, el.checked);
    } else if (el.type === "number") {
      if (el.value !== "") setPath(path, Number(el.value));
    } else {
      setPath(path, el.value);
    }
  });
  return patch;
}

$("#cfg-save").addEventListener("click", async () => {
  const res = await api("/api/config", {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(collectPatch()),
  });
  $("#cfg-msg").textContent = res.note || "已儲存";
});

$("#cfg-restart").addEventListener("click", async () => {
  $("#cfg-msg").textContent = "重啟引擎中…";
  await post("/api/restart");
  $("#cfg-msg").textContent = "引擎已用最新設定重啟 ✓";
  loadSettings();
});

/* ---------------- 相機校正 ---------------- */
async function refreshCalibStatus() {
  const st = await api("/api/calib/status");
  const box = $("#calib-status");
  if (!st.active) {
    box.innerHTML = kvRow("狀態", "未開始（按「開始」建立校正工作）");
    $("#calib-capture").disabled = true;
    $("#calib-compute").disabled = true;
    $("#calib-save").disabled = true;
    return;
  }
  box.innerHTML =
    kvRow("棋盤內角點", `${st.board[0]} × ${st.board[1]}`) +
    kvRow("已擷取", `${st.count} 張`, st.count >= 15 ? "good" : "") +
    kvRow("上次擷取", st.last_found ? "✓ 找到棋盤" : "✗ 沒找到棋盤", st.last_found ? "good" : "bad");
  $("#calib-capture").disabled = false;
  $("#calib-compute").disabled = st.count < 8;
  $("#calib-save").disabled = !st.has_result;
}

$("#calib-start").addEventListener("click", async () => {
  await post("/api/calib/start", {
    cols: Number($("#cb-cols").value),
    rows: Number($("#cb-rows").value),
    square_mm: Number($("#cb-square").value),
  });
  $("#calib-result").hidden = true;
  refreshCalibStatus();
});

$("#calib-capture").addEventListener("click", async () => {
  await post("/api/calib/capture");
  refreshCalibStatus();
});

$("#calib-compute").addEventListener("click", async () => {
  const res = await post("/api/calib/compute");
  const box = $("#calib-result");
  box.hidden = false;
  if (!res.ok) {
    box.textContent = "計算失敗：" + res.error;
  } else {
    box.innerHTML =
      `<b>RMS 重投影誤差</b>：${res.rms} px ${res.rms < 1 ? "✓ 佳" : "（>1px，建議補拍再算）"}<br>` +
      `<b>水平 FOV</b>：${res.hfov_deg}°｜fx=${res.fx} fy=${res.fy} cx=${res.cx} cy=${res.cy}<br>` +
      `<b>畸變係數</b>：[${res.dist.join(", ")}]`;
  }
  refreshCalibStatus();
});

$("#calib-save").addEventListener("click", async () => {
  const res = await post("/api/calib/save");
  if (res.ok) {
    $("#calib-result").innerHTML += `<br><b>✓ 已存檔</b> ${res.path}（${res.note}）`;
  }
  refreshCalibStatus();
});

/* ---------------- 啟動 ---------------- */
poll();
