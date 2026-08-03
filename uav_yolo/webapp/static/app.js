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

// 後端拒絕（409/400 帶 {ok:false, error}）要讓操作員看到原因，
// 不能讓「點了沒反應」自己猜。鎖定/解鎖等操作共用。
async function postOrExplain(path, body) {
  const res = await post(path, body);
  if (res && res.ok === false && res.error) {
    const b = $("#badge-state");
    const keepText = b.textContent, keepCls = b.className;
    b.textContent = `⚠ ${res.error}`;
    b.className = "badge st-LOST";
    setTimeout(() => { b.textContent = keepText; b.className = keepCls; }, 2500);
  }
  return res;
}

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

// 伺服器 UI 指紋：伺服器更新重啟後指紋會變 → 自動整頁重載，
// 永遠不會再出現「伺服器是新的、瀏覽器跑舊 JS」的狀況。
let uiVersionSeen = null;
function checkUiVersion(st) {
  if (!st.ui_version) return;
  if (uiVersionSeen === null) {
    uiVersionSeen = st.ui_version;
  } else if (st.ui_version !== uiVersionSeen) {
    location.reload();
  }
}

function fmt(v, digits = 1) {
  return v === null || v === undefined ? "—" : Number(v).toFixed(digits);
}

function kvRow(k, v, cls = "") {
  return `<span class="k">${k}</span><span class="v ${cls}">${v}</span>`;
}

// 引擎凍結偵測：/api/status 永遠回 200，引擎迴圈卡死時大多數欄位「本來就
// 常常不動」，操作員分不出來。唯一可靠的訊號是發布序號 seq 停止前進
// （引擎正常時即使沒影像也 ≥2Hz 發布）。
let lastSeq = null, lastSeqAt = 0;
async function poll() {
  try {
    const st = await api("/api/status");
    checkUiVersion(st);
    lastStatus = st;
    renderStatus(st);
    if (st.ready) {
      const nowT = Date.now();
      if (st.seq !== lastSeq) {
        lastSeq = st.seq; lastSeqAt = nowT;
      } else if (nowT - lastSeqAt > 2000) {
        const b = $("#badge-state");
        b.textContent = `⚠ 引擎迴圈凍結（狀態 ${Math.round((nowT - lastSeqAt) / 1000)}s 未更新）`;
        b.className = "badge st-LOST";
      }
    }
  } catch (e) {
    $("#badge-state").textContent = "連線中斷";
    $("#badge-state").className = "badge st-LOST";
  }
  setTimeout(poll, 500);
}

function renderStatus(st) {
  if (!st.ready) {
    // restart 失敗時後端會帶 error——只顯示「啟動中」會讓人傻等
    $("#badge-state").textContent = st.error ? `⚠ ${st.error}` : "引擎啟動中…";
    if (st.error) $("#badge-state").className = "badge st-LOST";
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

  // 導引開關（arming＝待確認狀態，poll 不可覆寫掉提示）
  const btn = $("#btn-guidance");
  if (btn.classList.contains("arming")) {
    btn.textContent = "⚠ 再點一次確認啟用導引（4 秒內）";
  } else {
    btn.textContent = st.guidance_enabled ? "導引：啟用中（點擊關閉）" : "導引：關閉（點擊啟用）";
    btn.className = "big-btn " + (st.guidance_enabled ? "on" : "off");
  }

  // 接管恢復按鈕：只在「導引開著但被閂鎖」時出現
  const rbtn = $("#btn-resume");
  const showResume = st.guidance_enabled && st.latched;
  if (!showResume && !rbtn.hidden) rbtn.hidden = true;
  if (showResume && rbtn.hidden) {
    rbtn.hidden = false;
    rbtn.textContent = "⚠ 已接管——點擊恢復導引";
  }
  if (showResume && !rbtn.classList.contains("arming")) {
    rbtn.textContent = "⚠ 已接管——點擊恢復導引";
  }

  // 閘門
  const gates = $("#gate-list");
  // 高度被夾制永遠優先顯示：設定頁寫 4m、實際飛 20m，這種落差不能只在存檔時講一次
  const altWarn = st.alt_clamp_note
    ? `<div class="gate-item warn">⚠ ${st.alt_clamp_note}</div>` : "";
  // 🔴 「停止發送指令」不等於「飛機會停下來」。實測（模擬）：關掉導引後
  // 新指令數為 0，飛機仍朝最後一個指令點飛了 13.5m——PX4 收下的
  // DO_REPOSITION 不會因為我們不再發而取消，它會飛到該點並在那裡盤旋。
  // 這件事只有飛手用遙控器切模式才能解除，介面必須講出來。
  // 觸發條件看的是「指令還在外面、而我們已經不再更新它」，不是只看開關。
  // 只綁開關的話，最需要這句話的情況反而不會出現：目標丟失(LOST)、閘門擋住、
  // 飛手接管閂鎖——這些都是「導引還開著但已經不再發指令」，而飛機仍在飛向舊點。
  const lcAge = st.last_command ? st.last_command.age_s : null;
  const commandsFlowing = st.guidance_enabled && st.gates.length === 0 && st.state !== "LOST";
  const stillFlying = !commandsFlowing && lcAge !== null && lcAge !== undefined && lcAge < 180;
  const handoverWarn = stillFlying
    ? `<div class="gate-item warn">⚠ 已停止發送新指令，但飛機仍會飛向最後一個指令點並在該處盤旋
       （${fmt(lcAge, 0)} 秒前發出）。要立刻停住請由飛手用遙控器切 Position／Hold。</div>`
    : "";
  if (st.guidance_enabled && st.gates.length === 0 && st.state === "LOST") {
    gates.innerHTML = altWarn + handoverWarn +
      `<div class="gate-item">✗ 目標已丟失，停止發送新指令</div>`;
  } else if (st.guidance_enabled && st.gates.length === 0) {
    // 節流/deadband 是正常流量控制，用中性樣式；紅色 ✗ 只保留給真正的安全阻擋
    gates.innerHTML = altWarn + `<div class="gate-item ok">✓ 全部閘門通過，指令發送中</div>` +
      (st.guidance_note ? `<div class="gate-item info">⏱ ${st.guidance_note}</div>` : "");
  } else if (st.gates.length) {
    gates.innerHTML = altWarn + handoverWarn +
      st.gates.map((g) => `<div class="gate-item">✗ ${g}</div>`).join("");
  } else {
    gates.innerHTML = altWarn + handoverWarn;
  }

  // 最後指令
  const lc = st.last_command;
  // 任務記錄指示：導引啟用中會逐行寫入 data/missions/<檔名>.jsonl
  const ml = $("#mission-log");
  ml.hidden = !st.mission_log;
  if (st.mission_log) ml.textContent = `⏺ 任務記錄中：data/missions/${st.mission_log}`;

  // 已發送指令記錄：沒有這個，「鎖定了卻沒動」只能瞎猜是導引沒開、
  // 閘門在擋、還是飛控拒收——三種原因的處置完全不同。
  const log = $("#cmd-log");
  const cmds = st.commands || [];
  $("#cmd-count").textContent = cmds.length ? `${cmds.length} 筆` : "";
  if (cmds.length) {
    log.innerHTML = cmds.slice(0, 8).map((c) => {
      const ackCls = c.ack === "ACCEPTED" ? "ok" : (c.ack ? "bad" : "wait");
      const ackTxt = c.ack || "等待ACK";
      const spd = c.speed !== null && c.speed !== undefined ? `｜${c.speed}m/s` : "";
      return `<div class="cmd-row"><span class="mut">${c.wall}</span> ` +
        `N${c.n >= 0 ? "+" : ""}${c.n} E${c.e >= 0 ? "+" : ""}${c.e}｜${c.alt_rel}m${spd} ` +
        `<span class="ack ${ackCls}">${ackTxt}</span></div>`;
    }).join("");
  }
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

  // 偵測列表（顏色與畫面框一致：綠=偵測到、紅=已鎖定）
  $("#det-list").innerHTML = st.detections.length
    ? st.detections
        .map(
          (d) =>
            `<div class="det-item ${d.locked ? "locked" : ""}" data-id="${d.id}">
               <span class="det-dot"></span>
               <span><b>#${d.id}</b></span><span>${d.cls}</span><span>${(d.conf * 100).toFixed(0)}%</span>
               ${d.locked ? "<span>🔒 已鎖定</span>" : "<span class='mut'>點擊鎖定</span>"}
             </div>`
        )
        .join("")
    : `<div class="mut small">畫面中沒有偵測到目標</div>`;
  document.querySelectorAll(".det-item").forEach((el) =>
    el.addEventListener("click", () => postOrExplain("/api/lock", { track_id: Number(el.dataset.id) }))
  );

  // 載具
  const v = st.vehicle;
  $("#vehicle-info").innerHTML =
    kvRow("模式", v.mode || "—", v.mode === "AUTO.LOITER" ? "good" : "") +
    kvRow("Arm", v.armed ? "已解鎖" : "未解鎖", v.armed ? "good" : "") +
    kvRow("數傳", v.link_ok ? "正常" : "斷線", v.link_ok ? "good" : "bad") +
    kvRow("高度(相對)", v.rel_alt !== null && v.rel_alt !== undefined ? fmt(v.rel_alt, 0) + " m" : "—") +
    kvRow("Home", v.home_set ? "已取得" : "未取得", v.home_set ? "good" : "bad") +
    // 「有回報」不等於「實測」：PX4 會拿指令角合成同一則訊息，長得一模一樣。
    // 測地精度差在這裡，所以標示要分得出來。
    kvRow("雲台", st.gimbal.present
      ? `${st.gimbal.control}${st.gimbal.has_feedback
          ? (st.gimbal.attitude_measured ? "・實測角度" : "・僅指令角") : ""}`
      : "無",
      st.gimbal.present && st.gimbal.has_feedback && !st.gimbal.attitude_measured ? "bad" : "") +
    kvRow("處理速率", fmt(st.loop_hz, 0) + " Hz") +
    (st.mavlink && st.mavlink.backend === "lr24" && st.mavlink.lr24
      ? kvRow("LR24 指令",
          `送${st.mavlink.lr24.sent} ACK${st.mavlink.lr24.ack} ERR${st.mavlink.lr24.err}` +
          (st.mavlink.lr24.ready_for_goto === true ? "・可 GOTO" : st.mavlink.lr24.ready_for_goto === false ? "・未就緒" : ""),
          st.mavlink.lr24.connected ? "good" : "bad")
      : "") +
    // 飛控自己的訊息（PX4 STATUSTEXT）。「Arming denied: ...」只從這裡來，
    // 不顯示的話操作員面對的就只是「就是不能 arm」，毫無線索可查。
    ((v.messages && v.messages.length)
      ? `<div class="fc-msgs"><div class="mut small">飛控訊息</div>` +
        v.messages.slice(0, 5).map((m) =>
          `<div class="fc-msg${m.severity <= 3 ? " sev-err" : m.severity <= 4 ? " sev-warn" : ""}">${m.text}</div>`
        ).join("") + `</div>`
      : "");

  // 影像資訊。三種失效必須分開講，處置完全不同：
  //   擷取中斷（connected=false）→ USB／採集卡／驅動
  //   畫面停格（有幀但內容不變）→ 圖傳失鎖（RF）或相機端凍結
  //   幀齡變大但仍連線     → 引擎迴圈或推論太慢
  $("#video-label").textContent = st.video.device || "影像";
  const vh = st.video.health || {};
  const age = st.video.frame_age_s;
  let vtxt;
  if (!st.video.connected) {
    vtxt = vh.reopen_total ? `未連線（已重開 ${vh.reopen_total} 次）` : "未連線";
  } else if (vh.frozen) {
    vtxt = `⚠ 畫面停格${vh.blank ? "・全黑" : ""}`;
  } else if (age !== null && age !== undefined && age > 2) {
    vtxt = `⚠ ${fmt(age, 1)}s 未更新`;
  } else {
    vtxt = `${fmt(st.video.fps, 0)} FPS`;
  }
  $("#video-fps").textContent = vtxt;
  // 🔴 畫面凍住時 /frame.jpg 仍會回上一張舊圖（HTTP 200），輪詢器的 onload
  // 照常觸發，所以舊版覆蓋層在飛行中斷線時**永遠不會亮**——操作員對著一張
  // 死畫面以為一切正常。改由引擎回報的真實狀態驅動（見 setVideoDead）。
  setVideoDead(
    !st.video.connected ? "擷取中斷．重新連接中…"
      : vh.frozen ? (vh.blank ? "畫面停格・全黑（疑似圖傳失鎖）"
                              : "畫面停格中（採集卡有訊號、畫面沒變）")
      : null
  );
  const ve = $("#video-error");
  // 偵測/迴圈例外排在最前面：它們的症狀是「畫面正常但完全偵測不到」，
  // 不講出來只會被誤判成「模型不準」而往錯的方向查。
  const err = (st.loop_error ? `引擎迴圈錯誤：${st.loop_error}` : "")
    || (st.detector_error ? `偵測失敗：${st.detector_error}` : "")
    || st.video.error || st.mavlink.error;
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
// 啟用導引＝安全關鍵動作，用「頁內兩段式確認」而非原生 confirm()：
// confirm 對話框一旦被使用者勾選「不再顯示對話方塊」就會靜默回 false，
// 按鈕看起來就像壞掉。頁內確認不受此影響，且意圖更明確。
let armTimer = null;
function disarmGuidance() {
  clearTimeout(armTimer);
  armTimer = null;
  const btn = $("#btn-guidance");
  btn.classList.remove("arming");
}
// 送出後立刻把 lastStatus 改成目標值（樂觀更新）：輪詢週期 500ms，
// 剛啟用就想緊急關閉時，若還照「上一輪的舊狀態」分支，第一下會被
// 誤判成「要啟用」而進入確認流程——緊急關閉被拖慢 4 秒是不可接受的。
async function setGuidance(enabled) {
  const btn = $("#btn-guidance");
  btn.disabled = true;
  try {
    await post("/api/guidance", { enabled });
    if (lastStatus) { lastStatus.guidance_enabled = enabled; renderStatus(lastStatus); }
  } finally {
    btn.disabled = false;
  }
}
$("#btn-guidance").addEventListener("click", async () => {
  const btn = $("#btn-guidance");
  if (btn.disabled) return;
  const enabled = lastStatus && lastStatus.guidance_enabled;

  if (enabled) {          // 關閉不需確認
    disarmGuidance();
    await setGuidance(false);
    return;
  }

  if (armTimer) {         // 第二次點擊：確認啟用
    disarmGuidance();
    await setGuidance(true);
    return;
  }

  // 第一次點擊：進入待確認狀態，4 秒內沒再點就自動取消
  btn.classList.add("arming");
  armTimer = setTimeout(disarmGuidance, 4000);
});
$("#btn-unlock").addEventListener("click", () => post("/api/unlock"));

// 接管後的恢復：後端「重新啟用」= 解除閂鎖 + 清 deadband + 不換任務記錄檔。
// 沒有這顆專用按鈕的話，恢復要「關導引 → 再開 → 確認」按三次——飛行中太笨拙，
// 而且會把任務記錄切成兩份。一樣用兩段式確認，防誤觸。
let resumeTimer = null;
$("#btn-resume").addEventListener("click", async () => {
  const btn = $("#btn-resume");
  if (resumeTimer) {
    clearTimeout(resumeTimer); resumeTimer = null;
    btn.classList.remove("arming");
    await post("/api/guidance", { enabled: true });   // 重新啟用＝解除閂鎖
    return;
  }
  btn.classList.add("arming");
  btn.textContent = "⚠ 再點一次確認恢復（4 秒內）";
  resumeTimer = setTimeout(() => {
    resumeTimer = null; btn.classList.remove("arming");
  }, 4000);
});

/* ---------------- 鏈路自檢 ---------------- */
const SC_ICON = { pass: "✅", warn: "⚠️", fail: "❌", skip: "➖" };
$("#run-selfcheck").addEventListener("click", async () => {
  const btn = $("#run-selfcheck");
  const list = $("#selfcheck-list");
  const verdict = $("#selfcheck-verdict");
  btn.disabled = true;
  btn.textContent = "檢測中…";
  list.innerHTML = `<div class="mut small" style="padding:10px 0">正在實測各條鏈路…</div>`;
  verdict.hidden = true;

  let res;
  try {
    res = await post("/api/selfcheck");
  } catch (e) {
    res = { verdict: "fail", summary: "無法連線到伺服器", checks: [] };
  }
  btn.disabled = false;
  btn.textContent = "重新自檢";

  verdict.hidden = false;
  verdict.className = "sc-verdict sc-" + res.verdict;
  const c = res.counts || {};
  verdict.textContent =
    `${SC_ICON[res.verdict] || ""} ${res.summary}` +
    (res.counts ? `（通過 ${c.pass || 0}｜提醒 ${c.warn || 0}｜不通 ${c.fail || 0}）` : "") +
    (res.sim ? "　※ 目前為模擬模式" : "");

  list.innerHTML = (res.checks || [])
    .map(
      (k) => `
      <div class="sc-item sc-${k.status}">
        <span class="sc-icon">${SC_ICON[k.status] || "•"}</span>
        <div class="sc-body">
          <div class="sc-name">${k.name}</div>
          <div class="sc-detail">${k.detail}</div>
          ${k.fix && k.status !== "pass" && k.status !== "skip"
            ? `<div class="sc-fix">→ ${k.fix}</div>` : ""}
        </div>
      </div>`
    )
    .join("");
});

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
  { path: "video.rtsp_url", label: "RTSP 網址", type: "text", hint: "例 rtsp://192.168.144.25:8554/main；相機請設固定 IP ⟳" },
  { path: "video.rtsp_transport", label: "RTSP 傳輸", type: "select",
    options: [["auto", "自動（先 UDP 不通退 TCP，建議）"], ["udp", "UDP（最低延遲）"], ["tcp", "TCP（網路擋 UDP 時）"]],
    hint: "UDP 延遲最低；有些網路會擋，auto 會自動退 TCP ⟳" },
  { path: "video.latency_ms", label: "圖傳延遲 ms", type: "number",
    hint: "曝光→本機收到的固定延遲。未補償會有「延遲×飛行速度」的測地偏移（20m/s×300ms=6m）。鏡頭拍手機碼錶可量" },
  { sec: "偵測" },
  // 選項在 loadSettings 依 weights/ 目錄動態填入
  { path: "detector.weights", label: "偵測模型", type: "select", options: [],
    dynamic: "weights",
    hint: "⚠ 各模型只認自己訓練過的目標：玩具車模型偵測不到真車，空拍模型也偵測不到玩具車。"
        + "換場景一定要換模型 ⟳" },
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
  { path: "safety.multirotor.min_cmd_alt_m", label: "指令高度下限 m（旋翼）", type: "number",
    hint: "導引高度低於此值會被夾到此值；旋翼可低空跟隨" },
  { path: "safety.fixedwing.min_cmd_alt_m", label: "指令高度下限 m（固定翼）", type: "number",
    hint: "固定翼要有迴旋與失速餘裕，別跟旋翼共用" },
  { path: "safety.max_cmd_alt_m", label: "指令高度上限 m", type: "number" },
  { sec: "接飛控的 MAVLink 數傳（電台）" },
  { path: "mavlink.port", label: "數傳 COM 埠", type: "text",
    hint: "接 Pixhawk 的那條電台在筆電上的 COM 埠。你的 LR24 直接接飛控＝就填 LR24 的埠（例 COM10）⟳" },
  { path: "mavlink.baud", label: "鮑率", type: "number", hint: "LR24 用 115200；一般 SiK 電台 57600 ⟳" },
  { sec: "指令送出方式（線路選擇）" },
  { path: "link.command_backend", label: "指令走哪條路", type: "seg",
    options: [["direct", "直接接飛控"], ["lr24", "經機上 RPi"]],
    hint: "direct=上面那條數傳既讀 pose 又直接發 DO_REPOSITION 給 PX4（四軸首飛、你目前的接法）；"
        + "lr24=另有機上 companion 跑 global_goto_node，這條 LR24 送文字指令給它 ⟳" },
  { path: "link.lr24_port", label: "LR24 指令 COM 埠", type: "text", only: "lr24",
    hint: "只有『經機上 RPi』才需要：送指令用的 LR24，與上面讀 pose 的電台是不同兩條 ⟳" },
  { path: "link.lr24_baud", label: "LR24 鮑率", type: "number", only: "lr24", hint: "常見 115200 ⟳" },
  { path: "link.goto_altitude_ref", label: "GOTO 高度基準", type: "select", only: "lr24",
    options: [["rel_home", "相對 Home（節點限 30~120m）"], ["amsl", "AMSL 海拔"]], hint: "送 GOTO / GOTO_AMSL ⟳" },
];

function getPath(obj, path) {
  return path.split(".").reduce((o, k) => (o == null ? undefined : o[k]), obj);
}

/* 依「指令走哪條路」顯示/隱藏 LR24 專屬欄位，並在下方畫出對應接線說明 */
function updateBackendRows() {
  const seg = document.querySelector('.seg[data-path="link.command_backend"]');
  if (!seg) return;
  const sel = seg.querySelector("button.sel");
  const backend = sel ? sel.dataset.val : "direct";

  document.querySelectorAll("#settings-form .cfg-row[data-only]").forEach((row) => {
    row.style.display = row.dataset.only === backend ? "" : "none";
  });

  let note = document.getElementById("backend-note");
  if (!note) {
    note = document.createElement("div");
    note.id = "backend-note";
    note.className = "meta-box";
    seg.closest(".cfg-section").appendChild(note);
  }
  note.innerHTML =
    backend === "direct"
      ? "<b>接線</b>：數傳電台（你的 LR24 或 SiK）直接接 Pixhawk TELEM1，另一端插筆電。" +
        "上面「數傳 COM 埠」填這條電台的埠，這一條就同時負責讀 pose 和發指令。" +
        "<b>不需要機上 RPi。</b>下面的 LR24 指令欄位在這個模式用不到。"
      : "<b>接線</b>：需要兩條無線鏈路——① SiK/MAVLink 電台接 Pixhawk 讀 pose（上面「數傳 COM 埠」）；" +
        "② LR24 送文字指令給機上 companion 的 global_goto_node（下面「LR24 指令 COM 埠」）。" +
        "四軸要走這條需先套 global_goto_node 多旋翼 patch。";
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
    // 動態選項：模型清單由伺服器列出 weights/ 目錄提供
    if (f.dynamic === "weights") {
      const list = (data.meta && data.meta.weights) || [];
      f = Object.assign({}, f, {
        options: list.length
          ? list.map((w) => [w.path, `${w.name}${w.note ? "  — " + w.note : ""}`])
          : [[value || "weights/best.pt", "（weights/ 目錄沒有 .pt 檔）"]],
      });
    }
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
    if (f.only) row.dataset.only = f.only;   // 僅特定後端才顯示的欄位
    section.appendChild(row);
  });

  document.querySelectorAll(".seg button").forEach((btn) =>
    btn.addEventListener("click", () => {
      btn.parentElement.querySelectorAll("button").forEach((b) => b.classList.remove("sel"));
      btn.classList.add("sel");
      if (btn.parentElement.dataset.path === "link.command_backend") updateBackendRows();
    })
  );
  updateBackendRows();  // 初次載入依目前選擇顯示/隱藏

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
  const applied = (res.applied || []).length ? `（已即時套用：${res.applied.join("、")}）` : "";
  $("#cfg-msg").textContent = (res.note || "已儲存") + applied;
});

$("#cfg-testvideo").addEventListener("click", async () => {
  const box = $("#video-test-result");
  box.hidden = false;
  box.textContent = "測試中…（RTSP 可能要數秒）";
  // 用目前表單上的值試，不必先儲存
  const patch = collectPatch();
  const res = await post("/api/video/test", (patch && patch.video) || {});
  if (res.ok) {
    box.innerHTML =
      `<b>✓ 影像來源正常</b>（${res.mode}）<br>` +
      `來源：${res.device || "—"}<br>` +
      `實際解析度：<b>${res.width}×${res.height}</b>｜擷取速率：約 ${res.fps ?? "—"} FPS` +
      (res.fourcc ? `｜格式 ${res.fourcc}` : "") + "<br>" +
      (res.fps !== null && res.fps < 15
        ? `<span style="color:#b5493f">⚠ FPS 偏低：若格式不是 MJPG → USB2 頻寬不足；` +
          `若格式已是 MJPG → 多半是<b>採集卡沒收到 HDMI 訊號</b>（圖傳未供電/未鎖定）</span><br>`
        : "") +
      `<span class="mut">校正請用這個實際解析度重做；GStreamer：${res.gstreamer ? "可用（RTSP 延遲更低）" : "未編入（用 FFMPEG）"}</span>`;
  } else {
    box.innerHTML = `<b>✗ 測試失敗</b>（${res.mode}）<br>${res.error || "未知錯誤"}`;
  }
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

/* ---------------- 影像輪詢（自癒；取代 MJPEG 長連線） ----------------
   MJPEG <img> 在伺服器/引擎重啟後會卡死不重連，操作員看到凍結畫面很危險。
   改成輪詢單幀 /frame.jpg：用 onload 串接下一張（不會塞車），
   失敗就顯示「連線中斷」並自動重試——任何重啟都能恢復。 */
// 引擎回報的影像死亡原因（null＝正常）。renderStatus 寫、輪詢器讀——
// 兩邊都在控制同一個覆蓋層，不共用狀態就會互相蓋來蓋去。
let videoDeadReason = null;
function setVideoDead(reason) {
  videoDeadReason = reason;
  if (typeof refreshFrameOverlay === "function") refreshFrameOverlay();
}
let refreshFrameOverlay = null;

function startFramePoller(imgId, { fps = 12, lostId = null, staleMs = 2500 } = {}) {
  const img = document.getElementById(imgId);
  if (!img) return;
  const lost = lostId ? document.getElementById(lostId) : null;
  const minGap = 1000 / fps;
  let stopped = false;
  let lastGoodAt = performance.now();

  // 覆蓋層的判斷來源有兩個，缺一不可：
  //   ① 引擎說影像死了（擷取中斷或畫面停格）——**唯一**能偵測飛行中斷線的來源，
  //      因為 /frame.jpg 會一直回上一張舊圖，HTTP 永遠成功。
  //   ② HTTP 端真的拿不到圖超過 staleMs（伺服器/引擎重啟中）。
  // 只看單次請求成敗是不行的：畫面正在更新卻顯示「中斷」是最糟的 UI 謊言。
  const refreshOverlay = () => {
    if (!lost) return;
    const httpStale = performance.now() - lastGoodAt >= staleMs;
    lost.hidden = !(videoDeadReason || httpStale);
    if (!lost.hidden) {
      lost.textContent = videoDeadReason || "影像連線中斷．重新連接中…";
    }
  };
  if (lostId === "stream-lost") refreshFrameOverlay = refreshOverlay;

  const tick = () => {
    if (stopped) return;
    // 面板沒顯示時（在別的分頁）不輪詢，省瀏覽器連線數。此時也不該累積「過期」，
    // 否則切回來會先閃一下中斷。
    if (img.offsetParent === null) {
      lastGoodAt = performance.now();
      if (lost) lost.hidden = true;
      setTimeout(tick, 400);
      return;
    }
    const started = performance.now();
    const probe = new Image();
    probe.onload = () => {
      img.src = probe.src;           // 成功才換上去，避免破圖閃爍
      lastGoodAt = performance.now();
      refreshOverlay();
      const wait = Math.max(0, minGap - (performance.now() - started));
      setTimeout(tick, wait);
    };
    probe.onerror = () => {
      // 單次失敗完全正常（連線佔用、剛好沒新幀、引擎重啟中）；
      // 只有超過 staleMs 沒拿到任何新畫面才提示。
      refreshOverlay();
      setTimeout(tick, minGap);
    };
    probe.src = "/frame.jpg?t=" + Date.now();
  };
  tick();
  return () => { stopped = true; };
}
startFramePoller("stream", { fps: 12, lostId: "stream-lost" });
startFramePoller("calib-stream", { fps: 10 });

/* ---------------- 點畫面選取目標 ----------------
   直接點框內即可鎖定，不必到列表找。座標要換算：
   <img> 是縮放顯示的，點擊位置得換回原始幀像素才能跟 bbox 比對。 */
function enableClickToLock(imgId) {
  const img = document.getElementById(imgId);
  if (!img) return;
  img.addEventListener("click", (e) => {
    if (!lastStatus || !lastStatus.detections || !lastStatus.detections.length) return;
    // 縮放/平移後，用 getBoundingClientRect 直接算出點在圖上的相對位置
    const r = img.getBoundingClientRect();
    const nx = (e.clientX - r.left) / r.width;   // 0~1
    const ny = (e.clientY - r.top) / r.height;
    const W = img.naturalWidth || 1920, H = img.naturalHeight || 1080;
    const px = nx * W, py = ny * H;

    // 找出點到的框（多個重疊時取面積最小的，通常是使用者想選的那個）
    let best = null, bestArea = Infinity;
    for (const d of lastStatus.detections) {
      const [x1, y1, x2, y2] = d.bbox;
      if (px >= x1 && px <= x2 && py >= y1 && py <= y2) {
        const area = (x2 - x1) * (y2 - y1);
        if (area < bestArea) { best = d; bestArea = area; }
      }
    }
    if (best) postOrExplain("/api/lock", { track_id: best.id });
  });
  img.style.cursor = "crosshair";
}
enableClickToLock("stream");

/* ---------------- 影像縮放（滾輪縮放/拖曳平移/雙擊還原） ---------------- */
function makeZoomable(wrapId) {
  const wrap = document.getElementById(wrapId);
  if (!wrap) return;
  const img = wrap.querySelector("img");
  let scale = 1, tx = 0, ty = 0, dragging = false, lastX = 0, lastY = 0;
  let levelEl = null;

  const apply = () => {
    const r = wrap.getBoundingClientRect();
    // 平移夾在邊界內：畫面永遠被影像蓋滿，不露出黑邊外的空洞
    tx = Math.min(0, Math.max(r.width - r.width * scale, tx));
    ty = Math.min(0, Math.max(r.height - r.height * scale, ty));
    img.style.transform = `translate(${tx}px, ${ty}px) scale(${scale})`;
    wrap.classList.toggle("zoomed", scale > 1.001);
    if (scale > 1.001) {
      if (!levelEl) {
        levelEl = document.createElement("span");
        levelEl.className = "zoom-level";
        wrap.appendChild(levelEl);
      }
      levelEl.textContent = `${scale.toFixed(1)}×`;
    } else if (levelEl) {
      levelEl.remove();
      levelEl = null;
    }
  };

  wrap.addEventListener("wheel", (e) => {
    e.preventDefault();
    const r = wrap.getBoundingClientRect();
    const mx = e.clientX - r.left, my = e.clientY - r.top;
    const prev = scale;
    scale = Math.min(8, Math.max(1, scale * (e.deltaY < 0 ? 1.25 : 0.8)));
    // 以游標位置為錨點縮放（放大時游標下的東西不跑掉）
    tx = mx - ((mx - tx) * scale) / prev;
    ty = my - ((my - ty) * scale) / prev;
    apply();
  }, { passive: false });

  wrap.addEventListener("mousedown", (e) => {
    if (scale > 1) { dragging = true; lastX = e.clientX; lastY = e.clientY; e.preventDefault(); }
  });
  window.addEventListener("mousemove", (e) => {
    if (!dragging) return;
    tx += e.clientX - lastX; ty += e.clientY - lastY;
    lastX = e.clientX; lastY = e.clientY;
    apply();
  });
  window.addEventListener("mouseup", () => (dragging = false));
  wrap.addEventListener("dblclick", () => { scale = 1; tx = 0; ty = 0; apply(); });
  img.addEventListener("dragstart", (e) => e.preventDefault());
}
makeZoomable("stream-zoom");
makeZoomable("calib-zoom");

/* ---------------- 使用說明 ---------------- */
const helpOverlay = $("#help-overlay");
const openHelp = () => (helpOverlay.hidden = false);
const closeHelp = () => {
  helpOverlay.hidden = true;
  localStorage.setItem("uav_yolo_help_seen", "1");
};
$("#btn-help").addEventListener("click", openHelp);
$("#help-close").addEventListener("click", closeHelp);
helpOverlay.addEventListener("click", (e) => {
  if (e.target === helpOverlay) closeHelp();
});
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape" && !helpOverlay.hidden) closeHelp();
});
if (!localStorage.getItem("uav_yolo_help_seen")) openHelp(); // 首次進站自動展示

/* ---------------- 啟動 ---------------- */
poll();
