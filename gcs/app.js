/**
 * NIDAR RescueSwarm — Ground Control Station (GCS)
 * Autonomous Multi-Drone Coordination & Live Canvas Radar Simulation
 */

// Search area boundaries (Bangalore test polygon)
const GEOFENCE = {
  minLat: 12.9710,
  maxLat: 12.9736,
  minLon: 12.9738,
  maxLon: 12.9776,
  baseLat: 12.9712,
  baseLon: 12.9734,
};

// State
let isPlaying = true;
let simSpeed = 1.0;
let showSectors = true;
let showPaths = true;
let simTime = 0.0;

// Search sectors (S1, S2, S3)
const SECTORS = [
  { id: 'S1', color: '#10b981', minLon: GEOFENCE.minLon, maxLon: GEOFENCE.minLon + (GEOFENCE.maxLon - GEOFENCE.minLon) / 3 },
  { id: 'S2', color: '#00f2fe', minLon: GEOFENCE.minLon + (GEOFENCE.maxLon - GEOFENCE.minLon) / 3, maxLon: GEOFENCE.minLon + 2 * (GEOFENCE.maxLon - GEOFENCE.minLon) / 3 },
  { id: 'S3', color: '#a855f7', minLon: GEOFENCE.minLon + 2 * (GEOFENCE.maxLon - GEOFENCE.minLon) / 3, maxLon: GEOFENCE.maxLon },
];

// Swarm Fleet Definition
const drones = [
  // Search fleet
  { id: 'S1', role: 'search', status: 'SEARCHING', lat: GEOFENCE.minLat, lon: GEOFENCE.minLon + 0.0006, alt: 30.0, heading: 0, battery: 0.96, waypoints: [], currentWpIdx: 0, trail: [] },
  { id: 'S2', role: 'search', status: 'SEARCHING', lat: GEOFENCE.minLat, lon: GEOFENCE.minLon + 0.0019, alt: 30.0, heading: 0, battery: 0.94, waypoints: [], currentWpIdx: 0, trail: [] },
  { id: 'S3', role: 'search', status: 'SEARCHING', lat: GEOFENCE.minLat, lon: GEOFENCE.minLon + 0.0031, alt: 30.0, heading: 0, battery: 0.98, waypoints: [], currentWpIdx: 0, trail: [] },
  // Delivery fleet
  { id: 'D1', role: 'delivery', status: 'READY', lat: GEOFENCE.baseLat, lon: GEOFENCE.baseLon, alt: 0.0, heading: 45, battery: 0.88, activeTaskId: null, state: 'STANDBY', stateTimer: 0, trail: [] },
  { id: 'D2', role: 'delivery', status: 'READY', lat: GEOFENCE.baseLat, lon: GEOFENCE.baseLon, alt: 0.0, heading: 45, battery: 0.95, activeTaskId: null, state: 'STANDBY', stateTimer: 0, trail: [] },
  { id: 'D3', role: 'delivery', status: 'READY', lat: GEOFENCE.baseLat, lon: GEOFENCE.baseLon, alt: 0.0, heading: 45, battery: 0.81, activeTaskId: null, state: 'STANDBY', stateTimer: 0, trail: [] },
];

// Tasks & Survivors
let tasks = [];
let taskCounter = 1;
let missionState = 'SEARCHING';
let pulsePhase = 0.0;

// Generate Lawnmower Waypoints for each search sector
function generateSectorWaypoints(sector, tracksCount = 4) {
  const wps = [];
  const lonStep = (sector.maxLon - sector.minLon) / tracksCount;
  let sweepNorth = true;
  for (let i = 0; i < tracksCount; i++) {
    const lon = sector.minLon + (i + 0.5) * lonStep;
    if (sweepNorth) {
      wps.push({ lat: GEOFENCE.minLat + 0.0002, lon: lon });
      wps.push({ lat: GEOFENCE.maxLat - 0.0002, lon: lon });
    } else {
      wps.push({ lat: GEOFENCE.maxLat - 0.0002, lon: lon });
      wps.push({ lat: GEOFENCE.minLat + 0.0002, lon: lon });
    }
    sweepNorth = !sweepNorth;
  }
  return wps;
}

// Initialize Waypoints
drones[0].waypoints = generateSectorWaypoints(SECTORS[0]);
drones[1].waypoints = generateSectorWaypoints(SECTORS[1]);
drones[2].waypoints = generateSectorWaypoints(SECTORS[2]);

// Setup DOM elements
const canvas = document.getElementById('tactical-canvas');
const ctx = canvas.getContext('2d');
const droneCardsContainer = document.getElementById('drone-cards-container');
const taskTableBody = document.getElementById('task-table-body');
const terminalFeed = document.getElementById('terminal-log-feed');
const missionStateVal = document.getElementById('mission-state-val');
const survivorsCountVal = document.getElementById('survivors-count');
const deliveriesCountVal = document.getElementById('deliveries-count');
const safetyLevelVal = document.getElementById('safety-level-val');

// Logging
function addLog(msg, level = 'normal') {
  const entry = document.createElement('div');
  entry.className = 'log-entry';
  const timeStr = `[${new Date().toTimeString().split(' ')[0]}]`;
  entry.innerHTML = `<span class="log-time">${timeStr}</span> <span class="log-level-${level}">${msg}</span>`;
  terminalFeed.appendChild(entry);
  terminalFeed.scrollTop = terminalFeed.scrollHeight;
}

// Convert Lat/Lon to Canvas Pixels
function project(lat, lon, width, height, padding = 45) {
  const x = padding + ((lon - (GEOFENCE.baseLon - 0.0003)) / (GEOFENCE.maxLon - GEOFENCE.baseLon + 0.0008)) * (width - 2 * padding);
  const y = height - padding - ((lat - GEOFENCE.minLat) / (GEOFENCE.maxLat - GEOFENCE.minLat)) * (height - 2 * padding);
  return { x, y };
}

// Inverse Projection (Pixels to Lat/Lon for Click-to-Detect)
function unproject(px, py, width, height, padding = 45) {
  const lon = (GEOFENCE.baseLon - 0.0003) + ((px - padding) / (width - 2 * padding)) * (GEOFENCE.maxLon - GEOFENCE.baseLon + 0.0008);
  const lat = GEOFENCE.minLat + ((height - padding - py) / (height - 2 * padding)) * (GEOFENCE.maxLat - GEOFENCE.minLat);
  return { lat, lon };
}

// Resize canvas dynamically
function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width * window.devicePixelRatio;
  canvas.height = rect.height * window.devicePixelRatio;
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Initial Detection Setup
function createInitialScenario() {
  addLog('RescueSwarm 6-Drone Network Initialized.', 'info');
  addLog('S1, S2, S3 assigned non-overlapping search sectors.', 'info');
  addLog('D1, D2, D3 armed and ready for deterministic task allocation.', 'info');

  // Initial detection originates from verified best.pt model inference on test_survivors.jpg
  injectSurvivor(12.97198, 77.59446, 'S1', 'MODEL_INFERENCE');
  setTimeout(() => {
    injectSurvivor(12.9729, 77.5968, 'S3', 'DEMO_INJECTION');
  }, 1200);
}

// Inject Survivor
function injectSurvivor(lat, lon, spottedBy = 'S1', source = 'DEMO_INJECTION') {
  // Spatial Deduplication check
  for (const t of tasks) {
    const dLat = Math.abs(t.lat - lat);
    const dLon = Math.abs(t.lon - lon);
    if (dLat < 0.00015 && dLon < 0.00015) {
      addLog(`Spatial Deduplication: Sighting matches existing ${t.id}. Updated confidence to 0.98.`, 'info');
      t.confidence = 0.98;
      updateTaskTable();
      return;
    }
  }

  const taskId = `task_${String(taskCounter++).padStart(3, '0')}`;
  const newTask = {
    id: taskId,
    spottedBy: spottedBy,
    lat: lat,
    lon: lon,
    confidence: source === 'MODEL_INFERENCE' ? 0.94 : (0.85 + Math.random() * 0.10),
    status: 'GEOLOCATED',
    assignedDroneId: null,
    source: source,
  };

  // Trigger search drone approach and hover behavior
  const searchDrone = drones.find(d => d.id === spottedBy);
  if (searchDrone) {
    searchDrone.status = 'APPROACHING';
    searchDrone.approachState = 'APPROACHING';
    searchDrone.targetLat = lat;
    searchDrone.targetLon = lon;
    searchDrone.hoverTimer = 0.0;
    addLog(`${spottedBy} target acquired -> Pausing coverage waypoints and entering visual servoing APPROACH.`, 'warning');
  }

  tasks.push(newTask);
  const srcBadge = source === 'MODEL_INFERENCE' ? '[MODEL INFERENCE]' : '[DEMO INJECTION]';
  addLog(`New Survivor Detected by ${spottedBy} ${srcBadge} -> Geotagged at [${lat.toFixed(6)}, ${lon.toFixed(6)}]`, 'warning');
  dispatchDeliveryTask(newTask);
  updateTaskTable();
  updateMetrics();
}

// Deterministic Allocator: pick highest battery available delivery drone
function dispatchDeliveryTask(task) {
  const eligibleDrones = drones.filter(d => d.role === 'delivery' && d.status === 'READY');
  if (eligibleDrones.length === 0) {
    addLog(`No delivery drones available. ${task.id} queued in GEOLOCATED state.`, 'warning');
    return;
  }

  eligibleDrones.sort((a, b) => b.battery - a.battery);
  const selected = eligibleDrones[0];

  task.assignedDroneId = selected.id;
  task.status = 'ASSIGNED';
  selected.status = 'BUSY';
  selected.activeTaskId = task.id;
  selected.state = 'TAKEOFF';
  selected.stateTimer = 0;

  missionState = 'RESCUE_IN_PROGRESS';
  missionStateVal.textContent = 'RESCUE IN PROGRESS';
  missionStateVal.className = 'status-val text-warning';

  addLog(`Allocated highest-battery drone ${selected.id} (${Math.round(selected.battery * 100)}%) to ${task.id}.`, 'info');
  renderDroneCards();
}

// Advance Delivery Drone FSM
function updateDeliveryDrone(drone, dt) {
  if (drone.status !== 'BUSY' || !drone.activeTaskId) return;
  const task = tasks.find(t => t.id === drone.activeTaskId);
  if (!task) return;

  const cruiseAlt = 25.0;
  const dropAlt = 8.0;
  const speed = 0.00008; // coordinate delta per second

  drone.battery = Math.max(0.05, drone.battery - 0.0004 * dt);

  if (drone.state === 'TAKEOFF') {
    drone.alt = Math.min(cruiseAlt, drone.alt + 6.0 * dt);
    task.status = 'EN_ROUTE';
    if (drone.alt >= cruiseAlt) {
      drone.state = 'EN_ROUTE';
    }
  } else if (drone.state === 'EN_ROUTE') {
    const dLat = task.lat - drone.lat;
    const dLon = task.lon - drone.lon;
    const dist = Math.hypot(dLat, dLon);

    drone.heading = (Math.atan2(dLon, dLat) * 180 / Math.PI + 360) % 360;

    if (dist < 0.0001) {
      drone.lat = task.lat;
      drone.lon = task.lon;
      drone.state = 'DESCENT';
    } else {
      drone.lat += (dLat / dist) * speed * dt;
      drone.lon += (dLon / dist) * speed * dt;
    }
  } else if (drone.state === 'DESCENT') {
    drone.alt = Math.max(dropAlt, drone.alt - 5.0 * dt);
    if (drone.alt <= dropAlt) {
      drone.state = 'RELEASE';
      drone.stateTimer = 0;
    }
  } else if (drone.state === 'RELEASE') {
    drone.stateTimer += dt;
    task.status = 'RELEASED';
    if (drone.stateTimer >= 1.5) {
      addLog(`Payload released by ${drone.id} over survivor ${task.id}. Initiating verification.`, 'info');
      drone.state = 'VERIFY';
      drone.stateTimer = 0;
    }
  } else if (drone.state === 'VERIFY') {
    drone.stateTimer += dt;
    if (drone.stateTimer >= 2.0) {
      task.status = 'VERIFIED';
      addLog(`✓ Delivery ${task.id} VERIFIED by ${drone.id}. Return-To-Launch triggered.`, 'info');
      drone.state = 'RTL';
      updateMetrics();
      updateTaskTable();
    }
  } else if (drone.state === 'RTL') {
    drone.alt = Math.min(cruiseAlt, drone.alt + 6.0 * dt);
    const dLat = GEOFENCE.baseLat - drone.lat;
    const dLon = GEOFENCE.baseLon - drone.lon;
    const dist = Math.hypot(dLat, dLon);

    drone.heading = (Math.atan2(dLon, dLat) * 180 / Math.PI + 360) % 360;

    if (dist < 0.0001) {
      drone.lat = GEOFENCE.baseLat;
      drone.lon = GEOFENCE.baseLon;
      drone.alt = 0.0;
      drone.status = 'READY';
      drone.state = 'STANDBY';
      drone.activeTaskId = null;
      addLog(`Drone ${drone.id} landed at Base Station. Status reset to READY.`, 'info');
      
      // Dispatch next pending task if any
      const pending = tasks.find(t => t.status === 'GEOLOCATED');
      if (pending) dispatchDeliveryTask(pending);
    } else {
      drone.lat += (dLat / dist) * speed * dt;
      drone.lon += (dLon / dist) * speed * dt;
    }
  }

  // Update trail
  drone.trail.push({ lat: drone.lat, lon: drone.lon });
  if (drone.trail.length > 30) drone.trail.shift();
}

// Advance Search Drones along lawnmower waypoints or visual servoing approach
function updateSearchDrone(drone, dt) {
  drone.battery = Math.max(0.10, drone.battery - 0.00015 * dt);

  // Case 1: Search drone actively approaching detected person
  if (drone.status === 'APPROACHING') {
    const dLat = drone.targetLat - drone.lat;
    const dLon = drone.targetLon - drone.lon;
    const dist = Math.hypot(dLat, dLon);
    drone.heading = (Math.atan2(dLon, dLat) * 180 / Math.PI + 360) % 360;

    const approachSpeed = 0.00007;
    if (dist < 0.00004) {
      drone.status = 'HOVERING';
      drone.approachState = 'HOVERING';
      drone.hoverTimer = 0.0;
      addLog(`${drone.id} reached target coordinates -> Centered overhead HOVERING (visual servoing deadband met).`, 'info');
    } else {
      drone.lat += (dLat / dist) * approachSpeed * dt;
      drone.lon += (dLon / dist) * approachSpeed * dt;
    }
    drone.trail.push({ lat: drone.lat, lon: drone.lon });
    if (drone.trail.length > 25) drone.trail.shift();
    return;
  }

  // Case 2: Search drone hovering over detected person for sensor inspection
  if (drone.status === 'HOVERING') {
    drone.hoverTimer = (drone.hoverTimer || 0) + dt;
    if (drone.hoverTimer >= 5.0) {
      drone.status = 'SEARCHING';
      drone.approachState = 'SEARCH_PATROL';
      addLog(`${drone.id} target inspection complete -> Resuming search coverage lawnmower waypoints.`, 'info');
    }
    return;
  }

  // Case 3: Normal lawnmower waypoint following
  if (drone.waypoints.length === 0) return;
  const targetWp = drone.waypoints[drone.currentWpIdx];
  const dLat = targetWp.lat - drone.lat;
  const dLon = targetWp.lon - drone.lon;
  const dist = Math.hypot(dLat, dLon);

  const speed = 0.00006;
  drone.heading = (Math.atan2(dLon, dLat) * 180 / Math.PI + 360) % 360;

  if (dist < 0.00008) {
    drone.currentWpIdx = (drone.currentWpIdx + 1) % drone.waypoints.length;
  } else {
    drone.lat += (dLat / dist) * speed * dt;
    drone.lon += (dLon / dist) * speed * dt;
  }

  drone.trail.push({ lat: drone.lat, lon: drone.lon });
  if (drone.trail.length > 25) drone.trail.shift();
}

// Main Simulation Tick
function simTick(dt) {
  simTime += dt;
  pulsePhase += dt * 3;

  for (const d of drones) {
    if (d.role === 'search') {
      updateSearchDrone(d, dt);
    } else {
      updateDeliveryDrone(d, dt);
    }
  }

  renderTacticalMap();
  renderDroneCards();
}

// Render Tactical Radar View
function renderTacticalMap() {
  const w = canvas.parentElement.clientWidth;
  const h = canvas.parentElement.clientHeight;

  ctx.clearRect(0, 0, w, h);

  // 1. Draw subtle grid
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
  ctx.lineWidth = 1;
  const gridSize = 40;
  for (let x = 0; x < w; x += gridSize) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += gridSize) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  // 2. Search Sectors
  if (showSectors) {
    for (const sec of SECTORS) {
      const p1 = project(GEOFENCE.maxLat, sec.minLon, w, h);
      const p2 = project(GEOFENCE.minLat, sec.maxLon, w, h);
      const secW = p2.x - p1.x;
      const secH = p2.y - p1.y;

      ctx.fillStyle = `${sec.color}0c`;
      ctx.fillRect(p1.x, p1.y, secW, secH);

      ctx.strokeStyle = `${sec.color}35`;
      ctx.lineWidth = 1.2;
      ctx.setLineDash([4, 4]);
      ctx.strokeRect(p1.x, p1.y, secW, secH);
      ctx.setLineDash([]);

      ctx.fillStyle = `${sec.color}aa`;
      ctx.font = '600 11px Inter, sans-serif';
      ctx.fillText(`SECTOR ${sec.id}`, p1.x + 8, p1.y + 18);
    }
  }

  // 3. Planned Paths
  if (showPaths) {
    for (let i = 0; i < 3; i++) {
      const drone = drones[i];
      const secColor = SECTORS[i].color;
      ctx.strokeStyle = `${secColor}40`;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 6]);
      ctx.beginPath();
      for (let j = 0; j < drone.waypoints.length; j++) {
        const pt = project(drone.waypoints[j].lat, drone.waypoints[j].lon, w, h);
        if (j === 0) ctx.moveTo(pt.x, pt.y);
        else ctx.lineTo(pt.x, pt.y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  // 4. Base Station
  const basePt = project(GEOFENCE.baseLat, GEOFENCE.baseLon, w, h);
  ctx.fillStyle = 'rgba(59, 130, 246, 0.2)';
  ctx.beginPath();
  ctx.arc(basePt.x, basePt.y, 16, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#3b82f6';
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.fillStyle = '#60a5fa';
  ctx.font = '600 10px JetBrains Mono, monospace';
  ctx.fillText('BASE STATION', basePt.x - 34, basePt.y + 28);

  // 5. Survivors / Target Locations
  for (const t of tasks) {
    const pt = project(t.lat, t.lon, w, h);
    const pulseRad = 12 + Math.sin(pulsePhase) * 6;

    // Sonar wave pulse
    ctx.strokeStyle = t.status === 'VERIFIED' ? 'rgba(16, 185, 129, 0.4)' : 'rgba(239, 68, 68, 0.4)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, pulseRad, 0, Math.PI * 2);
    ctx.stroke();

    // Center pin
    ctx.fillStyle = t.status === 'VERIFIED' ? '#10b981' : '#ef4444';
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#f8fafc';
    ctx.font = '600 10px JetBrains Mono, monospace';
    ctx.fillText(`${t.id} (${t.status})`, pt.x + 9, pt.y + 3);
  }

  // 6. Draw Drones and Trails
  for (const d of drones) {
    const pt = project(d.lat, d.lon, w, h);

    // Trail
    if (d.trail.length > 1) {
      ctx.strokeStyle = d.role === 'search' ? 'rgba(0, 242, 254, 0.25)' : 'rgba(245, 158, 11, 0.25)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < d.trail.length; i++) {
        const tp = project(d.trail[i].lat, d.trail[i].lon, w, h);
        if (i === 0) ctx.moveTo(tp.x, tp.y);
        else ctx.lineTo(tp.x, tp.y);
      }
      ctx.stroke();
    }

    // Drone Icon
    ctx.save();
    ctx.translate(pt.x, pt.y);
    ctx.rotate((d.heading * Math.PI) / 180);

    const droneColor = d.role === 'search' ? '#00f2fe' : (d.status === 'BUSY' ? '#f59e0b' : '#64748b');

    // Quadcopter Arms
    ctx.strokeStyle = droneColor;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-7, -7); ctx.lineTo(7, 7);
    ctx.moveTo(7, -7); ctx.lineTo(-7, 7);
    ctx.stroke();

    // Rotors
    ctx.fillStyle = droneColor;
    [-7, 7].forEach(rx => {
      [-7, 7].forEach(ry => {
        ctx.beginPath();
        ctx.arc(rx, ry, 2.5, 0, Math.PI * 2);
        ctx.fill();
      });
    });

    // Heading nose triangle
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.moveTo(0, -9);
    ctx.lineTo(-3, -4);
    ctx.lineTo(3, -4);
    ctx.closePath();
    ctx.fill();

    ctx.restore();

    // Label
    ctx.fillStyle = '#e2e8f0';
    ctx.font = '600 10px JetBrains Mono, monospace';
    ctx.fillText(`${d.id} [${d.alt.toFixed(0)}m]`, pt.x + 10, pt.y - 8);
  }
}

// Render Drone Telemetry Cards
function renderDroneCards() {
  droneCardsContainer.innerHTML = '';
  for (const d of drones) {
    const card = document.createElement('div');
    card.className = 'drone-card';

    const battPct = Math.round(d.battery * 100);
    const battClass = battPct > 35 ? '' : (battPct > 20 ? 'warning' : 'danger');
    const roleClass = d.role === 'search' ? 'role-search' : 'role-delivery';

    card.innerHTML = `
      <div class="drone-card-top">
        <div class="drone-id-tag">
          <span>${d.id}</span>
          <span class="role-pill ${roleClass}">${d.role}</span>
        </div>
        <span class="drone-status-tag ${d.status === 'BUSY' || d.status === 'SEARCHING' ? 'active' : ''}">${d.status}</span>
      </div>
      <div class="battery-row">
        <div class="battery-bar-bg">
          <div class="battery-bar-fill ${battClass}" style="width: ${battPct}%"></div>
        </div>
        <span class="battery-pct">${battPct}%</span>
      </div>
      <div class="telemetry-meta">
        <div>ALT: <span class="meta-val">${d.alt.toFixed(1)}m</span></div>
        <div>HDG: <span class="meta-val">${Math.round(d.heading)}°</span></div>
        <div>TASK: <span class="meta-val">${d.activeTaskId || 'NONE'}</span></div>
        <div>MODE: <span class="meta-val">${d.state || 'CRUISE'}</span></div>
      </div>
    `;
    droneCardsContainer.appendChild(card);
  }
}

// Update Task Table UI
function updateTaskTable() {
  taskTableBody.innerHTML = '';
  for (const t of tasks) {
    const row = document.createElement('tr');
    const badgeClass = `badge-${t.status.toLowerCase()}`;
    const srcTag = t.source === 'MODEL_INFERENCE' 
      ? '<span style="color:#10b981; font-weight:600; font-size:10px; margin-left:4px;">[REAL MODEL]</span>' 
      : '<span style="color:#64748b; font-weight:500; font-size:10px; margin-left:4px;">[DEMO]</span>';
    row.innerHTML = `
      <td>${t.id} ${srcTag}</td>
      <td>${t.spottedBy}</td>
      <td>${t.lat.toFixed(5)}, ${t.lon.toFixed(5)}</td>
      <td>${t.assignedDroneId || 'PENDING'}</td>
      <td><span class="badge-status ${badgeClass}">${t.status}</span></td>
    `;
    taskTableBody.appendChild(row);
  }
  document.getElementById('task-badge').textContent = `${tasks.length} Tracked`;
}

// Update Header Metrics
function updateMetrics() {
  survivorsCountVal.textContent = tasks.length;
  const verifiedCount = tasks.filter(t => t.status === 'VERIFIED').length;
  deliveriesCountVal.textContent = verifiedCount;
}

// Click Canvas to Detect Survivor
canvas.addEventListener('click', (e) => {
  const rect = canvas.getBoundingClientRect();
  const px = e.clientX - rect.left;
  const py = e.clientY - rect.top;
  const geo = unproject(px, py, canvas.parentElement.clientWidth, canvas.parentElement.clientHeight);

  // Identify closest search drone
  let closest = 'S1';
  let minDist = Infinity;
  for (const d of drones.filter(d => d.role === 'search')) {
    const dist = Math.hypot(d.lat - geo.lat, d.lon - geo.lon);
    if (dist < minDist) {
      minDist = dist;
      closest = d.id;
    }
  }

  injectSurvivor(geo.lat, geo.lon, closest);
});

// Event Listeners for Sim Controls
document.getElementById('btn-play-pause').addEventListener('click', () => {
  isPlaying = !isPlaying;
  document.getElementById('play-icon').textContent = isPlaying ? '⏸' : '▶';
  document.getElementById('play-text').textContent = isPlaying ? 'Pause Sim' : 'Resume Sim';
});

document.getElementById('btn-step').addEventListener('click', () => {
  simTick(1.0);
});

document.getElementById('btn-speed').addEventListener('click', () => {
  if (simSpeed === 1.0) simSpeed = 2.0;
  else if (simSpeed === 2.0) simSpeed = 5.0;
  else simSpeed = 1.0;
  document.getElementById('btn-speed').textContent = `Speed: ${simSpeed}x`;
});

document.getElementById('btn-toggle-sectors').addEventListener('click', () => {
  showSectors = !showSectors;
  document.getElementById('btn-toggle-sectors').textContent = `Sectors: ${showSectors ? 'ON' : 'OFF'}`;
});

document.getElementById('btn-toggle-paths').addEventListener('click', () => {
  showPaths = !showPaths;
  document.getElementById('btn-toggle-paths').textContent = `Paths: ${showPaths ? 'ON' : 'OFF'}`;
});

document.getElementById('btn-reset-view').addEventListener('click', () => {
  resizeCanvas();
});

document.getElementById('btn-inject-survivor').addEventListener('click', () => {
  const randomLat = GEOFENCE.minLat + 0.0004 + Math.random() * (GEOFENCE.maxLat - GEOFENCE.minLat - 0.0008);
  const randomLon = GEOFENCE.minLon + 0.0004 + Math.random() * (GEOFENCE.maxLon - GEOFENCE.minLon - 0.0008);
  injectSurvivor(randomLat, randomLon, ['S1', 'S2', 'S3'][Math.floor(Math.random() * 3)]);
});

document.getElementById('btn-inject-battery').addEventListener('click', () => {
  const busyDelivery = drones.find(d => d.role === 'delivery' && d.status === 'BUSY');
  if (busyDelivery) {
    busyDelivery.battery = 0.18;
    addLog(`Failsafe Event: Drone ${busyDelivery.id} battery dropped to 18% (<= 20% RTL threshold). Immediate RTL triggered!`, 'critical');
    safetyLevelVal.textContent = 'CRITICAL';
    safetyLevelVal.className = 'metric-value text-danger';
    busyDelivery.state = 'RTL';
  } else {
    drones[0].battery = 0.19;
    addLog(`Failsafe Event: S1 battery dropped to 19%. Failsafe warning triggered.`, 'warning');
  }
  renderDroneCards();
});

document.getElementById('btn-emergency-rtl').addEventListener('click', () => {
  addLog('EMERGENCY: Fleet-wide abort command issued from GCS. All drones returning to base.', 'critical');
  missionState = 'RETURNING';
  missionStateVal.textContent = 'RETURNING';
  missionStateVal.className = 'status-val text-danger';
  safetyLevelVal.textContent = 'ABORT / RTL';
  safetyLevelVal.className = 'metric-value text-danger';

  for (const d of drones) {
    if (d.role === 'delivery') {
      d.state = 'RTL';
    }
  }
});

document.getElementById('btn-reset').addEventListener('click', () => {
  tasks = [];
  taskCounter = 1;
  drones.forEach((d, idx) => {
    d.battery = [0.96, 0.94, 0.98, 0.88, 0.95, 0.81][idx];
    d.trail = [];
    if (d.role === 'delivery') {
      d.status = 'READY';
      d.state = 'STANDBY';
      d.activeTaskId = null;
      d.lat = GEOFENCE.baseLat;
      d.lon = GEOFENCE.baseLon;
      d.alt = 0.0;
    } else {
      d.status = 'SEARCHING';
      d.currentWpIdx = 0;
    }
  });
  missionState = 'SEARCHING';
  missionStateVal.textContent = 'SEARCHING';
  missionStateVal.className = 'status-val text-success';
  safetyLevelVal.textContent = 'NOMINAL';
  safetyLevelVal.className = 'metric-value text-success';
  addLog('Mission reset to initial state.', 'info');
  createInitialScenario();
  updateTaskTable();
  updateMetrics();
});

document.getElementById('btn-clear-logs').addEventListener('click', () => {
  terminalFeed.innerHTML = '';
});

// Animation Loop
let lastTime = performance.now();
function animLoop(now) {
  const dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (isPlaying && dt > 0 && dt < 1.0) {
    simTick(dt * simSpeed);
  } else {
    renderTacticalMap();
  }

  requestAnimationFrame(animLoop);
}

// Boot
createInitialScenario();
renderDroneCards();
updateTaskTable();
requestAnimationFrame(animLoop);
