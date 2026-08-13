"use strict";

const activeKeys = new Set();
const createdJointGroups = new Set();

let commandTimer = null;
let statusSocket = null;
let jointActionBusy = false;

const keyVectors = {
  KeyW: { x: 1, y: 0, z: 0 },
  KeyS: { x: -1, y: 0, z: 0 },
  KeyA: { x: 0, y: 1, z: 0 },
  KeyD: { x: 0, y: -1, z: 0 },
  KeyQ: { x: 0, y: 0, z: 1 },
  KeyE: { x: 0, y: 0, z: -1 },
};

const stateNames = {
  0: "NONE",
  1: "IDLE",
  2: "ENABLE",
  3: "EXECUTING",
  4: "MAJOR FAULT",
  5: "MINOR FAULT",
};

const jointGroupTitles = {
  torso: "Thân",
  right_arm: "Tay phải",
  left_arm: "Tay trái",
  head: "Đầu",
};

const $ = (id) => document.getElementById(id);

const connectionBadge = $("connectionBadge");
const controlState = $("controlState");
const streamState = $("streamState");
const collisionState = $("collisionState");
const jointCount = $("jointCount");
const jointActionState = $("jointActionState");
const odomX = $("odomX");
const odomY = $("odomY");
const odomYaw = $("odomYaw");
const linearSpeedInput = $("linearSpeed");
const angularSpeedInput = $("angularSpeed");
const linearSpeedText = $("linearSpeedText");
const angularSpeedText = $("angularSpeedText");
const jointStepInput = $("jointStep");
const jointMinimumTimeInput = $("jointMinimumTime");
const jointGroupsElement = $("jointGroups");
const jointWaitingMessage = $("jointWaitingMessage");
const jointDiagnostics = $("jointDiagnostics");
const jointNameList = $("jointNameList");
const logOutput = $("logOutput");

function log(message) {
  const time = new Date().toLocaleTimeString("vi-VN");
  logOutput.textContent = `[${time}] ${message}\n${logOutput.textContent}`;
}

async function api(path, body = undefined) {
  const options = {
    method: "POST",
    headers: {},
  };

  if (body !== undefined) {
    options.headers["Content-Type"] = "application/json";
    options.body = JSON.stringify(body);
  }

  const response = await fetch(path, options);

  let data;
  try {
    data = await response.json();
  } catch {
    data = { detail: `HTTP ${response.status}` };
  }

  if (!response.ok) {
    throw new Error(data.detail || `HTTP ${response.status}`);
  }

  return data;
}

function clamp(value) {
  return Math.max(-1, Math.min(1, value));
}

function currentDirection() {
  let x = 0;
  let y = 0;
  let z = 0;

  for (const key of activeKeys) {
    const vector = keyVectors[key];
    if (!vector) continue;

    x += vector.x;
    y += vector.y;
    z += vector.z;
  }

  return {
    x: clamp(x),
    y: clamp(y),
    z: clamp(z),
  };
}

async function sendCurrentCommand() {
  const direction = currentDirection();
  const linearSpeed = Number(linearSpeedInput.value);
  const angularSpeed = Number(angularSpeedInput.value);

  try {
    await api("/api/velocity", {
      linear_x: direction.x * linearSpeed,
      linear_y: direction.y * linearSpeed,
      angular_z: direction.z * angularSpeed,
    });
  } catch (error) {
    log(`Lỗi gửi vận tốc: ${error.message}`);
    await hardStop();
  }
}

function startMotion(key) {
  if (!keyVectors[key] || jointActionBusy) return;

  activeKeys.add(key);

  document
    .querySelectorAll(`[data-key="${key}"]`)
    .forEach((button) => button.classList.add("active"));

  if (commandTimer === null) {
    commandTimer = window.setInterval(sendCurrentCommand, 100);
  }

  void sendCurrentCommand();
}

async function stopMotion(key) {
  activeKeys.delete(key);

  document
    .querySelectorAll(`[data-key="${key}"]`)
    .forEach((button) => button.classList.remove("active"));

  if (activeKeys.size === 0) {
    if (commandTimer !== null) {
      clearInterval(commandTimer);
      commandTimer = null;
    }

    await hardStop();
    return;
  }

  await sendCurrentCommand();
}

async function hardStop() {
  activeKeys.clear();

  document
    .querySelectorAll(".motion-button")
    .forEach((button) => button.classList.remove("active"));

  if (commandTimer !== null) {
    clearInterval(commandTimer);
    commandTimer = null;
  }

  try {
    await api("/api/stop");
  } catch (error) {
    log(`Không gửi được STOP: ${error.message}`);
  }
}

function setJointButtonsDisabled(disabled) {
  document
    .querySelectorAll(".joint-command-button")
    .forEach((button) => {
      button.disabled = disabled;
    });

  $("readyPoseButton").disabled = disabled;
  $("zeroPoseButton").disabled = disabled;
}

function updateStatus(status) {
  if (status.connected) {
    connectionBadge.textContent = "ROS 2: Đã kết nối";
    connectionBadge.className = "badge online";
  } else {
    connectionBadge.textContent = "ROS 2: Mất kết nối";
    connectionBadge.className = "badge offline";
  }

  controlState.textContent =
    stateNames[status.control_manager_state]
    || `UNKNOWN (${status.control_manager_state})`;

  streamState.textContent = status.robot_stream_state ? "ON" : "OFF";
  collisionState.textContent = status.collision ? "CÓ" : "Không";
  jointCount.textContent = String(status.joint_count ?? 0);

  odomX.textContent = `${Number(status.odom?.x ?? 0).toFixed(3)} m`;
  odomY.textContent = `${Number(status.odom?.y ?? 0).toFixed(3)} m`;
  odomYaw.textContent = `${Number(status.odom?.yaw ?? 0).toFixed(3)} rad`;

  jointActionBusy = Boolean(status.joint_action?.busy);
  jointActionState.textContent = status.joint_action?.state || "idle";

  setJointButtonsDisabled(jointActionBusy);
  updateJointGroups(
    status.joint_groups || {},
    status.joint_names || [],
  );
}

function jointElementId(group, index) {
  return `joint-value-${group}-${index}`;
}

function createJointGroup(group, names, positions) {
  const section = document.createElement("article");
  section.className = "joint-group";
  section.dataset.group = group;

  const title = document.createElement("h3");
  title.textContent = jointGroupTitles[group] || group;
  section.appendChild(title);

  positions.forEach((position, index) => {
    const row = document.createElement("div");
    row.className = "joint-row";

    const name = document.createElement("span");
    name.className = "joint-name";
    name.textContent = names[index] || `${group}_${index}`;
    name.title = name.textContent;

    const minusButton = document.createElement("button");
    minusButton.type = "button";
    minusButton.className = "joint-command-button";
    minusButton.textContent = "−";
    minusButton.title = `Giảm ${name.textContent}`;

    const value = document.createElement("strong");
    value.id = jointElementId(group, index);
    value.textContent = `${Number(position).toFixed(3)} rad`;

    const plusButton = document.createElement("button");
    plusButton.type = "button";
    plusButton.className = "joint-command-button";
    plusButton.textContent = "+";
    plusButton.title = `Tăng ${name.textContent}`;

    minusButton.addEventListener("click", () => {
      void nudgeJoint(
        group,
        index,
        -Number(jointStepInput.value),
      );
    });

    plusButton.addEventListener("click", () => {
      void nudgeJoint(
        group,
        index,
        Number(jointStepInput.value),
      );
    });

    row.append(name, minusButton, value, plusButton);
    section.appendChild(row);
  });

  jointGroupsElement.appendChild(section);
  createdJointGroups.add(group);
}

function updateJointGroups(groups, allJointNames = []) {
  let hasAnyGroup = false;

  for (const [group, data] of Object.entries(groups)) {
    if (!Array.isArray(data.positions) || data.positions.length === 0) {
      continue;
    }

    hasAnyGroup = true;

    if (!createdJointGroups.has(group)) {
      createJointGroup(
        group,
        data.names || [],
        data.positions,
      );
    }

    data.positions.forEach((position, index) => {
      const element = $(jointElementId(group, index));
      if (element) {
        element.textContent = `${Number(position).toFixed(3)} rad`;
      }
    });
  }

  if (jointWaitingMessage) {
    jointWaitingMessage.hidden = hasAnyGroup;
  }

  if (jointDiagnostics && jointNameList) {
    const hasNames = allJointNames.length > 0;
    jointDiagnostics.hidden = !hasNames || hasAnyGroup;
    jointNameList.textContent = allJointNames.join("\n");
  }
}


async function nudgeJoint(group, jointIndex, delta) {
  if (jointActionBusy) {
    log("Một lệnh khớp khác đang chạy.");
    return;
  }

  await hardStop();

  const minimumTime = Number(jointMinimumTimeInput.value);

  log(
    `Điều khiển ${group}[${jointIndex}], `
    + `delta=${delta.toFixed(2)} rad, `
    + `thời gian=${minimumTime.toFixed(2)} s`
  );

  try {
    setJointButtonsDisabled(true);

    const result = await api("/api/joints/nudge", {
      group,
      joint_index: jointIndex,
      delta,
      minimum_time: minimumTime,
    });

    log(`Lệnh khớp hoàn thành: ${result.finish_code}`);
  } catch (error) {
    log(`Lỗi điều khiển khớp: ${error.message}`);
  } finally {
    setJointButtonsDisabled(false);
  }
}

function connectStatusSocket() {
  const protocol =
    location.protocol === "https:" ? "wss:" : "ws:";

  statusSocket = new WebSocket(
    `${protocol}//${location.host}/ws/status`
  );

  statusSocket.onopen = () => {
    log("Đã kết nối WebSocket trạng thái.");
  };

  statusSocket.onmessage = (event) => {
    const status = JSON.parse(event.data);
    updateStatus(status);
  };

  statusSocket.onerror = () => {
    log("Lỗi WebSocket.");
  };

  statusSocket.onclose = () => {
    connectionBadge.textContent = "Backend: Mất kết nối";
    connectionBadge.className = "badge offline";

    void hardStop();

    setTimeout(connectStatusSocket, 1000);
  };
}

linearSpeedInput.addEventListener("input", () => {
  linearSpeedText.textContent =
    `${Number(linearSpeedInput.value).toFixed(2)} m/s`;
});

angularSpeedInput.addEventListener("input", () => {
  angularSpeedText.textContent =
    `${Number(angularSpeedInput.value).toFixed(2)} rad/s`;
});

document
  .querySelectorAll(".motion-button")
  .forEach((button) => {
    const key = button.dataset.key;

    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      startMotion(key);
    });

    button.addEventListener("pointerup", () => {
      void stopMotion(key);
    });

    button.addEventListener("pointercancel", () => {
      void stopMotion(key);
    });

    button.addEventListener("lostpointercapture", () => {
      void stopMotion(key);
    });
  });

window.addEventListener("keydown", (event) => {
  if (
    event.target instanceof HTMLInputElement
    || event.target instanceof HTMLTextAreaElement
  ) {
    return;
  }

  if (event.code === "Space") {
    event.preventDefault();
    void hardStop();
    return;
  }

  if (keyVectors[event.code] && !event.repeat) {
    event.preventDefault();
    startMotion(event.code);
  }
});

window.addEventListener("keyup", (event) => {
  if (keyVectors[event.code]) {
    event.preventDefault();
    void stopMotion(event.code);
  }
});

window.addEventListener("blur", () => {
  void hardStop();
});

window.addEventListener("beforeunload", () => {
  navigator.sendBeacon("/api/stop");
});

$("prepareButton").addEventListener("click", async () => {
  log("Đang chuẩn bị robot...");

  try {
    const result = await api("/api/prepare");
    log(result.message);
  } catch (error) {
    log(`Chuẩn bị thất bại: ${error.message}`);
  }
});

$("streamOnButton").addEventListener("click", async () => {
  try {
    const result = await api("/api/stream", {
      enabled: true,
    });

    log(result.message || "Đã bật stream.");
  } catch (error) {
    log(`Không bật được stream: ${error.message}`);
  }
});

$("streamOffButton").addEventListener("click", async () => {
  await hardStop();

  try {
    const result = await api("/api/stream", {
      enabled: false,
    });

    log(result.message || "Đã tắt stream.");
  } catch (error) {
    log(`Không tắt được stream: ${error.message}`);
  }
});

$("cancelButton").addEventListener("click", async () => {
  await hardStop();

  try {
    const result = await api("/api/cancel");
    log(result.message || "Đã hủy điều khiển.");
  } catch (error) {
    log(`Không hủy được điều khiển: ${error.message}`);
  }
});

$("emergencyStopButton").addEventListener("click", hardStop);
$("centerStopButton").addEventListener("click", hardStop);

$("readyPoseButton").addEventListener("click", async () => {
  if (jointActionBusy) {
    log("Một lệnh khớp khác đang chạy.");
    return;
  }

  await hardStop();
  log("Đang đưa robot đến tư thế co tay...");

  try {
    setJointButtonsDisabled(true);
    const result = await api("/api/joints/ready");
    log(`Tư thế co tay: ${result.finish_code}`);
  } catch (error) {
    log(`Tư thế co tay thất bại: ${error.message}`);
  } finally {
    setJointButtonsDisabled(false);
  }
});

$("zeroPoseButton").addEventListener("click", async () => {
  if (jointActionBusy) {
    log("Một lệnh khớp khác đang chạy.");
    return;
  }

  await hardStop();

  log("Đang đưa thân, hai tay và đầu về tư thế 0...");

  try {
    setJointButtonsDisabled(true);

    const result = await api("/api/joints/zero");
    log(`Zero pose: ${result.finish_code}`);
  } catch (error) {
    log(`Zero pose thất bại: ${error.message}`);
  } finally {
    setJointButtonsDisabled(false);
  }
});

$("clearLogButton").addEventListener("click", () => {
  logOutput.textContent = "";
});

connectStatusSocket();
