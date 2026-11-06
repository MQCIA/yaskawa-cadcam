/**
 * Parse DX200 INFORM ``.JBI`` into a ``WeldProgram`` for offline edit / preview.
 *
 * Supports RECTAN C# (mm + RPY) and PULSE C# (S..T → approx FK via dx200Sim).
 * ARCON/ARCOF/TIMER/WVON/MACRO1 ARGFn and MOVJ/MOVL/MOVC/MOVS are mapped to
 * waypoints; unknown INST lines are ignored with a warning.
 */

import { fkTcp } from "@/lib/dx200Sim";
import {
  CELL_PULSE,
  LORCH_S8_SCHEDULES,
  type MoveType,
  type Vec3,
  type Waypoint,
  type WeldProgram,
  type WeldScheduleRow,
} from "@/lib/weldProgram";
import { computeDx200Durations } from "@/lib/dx200Sim";

/**
 * RECT-X placeholder lead (matches backend dx200_trajectory heuristic):
 * ~0.002 mm/pulse → metres. Used only for carriage slider sync, not path XYZ.
 */
export const BC_M_PER_PULSE = 0.002 / 1000;

/** Carriage Z from BC pulses relative to a reference pulse (usually first C#). */
export function railZFromBc(bc: number, refBc: number): number {
  return (bc - refBc) * BC_M_PER_PULSE;
}

/** Fallback mount when a job has no weld points yet. */
export const JBI_ALIGN_MOUNT: Vec3 = [1.1, 0.98, -1.2];

export type ParsedJbi = {
  program: WeldProgram;
  sourceText: string;
  postype: "PULSE" | "RECTAN" | "UNKNOWN";
  warnings: string[];
  /** BC pulse used as Z=0 for RECT-X when postype is PULSE. */
  bcRef?: number;
};

const pad = (n: number) => `P${String(n).padStart(3, "0")}`;

export function jointPulsesToDeg(pulses: number[]): number[] {
  if (pulses.length !== 6) throw new Error("Expected 6 joint pulses");
  return pulses.map((p, i) => p / CELL_PULSE.robotPulsesPerDeg[i]);
}

/** Rough torch −Z from Motoman RECTAN Rx,Ry,Rz (deg) — inverse of torchDirToRpy. */
function rpyToTorchDir(rx: number, ry: number, rz: number): Vec3 {
  void rz;
  const pitch = (ry * Math.PI) / 180;
  const yaw = (rx * Math.PI) / 180;
  // Prefer a downward-forward wire when RPY is near (180,0,0) style poses.
  const d: Vec3 = [
    Math.sin(yaw) * Math.cos(pitch),
    -Math.abs(Math.cos(pitch)),
    -Math.cos(yaw) * Math.cos(pitch),
  ];
  const l = Math.hypot(d[0], d[1], d[2]) || 1;
  return [d[0] / l, d[1] / l, d[2] / l];
}

function scheduleForArgf(argf: number | null): WeldScheduleRow {
  if (argf != null) {
    const hit = LORCH_S8_SCHEDULES.find((s) => s.job === argf);
    if (hit) return { ...hit };
  }
  return { ...LORCH_S8_SCHEDULES[0] };
}

type PosEntry =
  | { kind: "rectan"; xyzMm: Vec3; rpy: Vec3 }
  | { kind: "pulse"; pulses: number[] };

function parsePosBlock(lines: string[]): {
  name: string;
  positions: Map<number, PosEntry>;
  bc: Map<number, number>;
  ec: Map<number, [number, number]>;
  postype: "PULSE" | "RECTAN" | "UNKNOWN";
} {
  let name = "IMPORTED";
  const positions = new Map<number, PosEntry>();
  const bc = new Map<number, number>();
  const ec = new Map<number, [number, number]>();
  let postype: "PULSE" | "RECTAN" | "UNKNOWN" = "UNKNOWN";
  let section: "none" | "pos" | "inst" = "none";

  for (const raw of lines) {
    const line = raw.trim();
    if (!line || line.startsWith("'")) continue;
    if (line.startsWith("//NAME")) {
      name = line.replace(/^\/\/NAME\s+/i, "").trim() || name;
      continue;
    }
    if (line === "//POS") {
      section = "pos";
      continue;
    }
    if (line === "//INST") {
      section = "inst";
      continue;
    }
    if (section !== "pos") continue;

    if (line === "///RECTAN") {
      postype = "RECTAN";
      continue;
    }
    if (line === "///PULSE" || line.startsWith("///POSTYPE PULSE")) {
      // EC block often re-enters PULSE after RECTAN C# — only flip when C# not yet filled as rectan.
      if (postype === "UNKNOWN") postype = "PULSE";
      continue;
    }
    if (line.startsWith("///POSTYPE USER") || line.startsWith("///POSTYPE ROBOT")) {
      if (postype === "UNKNOWN") postype = "RECTAN";
      continue;
    }

    const cMatch = /^C(\d+)\s*=\s*(.+)$/i.exec(line);
    if (cMatch) {
      const idx = Number(cMatch[1]);
      const nums = cMatch[2].split(",").map((s) => Number(s.trim()));
      if (nums.length >= 6 && nums.every((n) => Number.isFinite(n))) {
        const looksPulse =
          nums.every((n) => Number.isInteger(n)) &&
          nums.some((n) => Math.abs(n) > 360);
        if (looksPulse || postype === "PULSE") {
          positions.set(idx, {
            kind: "pulse",
            pulses: nums.slice(0, 6) as number[],
          });
          postype = "PULSE";
        } else {
          positions.set(idx, {
            kind: "rectan",
            xyzMm: [nums[0], nums[1], nums[2]],
            rpy: [nums[3], nums[4], nums[5]],
          });
          postype = "RECTAN";
        }
      }
      continue;
    }

    const bcMatch = /^BC(\d+)\s*=\s*(-?\d+)/i.exec(line);
    if (bcMatch) {
      bc.set(Number(bcMatch[1]), Number(bcMatch[2]));
      continue;
    }
    const ecMatch = /^EC(\d+)\s*=\s*(-?\d+)\s*(?:,\s*(-?\d+))?/i.exec(line);
    if (ecMatch) {
      ec.set(Number(ecMatch[1]), [
        Number(ecMatch[2]),
        Number(ecMatch[3] ?? 0),
      ]);
    }
  }

  return { name, positions, bc, ec, postype };
}

type InstEvent =
  | { type: "move"; c: number; move: MoveType; speedLabel: string }
  | { type: "arcon" }
  | { type: "arcof" }
  | { type: "timer"; sec: number }
  | { type: "macro"; argf: number }
  | { type: "wvon"; wev: number }
  | { type: "wvof" };

function parseInstEvents(lines: string[], warnings: string[]): InstEvent[] {
  const events: InstEvent[] = [];
  let section = false;
  for (const raw of lines) {
    const line = raw.trim();
    if (!line || line.startsWith("'")) continue;
    if (line === "//INST") {
      section = true;
      continue;
    }
    if (!section) continue;
    if (line === "NOP" || line === "END" || line.startsWith("///")) continue;

    if (line === "ARCON") {
      events.push({ type: "arcon" });
      continue;
    }
    if (line === "ARCOF") {
      events.push({ type: "arcof" });
      continue;
    }
    if (line === "WVOF") {
      events.push({ type: "wvof" });
      continue;
    }
    const timer = /^TIMER\s+T\s*=\s*([0-9.]+)/i.exec(line);
    if (timer) {
      events.push({ type: "timer", sec: Number(timer[1]) });
      continue;
    }
    const macro = /^MACRO1\s+MJ#\(\s*0\s*\)\s+ARGF(\d+)/i.exec(line);
    if (macro) {
      events.push({ type: "macro", argf: Number(macro[1]) });
      continue;
    }
    const wvon = /^WVON\s+WEV#\(\s*(\d+)\s*\)/i.exec(line);
    if (wvon) {
      events.push({ type: "wvon", wev: Number(wvon[1]) });
      continue;
    }

    const mov =
      /^(MOVJ|MOVL|MOVC|MOVS)\s+C(\d+)\b(?:.*?\b(VJ|V)\s*=\s*([0-9.]+))?/i.exec(
        line,
      );
    if (mov) {
      const move = mov[1].toUpperCase() as MoveType;
      const c = Number(mov[2]);
      const unit = (mov[3] || (move === "MOVJ" ? "VJ" : "V")).toUpperCase();
      const val = mov[4] ? Number(mov[4]) : move === "MOVJ" ? 80 : 5;
      events.push({
        type: "move",
        c,
        move,
        speedLabel: `${unit}=${val.toFixed(1)}`,
      });
      continue;
    }

    if (
      /^(CALL|JUMP|WAIT|DOUT|DIN|PAUSE|ADVANCE|RET|SET|ADD|SUB|MUL|DIV|INC|DEC|IF|ENDIF|FOR|NEXT|SWITCH|CASE|DEFAULT|BREAK|ENDSWITCH)\b/i.test(
        line,
      )
    ) {
      warnings.push(`Pominięto instrukcję: ${line.slice(0, 48)}`);
    }
  }
  return events;
}

function resolvePose(
  entry: PosEntry | undefined,
  warnings: string[],
  cIdx: number,
): { pos: Vec3; dir: Vec3; jointsDeg?: number[]; pulses?: number[] } {
  if (!entry) {
    warnings.push(`Brak pozycji C${String(cIdx).padStart(5, "0")}`);
    return { pos: [...JBI_ALIGN_MOUNT], dir: [0, -Math.SQRT1_2, -Math.SQRT1_2] };
  }
  if (entry.kind === "rectan") {
    return {
      pos: [
        entry.xyzMm[0] / 1000,
        entry.xyzMm[1] / 1000,
        entry.xyzMm[2] / 1000,
      ],
      dir: rpyToTorchDir(entry.rpy[0], entry.rpy[1], entry.rpy[2]),
    };
  }
  const jointsDeg = jointPulsesToDeg(entry.pulses);
  const fk = fkTcp(jointsDeg);
  return { pos: fk.pos, dir: fk.dir, jointsDeg, pulses: entry.pulses };
}

/** Assign approach / clearance / retract around weld & sense runs. */
function assignAirKinds(waypoints: Waypoint[]): void {
  const firstWeld = waypoints.findIndex((w) => w.kind === "weld");
  let lastWeld = -1;
  for (let i = waypoints.length - 1; i >= 0; i--) {
    if (waypoints[i].kind === "weld") {
      lastWeld = i;
      break;
    }
  }
  for (let i = 0; i < waypoints.length; i++) {
    const w = waypoints[i];
    if (w.kind === "weld" || w.kind === "sense") continue;
    if (firstWeld < 0) {
      w.kind = i === 0 ? "approach" : i === waypoints.length - 1 ? "retract" : "travel";
      continue;
    }
    if (i < firstWeld) w.kind = i === 0 ? "approach" : "travel";
    else if (i > lastWeld) w.kind = "retract";
    else w.kind = "clearance";
  }
}

/**
 * Convert INFORM source into a browser ``WeldProgram``.
 * Throws if the text does not look like a Motoman job.
 */
export function parseJbi(text: string): ParsedJbi {
  const trimmed = text.replace(/^\uFEFF/, "").trim();
  if (!/^\/JOB\b/im.test(trimmed)) {
    throw new Error("Plik nie wygląda na .JBI (/JOB brak)");
  }
  const lines = trimmed.split(/\r?\n/);
  const warnings: string[] = [];
  const { name, positions, bc, ec, postype } = parsePosBlock(lines);
  const events = parseInstEvents(lines, warnings);

  let arcPending = false;
  let arcOn = false;
  /** TIMER waiting to attach — before ARCON (pre) or after ARCON (tack dwell). */
  let pendingTimer: number | undefined;
  let timerAfterArcon = false;
  let argf: number | null = null;
  let weaveNo = 21;
  const moveBuf: Array<{
    c: number;
    move: MoveType;
    speedLabel: string;
    arc: boolean;
    tag?: "ARCON" | "ARCOF" | "TOUCH";
    timerSec?: number;
    preArcTimerSec?: number;
    weaveNo?: number;
  }> = [];

  for (const ev of events) {
    if (ev.type === "macro") {
      argf = ev.argf;
      continue;
    }
    if (ev.type === "wvon") {
      weaveNo = ev.wev;
      continue;
    }
    if (ev.type === "wvof") {
      weaveNo = 0;
      continue;
    }
    if (ev.type === "timer") {
      pendingTimer = ev.sec;
      // If ARCON already open with no move yet, this TIMER is tack dwell.
      timerAfterArcon = arcPending || arcOn;
      continue;
    }
    if (ev.type === "arcon") {
      arcPending = true;
      arcOn = true;
      // TIMER before ARCON (e.g. T=0.50) stays in pendingTimer until first weld move.
      timerAfterArcon = false;
      continue;
    }
    if (ev.type === "arcof") {
      // Tack in place: ARCON [/ TIMER] / ARCOF with no intervening motion —
      // annotate the last C# (current TCP), do NOT invent a new point.
      if (arcPending && moveBuf.length > 0) {
        const last = moveBuf[moveBuf.length - 1];
        last.arc = true;
        last.tag = "ARCON";
        if (pendingTimer != null && timerAfterArcon) {
          last.timerSec = pendingTimer;
        }
        last.weaveNo = 0; // tack: no weave
      } else {
        for (let i = moveBuf.length - 1; i >= 0; i--) {
          if (moveBuf[i].arc) {
            if (!moveBuf[i].tag) moveBuf[i].tag = "ARCOF";
            break;
          }
        }
      }
      arcOn = false;
      arcPending = false;
      pendingTimer = undefined;
      timerAfterArcon = false;
      continue;
    }
    if (ev.type === "move") {
      const isCart = ev.move === "MOVL" || ev.move === "MOVC";
      const tag: "ARCON" | "ARCOF" | "TOUCH" | undefined = arcPending
        ? "ARCON"
        : ev.move === "MOVS"
          ? "TOUCH"
          : undefined;
      let timerSec: number | undefined;
      let preArcTimerSec: number | undefined;
      if (arcPending && pendingTimer != null) {
        if (timerAfterArcon && pendingTimer >= 1.5) {
          // Unusual: TIMER between ARCON and first move — treat as tack dwell.
          timerSec = pendingTimer;
        } else if (!timerAfterArcon) {
          // TIMER T=0.50 before ARCON (000.JBI pattern).
          preArcTimerSec = pendingTimer;
        }
      }
      moveBuf.push({
        c: ev.c,
        move: ev.move,
        speedLabel: ev.speedLabel,
        arc: arcOn && isCart,
        tag,
        timerSec,
        preArcTimerSec,
        weaveNo: arcOn && isCart ? weaveNo : undefined,
      });
      if (arcPending) {
        arcPending = false;
        pendingTimer = undefined;
        timerAfterArcon = false;
      }
    }
  }

  if (!moveBuf.length) {
    throw new Error("Brak ruchów MOVJ/MOVL/MOVC/MOVS w sekcji //INST");
  }

  const waypoints: Waypoint[] = [];
  let bcRef: number | undefined;
  for (const m of moveBuf) {
    const v = bc.get(m.c);
    if (typeof v === "number") {
      bcRef = v;
      break;
    }
  }

  const isPulseJob = Array.from(positions.values()).some((p) => p.kind === "pulse");

  moveBuf.forEach((m) => {
    const pose = resolvePose(positions.get(m.c), warnings, m.c);
    // Tack-in-place annotates a prior air C# as weld (arc+ARCON+timer).
    const kind: Waypoint["kind"] =
      m.move === "MOVS" ? "sense" : m.arc || m.tag === "ARCON" ? "weld" : "travel";
    const wp: Waypoint = {
      id: `C${String(m.c).padStart(5, "0")}`,
      cIndex: m.c,
      pos: pose.pos,
      move: m.move,
      speedLabel: m.speedLabel,
      arc: Boolean(m.arc || m.tag === "ARCON"),
      kind,
      tag: m.tag,
      timerSec: m.timerSec,
      preArcTimerSec: m.preArcTimerSec,
      weaveNo: m.weaveNo,
      bc: bc.get(m.c),
      ec: ec.get(m.c),
      jointsDeg: pose.jointsDeg,
      pulses: pose.pulses,
    };
    waypoints.push(wp);
  });

  assignAirKinds(waypoints);

  // Normalize ARCON/ARCOF on contiguous weld runs; never invent TIMER.
  {
    let i = 0;
    while (i < waypoints.length) {
      if (waypoints[i].kind !== "weld") {
        i++;
        continue;
      }
      const start = i;
      while (i < waypoints.length && waypoints[i].kind === "weld") i++;
      const end = i - 1;
      const startTimer = waypoints[start].timerSec;
      const pre = waypoints[start].preArcTimerSec;
      const wev = waypoints[start].weaveNo;
      waypoints[start] = {
        ...waypoints[start],
        tag: "ARCON",
        arc: true,
        timerSec: startTimer,
        preArcTimerSec: pre,
        weaveNo: wev,
      };
      if (start !== end) {
        waypoints[end] = { ...waypoints[end], tag: "ARCOF", arc: true };
        for (let j = start + 1; j < end; j++) {
          waypoints[j] = {
            ...waypoints[j],
            tag: undefined,
            arc: true,
            weaveNo: waypoints[j].weaveNo ?? wev,
          };
        }
      }
    }
  }

  if (isPulseJob) {
    warnings.push(
      "PULSE: TCP zostanie policzone z URDF MA2010 + TOOL 0 (jak robot w 3D).",
    );
  }

  const weldPts = waypoints.filter((w) => w.kind === "weld").map((w) => w.pos);
  const seam = [...weldPts];
  const torchFrames = waypoints
    .filter((w) => w.kind === "weld")
    .map((w) => ({
      pos: w.pos,
      dir: [0, -Math.SQRT1_2, -Math.SQRT1_2] as Vec3,
    }));

  let weldLenMm = 0;
  for (let k = 1; k < weldPts.length; k++) {
    weldLenMm +=
      Math.hypot(
        weldPts[k][0] - weldPts[k - 1][0],
        weldPts[k][1] - weldPts[k - 1][1],
        weldPts[k][2] - weldPts[k - 1][2],
      ) * 1000;
  }

  const mount: Vec3 =
    weldPts.length > 0
      ? [
          weldPts.reduce((s, p) => s + p[0], 0) / weldPts.length,
          weldPts.reduce((s, p) => s + p[1], 0) / weldPts.length,
          weldPts.reduce((s, p) => s + p[2], 0) / weldPts.length,
        ]
      : [...JBI_ALIGN_MOUNT];

  const denseJointAnglesDeg = waypoints.every((w) => w.jointsDeg?.length === 6)
    ? waypoints.map((w) => w.jointsDeg as number[])
    : undefined;

  const draft: WeldProgram = {
    name,
    mount,
    plates: [],
    seam: seam.length ? seam : weldPts,
    torchFrames: torchFrames.length
      ? torchFrames
      : waypoints.map((w) => ({
          pos: w.pos,
          dir: [0, -Math.SQRT1_2, -Math.SQRT1_2] as Vec3,
        })),
    waypoints,
    segDurations: [],
    cycleSec: 0,
    weldLenMm,
    schedule: scheduleForArgf(argf),
    meta: {
      collisionFree: false,
      touchSense: waypoints.some((w) => w.kind === "sense"),
      sequenceOptimized: false,
      singularitySafe: true,
      activeWeaveNo: weaveNo,
      denseJointAnglesDeg,
      bcRefPulse: bcRef,
      importedPulse: isPulseJob,
    },
  };

  const { segDurations, cycleSec } = computeDx200Durations(draft);
  const program: WeldProgram = { ...draft, segDurations, cycleSec };

  return {
    program,
    sourceText: trimmed.endsWith("\n") ? trimmed : `${trimmed}\n`,
    postype,
    warnings,
    bcRef,
  };
}

export function isLikelyJbi(text: string): boolean {
  return /^\/JOB\b/im.test(text.trim());
}
