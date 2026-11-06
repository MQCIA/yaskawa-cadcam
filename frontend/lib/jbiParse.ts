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

export type ParsedJbi = {
  program: WeldProgram;
  sourceText: string;
  postype: "PULSE" | "RECTAN" | "UNKNOWN";
  warnings: string[];
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
): { pos: Vec3; dir: Vec3; jointsDeg?: number[] } {
  if (!entry) {
    warnings.push(`Brak pozycji C${String(cIdx).padStart(5, "0")}`);
    return { pos: [1.1, 0.98, -1.2], dir: [0, -Math.SQRT1_2, -Math.SQRT1_2] };
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
  return { pos: fk.pos, dir: fk.dir, jointsDeg };
}

function inferKind(
  move: MoveType,
  arcOn: boolean,
  index: number,
  total: number,
  prevWasWeld: boolean,
): Waypoint["kind"] {
  if (move === "MOVS") return "sense";
  if (arcOn) return "weld";
  if (index === 0) return "approach";
  if (index === total - 1) return "retract";
  if (prevWasWeld) return "retract";
  if (move === "MOVJ") return "clearance";
  return "travel";
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
  let pendingTimer: number | undefined;
  let argf: number | null = null;
  let weaveNo = 21;
  const moveBuf: Array<{
    c: number;
    move: MoveType;
    speedLabel: string;
    arc: boolean;
    tag?: "ARCON" | "ARCOF" | "TOUCH";
    timerSec?: number;
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
    if (ev.type === "wvof") continue;
    if (ev.type === "timer") {
      pendingTimer = ev.sec;
      continue;
    }
    if (ev.type === "arcon") {
      arcPending = true;
      arcOn = true;
      continue;
    }
    if (ev.type === "arcof") {
      // Tack in place: ARCON / TIMER / ARCOF with no motion — weld at last C#.
      if (
        arcPending &&
        pendingTimer != null &&
        pendingTimer >= 1.5 &&
        moveBuf.length > 0
      ) {
        const last = moveBuf[moveBuf.length - 1];
        moveBuf.push({
          c: last.c,
          move: "MOVL",
          speedLabel: "V=5.0",
          arc: true,
          tag: "ARCON",
          timerSec: pendingTimer,
        });
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
      continue;
    }
    if (ev.type === "move") {
      const isCart = ev.move === "MOVL" || ev.move === "MOVC";
      const tag: "ARCON" | "ARCOF" | "TOUCH" | undefined = arcPending
        ? "ARCON"
        : ev.move === "MOVS"
          ? "TOUCH"
          : undefined;
      const timerSec =
        arcPending && pendingTimer != null && pendingTimer >= 1.5
          ? pendingTimer
          : undefined;
      moveBuf.push({
        c: ev.c,
        move: ev.move,
        speedLabel: ev.speedLabel,
        // Only cartesian weld moves carry the arc; MOVJ during an open arc is air.
        arc: arcOn && isCart,
        tag,
        timerSec,
      });
      if (arcPending) {
        arcPending = false;
        pendingTimer = undefined;
      }
    }
  }

  if (!moveBuf.length) {
    throw new Error("Brak ruchów MOVJ/MOVL/MOVC/MOVS w sekcji //INST");
  }

  const waypoints: Waypoint[] = [];
  const torchFrames: { pos: Vec3; dir: Vec3 }[] = [];
  const seam: Vec3[] = [];
  let prevWasWeld = false;

  moveBuf.forEach((m, i) => {
    const pose = resolvePose(positions.get(m.c), warnings, m.c);
    const kind = inferKind(m.move, m.arc, i, moveBuf.length, prevWasWeld);
    const wp: Waypoint = {
      id: pad(i + 1),
      pos: pose.pos,
      move: m.move,
      speedLabel: m.speedLabel,
      arc: m.arc,
      kind,
      tag: m.tag,
      timerSec: m.timerSec,
      bc: bc.get(m.c),
      ec: ec.get(m.c),
      jointsDeg: pose.jointsDeg,
    };
    waypoints.push(wp);
    if (kind === "weld") {
      seam.push(pose.pos);
      torchFrames.push({ pos: pose.pos, dir: pose.dir });
      prevWasWeld = true;
    } else if (kind === "sense") {
      prevWasWeld = false;
    } else if (kind === "retract" || kind === "clearance" || kind === "travel") {
      prevWasWeld = kind === "retract" ? false : prevWasWeld;
    }
  });

  // Ensure ARCON/ARCOF tags on contiguous weld runs.
  let i = 0;
  while (i < waypoints.length) {
    if (waypoints[i].kind !== "weld") {
      i++;
      continue;
    }
    const start = i;
    while (i < waypoints.length && waypoints[i].kind === "weld") i++;
    const end = i - 1;
    waypoints[start] = {
      ...waypoints[start],
      tag: "ARCON",
      arc: true,
      timerSec: waypoints[start].timerSec,
    };
    if (start === end) {
      waypoints[start] = {
        ...waypoints[start],
        timerSec: waypoints[start].timerSec ?? 2.0,
      };
    } else {
      waypoints[end] = { ...waypoints[end], tag: "ARCOF", arc: true };
      for (let j = start + 1; j < end; j++) {
        waypoints[j] = { ...waypoints[j], tag: undefined, arc: true };
      }
    }
  }

  const weldPts = waypoints.filter((w) => w.kind === "weld").map((w) => w.pos);
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
      : waypoints[0]?.pos ?? [1.1, 0.98, -1.2];

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
    },
  };

  const { segDurations, cycleSec } = computeDx200Durations(draft);
  const program: WeldProgram = { ...draft, segDurations, cycleSec };

  return {
    program,
    sourceText: trimmed.endsWith("\n") ? trimmed : `${trimmed}\n`,
    postype,
    warnings,
  };
}

export function isLikelyJbi(text: string): boolean {
  return /^\/JOB\b/im.test(text.trim());
}
