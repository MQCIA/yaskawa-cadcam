// Client-side welding "program" model — mirrors the backend path planner +
// DX200 postprocessor closely enough to demo a finished-looking welding job
// entirely in the browser (so it also works on the static GitHub Pages preview).
//
// Inspired by automatic offline-programming tools (ArcNC / MotoSim): a CAD part
// on the positioner, an auto-detected fillet seam, a generated weld path with
// approach/retract air-moves, torch-orientation frames, an ARCON/ARCOF waypoint
// list and a weld-schedule table.
//
// Everything here is an APPROXIMATE placeholder — validate in MotoSim before
// real hardware use.

export type Vec3 = [number, number, number];
export type MoveType = "MOVJ" | "MOVL";

export type Waypoint = {
  id: string;
  pos: Vec3;
  move: MoveType;
  speedLabel: string;
  arc: boolean;
  kind: "approach" | "weld" | "retract";
  tag?: "ARCON" | "ARCOF";
};

export type WeldScheduleRow = {
  condition: number;
  job: number;
  currentA: number;
  voltageV: number;
  wireMmin: number;
  travelCmMin: number;
  process: string;
};

export type WeldProgram = {
  name: string;
  mount: Vec3;
  plates: { size: Vec3; center: Vec3 }[];
  seam: Vec3[];
  torchFrames: { pos: Vec3; dir: Vec3 }[];
  waypoints: Waypoint[];
  segDurations: number[];
  cycleSec: number;
  weldLenMm: number;
  schedule: WeldScheduleRow;
};

// Mirror of backend welding_config.LORCH_S8_SCHEDULES (EXAMPLE values only).
export const LORCH_S8_SCHEDULES: WeldScheduleRow[] = [
  { condition: 1, job: 1, currentA: 180, voltageV: 22.0, wireMmin: 6.5, travelCmMin: 40, process: "SpeedPulse" },
  { condition: 2, job: 2, currentA: 240, voltageV: 26.0, wireMmin: 9.0, travelCmMin: 35, process: "SpeedPulse" },
  { condition: 3, job: 3, currentA: 120, voltageV: 18.5, wireMmin: 4.0, travelCmMin: 45, process: "SpeedArc" },
];

// Mount point on work-station 1's positioner faceplate (world coords, metres).
export const DEMO_MOUNT: Vec3 = [1.95, 0.98, -1.2];

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const dist = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const norm = (a: Vec3): Vec3 => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};
const pad = (n: number) => `P${String(n).padStart(3, "0")}`;

/** Build a T-fillet demo weld program mounted on the given positioner. */
export function buildDemoProgram(mount: Vec3 = DEMO_MOUNT, condition = 1): WeldProgram {
  const sch =
    LORCH_S8_SCHEDULES.find((s) => s.condition === condition) ?? LORCH_S8_SCHEDULES[0];

  // Part: a base plate with an upright plate — a classic T-fillet coupon.
  const platesLocal: { size: Vec3; center: Vec3 }[] = [
    { size: [0.5, 0.02, 0.34], center: [0, 0.01, 0] },
    { size: [0.5, 0.24, 0.02], center: [0, 0.13, -0.15] },
  ];
  const plates = platesLocal.map((p) => ({ size: p.size, center: add(mount, p.center) }));

  // Fillet seam along X at the inner corner where the plates meet.
  const seamStart = add(mount, [-0.22, 0.03, -0.135]);
  const seamEnd = add(mount, [0.22, 0.03, -0.135]);

  const N = 12;
  const seam: Vec3[] = [];
  for (let i = 0; i <= N; i++) {
    const t = i / N;
    seam.push([
      seamStart[0] + (seamEnd[0] - seamStart[0]) * t,
      seamStart[1] + (seamEnd[1] - seamStart[1]) * t,
      seamStart[2] + (seamEnd[2] - seamStart[2]) * t,
    ]);
  }

  // Torch pushes into the corner at ~45° (nozzle direction, unit vector).
  const torchDir = norm([0, -1, -1]);
  const torchFrames = seam.map((pos) => ({ pos, dir: torchDir }));

  const lift: Vec3 = [0, 0.1, 0.06];
  const approach = add(seamStart, lift);
  const retract = add(seamEnd, lift);

  const weldLabel = `V=${sch.travelCmMin} cm/min`;
  const airLabel = "VJ=50%";

  const waypoints: Waypoint[] = [];
  waypoints.push({ id: pad(1), pos: approach, move: "MOVJ", speedLabel: airLabel, arc: false, kind: "approach" });
  waypoints.push({ id: pad(2), pos: seamStart, move: "MOVL", speedLabel: "V=15 cm/min", arc: true, kind: "weld", tag: "ARCON" });
  for (let i = 1; i < N; i++) {
    waypoints.push({ id: pad(i + 2), pos: seam[i], move: "MOVL", speedLabel: weldLabel, arc: true, kind: "weld" });
  }
  waypoints.push({ id: pad(N + 2), pos: seamEnd, move: "MOVL", speedLabel: weldLabel, arc: true, kind: "weld", tag: "ARCOF" });
  waypoints.push({ id: pad(N + 3), pos: retract, move: "MOVL", speedLabel: airLabel, arc: false, kind: "retract" });

  const airSpeed = 0.5; // m/s for air-moves
  const weldSpeed = sch.travelCmMin / 100 / 60; // cm/min -> m/s
  const segDurations: number[] = [];
  for (let i = 0; i < waypoints.length - 1; i++) {
    const a = waypoints[i];
    const b = waypoints[i + 1];
    const welding = a.arc && b.arc;
    segDurations.push(dist(a.pos, b.pos) / (welding ? weldSpeed : airSpeed));
  }
  const cycleSec = segDurations.reduce((s, d) => s + d, 0);
  const weldLenMm = dist(seamStart, seamEnd) * 1000;

  return {
    name: "DEMO_TFILLET",
    mount,
    plates,
    seam,
    torchFrames,
    waypoints,
    segDurations,
    cycleSec,
    weldLenMm,
    schedule: sch,
  };
}

/** Interpolate the TCP along the program at progress p in [0,1]. */
export function sampleProgram(prog: WeldProgram, p: number) {
  const target = Math.max(0, Math.min(1, p)) * prog.cycleSec;
  let acc = 0;
  for (let i = 0; i < prog.segDurations.length; i++) {
    const d = prog.segDurations[i];
    if (target <= acc + d || i === prog.segDurations.length - 1) {
      const local = d > 0 ? (target - acc) / d : 0;
      const a = prog.waypoints[i].pos;
      const b = prog.waypoints[i + 1].pos;
      const pos: Vec3 = [
        a[0] + (b[0] - a[0]) * local,
        a[1] + (b[1] - a[1]) * local,
        a[2] + (b[2] - a[2]) * local,
      ];
      return {
        pos,
        wpIndex: i,
        arcOn: prog.waypoints[i].arc && prog.waypoints[i + 1].arc,
        elapsedSec: target,
      };
    }
    acc += d;
  }
  const last = prog.waypoints[prog.waypoints.length - 1];
  return { pos: last.pos, wpIndex: prog.waypoints.length - 1, arcOn: false, elapsedSec: prog.cycleSec };
}

const f4 = (v: number) => v.toFixed(4);

/**
 * Client-side DX200 .JBI generator — mirrors the backend postprocessor format
 * (coordinated robot+station group) for offline demo/export. Structurally
 * plausible template only; validate on the real controller / MotoSim.
 */
export function generateJbi(prog: WeldProgram): string {
  const pts = prog.waypoints;
  const npos = pts.length;
  const lines: string[] = [];

  lines.push("/JOB");
  lines.push(`//NAME ${prog.name}`);
  lines.push("//POS");
  lines.push(`///NPOS ${npos},0,0,0,0,0`);
  lines.push("///TOOL 0");
  lines.push("///POSTYPE ROBOT");
  lines.push("///RECTAN");
  lines.push("///RCONF " + Array(24).fill(0).map((v, i) => (i === 0 ? 1 : v)).join(","));

  pts.forEach((p, i) => {
    const x = p.pos[0] * 1000;
    const y = p.pos[1] * 1000;
    const z = p.pos[2] * 1000;
    lines.push(`C${String(i).padStart(5, "0")}=${f4(x)},${f4(y)},${f4(z)},${f4(0)},${f4(0)},${f4(0)}`);
  });

  // Station axis (H1000D) held at 0° for this demo station.
  lines.push("//POS-EX S1E");
  pts.forEach((_, i) => lines.push(`EC${String(i).padStart(5, "0")}=${f4(0)}`));

  lines.push("//INST");
  const now = new Date();
  const stamp = `${now.getFullYear()}/${String(now.getMonth() + 1).padStart(2, "0")}/${String(
    now.getDate(),
  ).padStart(2, "0")} ${String(now.getHours()).padStart(2, "0")}:${String(now.getMinutes()).padStart(2, "0")}`;
  lines.push(`///DATE ${stamp}`);
  lines.push("///ATTR SC,RW");
  lines.push("///GROUP1 RB1,ST1");
  lines.push("NOP");

  const sch = prog.schedule;
  pts.forEach((p, i) => {
    lines.push(`SMOVL C${String(i).padStart(5, "0")} V=${f4(10)}`);
    if (p.tag === "ARCON") {
      lines.push(
        `' Lorch S8 job ${sch.job}: ${sch.currentA}A / ${sch.voltageV.toFixed(1)}V / wire ${sch.wireMmin.toFixed(
          1,
        )} m/min / ${sch.process}`,
      );
      lines.push(`ARCON ASF#(${sch.condition})`);
    }
    if (p.tag === "ARCOF") {
      lines.push("ARCOF");
    }
  });
  lines.push("END");

  return lines.join("\n") + "\n";
}
