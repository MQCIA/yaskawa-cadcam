"use client";

/**
 * UI modelled on Verbotics Weld:
 *  - Top ribbon (Plan / Settings / View)
 *  - Left dock: Workspace | Welds | Program + Details
 *  - Center: 3D viewer with simulation bar underneath
 *  - Right dock (ALWAYS visible): Robot — joint jog, Home/Zero, tool, status
 *
 * Manual joint jogging lives permanently in the right Robot dock, matching
 * Verbotics' Robot Positioning panel.
 */

import { useState, useEffect, useRef, useMemo, useCallback } from "react";
import dynamic from "next/dynamic";
import AxisSliders, {
  cellHomeJoints,
  defaultJoints,
  type Joints,
} from "@/components/AxisSliders";
import TorchCalibrationPanel from "@/components/TorchCalibrationPanel";
import Dx200MotorosChecklist from "@/components/Dx200MotorosChecklist";
import { useI18n, LanguageToggle } from "@/lib/i18n";
import type { SeamSegment } from "@/components/RobotWorkspace";
import { analyzeCad, calculateIk, forwardKinematics } from "@/lib/api";
import { ROBOT_MODELS, POSITIONER_MODELS } from "@/lib/models";
import {
  DEFAULT_TORCH_CALIBRATION,
  loadTorchCalibration,
  saveTorchCalibration,
  type TorchCalibration,
} from "@/lib/tool0";
import { TSL_CARRIAGE_TOP_Y } from "@/lib/trackMesh";
import {
  buildDemoProgram,
  buildHardDemoProgram,
  sampleProgram,
  generateJbi,
  waypointsToIkPoints,
  DEMO_MOUNT,
  DEMO_HARD_MOUNT,
  LORCH_S8_SCHEDULES,
  WEAVE_FILES,
  DEFAULT_WEAVE_NO,
  ensureTouchSense,
  optimizeSequence,
  planMotions,
  deleteWaypoint,
  insertIdleWaypoint,
  duplicateWaypoint,
  setWaypointKind,
  setWaypointPosition,
  type IdleInsertKind,
  type Vec3,
  type Waypoint,
  type WeldProgram,
} from "@/lib/weldProgram";
import {
  applySeamEdit,
  buildTimelineMarkers,
  buildTimelinePhases,
  diagnoseWaypoints,
  discoverSeams,
  informCLabel,
  type SeamEdit,
} from "@/lib/trajectoryViz";
import SeamInspector from "@/components/SeamInspector";
import SimTimeline from "@/components/SimTimeline";

const RobotWorkspace = dynamic(() => import("@/components/RobotWorkspace"), {
  ssr: false,
});

const PLAYBACK_BASE_SEC = 12;
/** Match RobotWorkspace station axles — teach FK → program frame. */
const STATION_POS_XZ: [number, number][] = [
  [1.6, -1.2],
  [1.6, 1.2],
];
const AXLE_Y = 0.95;

function mountStationIndex(mount: Vec3): number {
  let best = 0;
  let bestD = Infinity;
  STATION_POS_XZ.forEach(([x, z], i) => {
    const d = Math.hypot(x - mount[0], z - mount[2]);
    if (d < bestD) {
      bestD = d;
      best = i;
    }
  });
  return best;
}

/** Inverse of WeldScene part-table transform (world TCP → program XYZ). */
function worldToProgramFrame(
  world: Vec3,
  mount: Vec3,
  tableRotateDeg: number,
): Vec3 {
  const si = mountStationIndex(mount);
  const pivot: Vec3 = [STATION_POS_XZ[si][0], AXLE_Y, STATION_POS_XZ[si][1]];
  const partRotZ = (-tableRotateDeg * Math.PI) / 180;
  const dx = world[0] - pivot[0];
  const dy = world[1] - pivot[1];
  const dz = world[2] - pivot[2];
  const c = Math.cos(-partRotZ);
  const s = Math.sin(-partRotZ);
  return [pivot[0] + dx * c - dy * s, pivot[1] + dx * s + dy * c, pivot[2] + dz];
}

type LeftTab = "workspace" | "welds" | "program";
type Ribbon = "plan" | "settings" | "view";
type MobileTab = "scene" | "cad" | "robot" | "jbi";

const btnPrimary =
  "rounded bg-[#e87722] px-3 py-1.5 text-sm font-semibold text-white transition hover:brightness-110 disabled:cursor-not-allowed disabled:opacity-40";
const btnGhost =
  "rounded border border-slate-300 bg-white px-3 py-1.5 text-sm text-slate-700 transition hover:bg-slate-50 disabled:cursor-not-allowed disabled:opacity-40";
const btnRibbon =
  "flex cursor-pointer flex-col items-center gap-0.5 rounded px-2 py-1 text-[10px] leading-tight text-slate-700 transition hover:bg-orange-50 disabled:opacity-40 sm:px-3 sm:py-1.5 sm:text-[11px]";

export default function Home() {
  const { t } = useI18n();
  const [modelId, setModelId] = useState("ma2010");
  const [joints, setJoints] = useState<Joints>(cellHomeJoints());
  const [seams, setSeams] = useState<SeamSegment[]>([]);
  const [status, setStatus] = useState("");

  const [positionerId, setPositionerId] = useState<string | null>("turn_st1");
  const [railTravel, setRailTravel] = useState(0);
  const [stations, setStations] = useState([
    { tilt: 0, rotate: 0 },
    { tilt: 0, rotate: 0 },
  ]);

  const [program, setProgram] = useState<WeldProgram | null>(null);
  const [weldCondition, setWeldCondition] = useState(1);
  const [weaveNo, setWeaveNo] = useState(DEFAULT_WEAVE_NO);
  const [simT, setSimT] = useState(0);
  const [simPlaying, setSimPlaying] = useState(false);
  const [simSpeed, setSimSpeed] = useState(1);
  const [selectedWp, setSelectedWp] = useState<number | null>(null);
  const [selectedSeamId, setSelectedSeamId] = useState<string | null>(null);
  const [insertKind, setInsertKind] = useState<IdleInsertKind>("clearance");
  const [liveTcpMm, setLiveTcpMm] = useState<string | null>(null);

  // Verbotics-style: joint jog is always available; ON until the user plays a plan.
  const [manualJog, setManualJog] = useState(true);

  const [ribbon, setRibbon] = useState<Ribbon>("plan");
  const [leftTab, setLeftTab] = useState<LeftTab>("workspace");
  const [selectedStation, setSelectedStation] = useState<0 | 1 | null>(null);
  const [mobileTab, setMobileTab] = useState<MobileTab>("scene");
  const [torchCalib, setTorchCalib] = useState<TorchCalibration>(DEFAULT_TORCH_CALIBRATION);

  useEffect(() => {
    setTorchCalib(loadTorchCalibration());
  }, []);

  const onTorchCalibChange = useCallback((next: TorchCalibration) => {
    setTorchCalib(next);
    saveTorchCalibration(next);
  }, []);

  const rafRef = useRef<number | undefined>(undefined);
  const lastRef = useRef<number | undefined>(undefined);

  const setStation = (i: number, patch: Partial<{ tilt: number; rotate: number }>) =>
    setStations((prev) => prev.map((s, idx) => (idx === i ? { ...s, ...patch } : s)));

  useEffect(() => {
    if (!simPlaying || !program) return;
    lastRef.current = undefined;
    const tick = (ts: number) => {
      if (lastRef.current == null) lastRef.current = ts;
      const dt = (ts - lastRef.current) / 1000;
      lastRef.current = ts;
      setSimT((prev) => Math.min(1, prev + (dt * simSpeed) / PLAYBACK_BASE_SEC));
      rafRef.current = requestAnimationFrame(tick);
    };
    rafRef.current = requestAnimationFrame(tick);
    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, [simPlaying, simSpeed, program]);

  useEffect(() => {
    if (simT >= 1 && simPlaying) setSimPlaying(false);
  }, [simT, simPlaying]);

  // Live TCP from joint jog (for teach + Robot dock readout).
  useEffect(() => {
    if (!manualJog) {
      setLiveTcpMm(null);
      return;
    }
    let cancelled = false;
    const q = [joints.S, joints.L, joints.U, joints.R, joints.B, joints.T];
    const timer = window.setTimeout(() => {
      void forwardKinematics(q)
        .then((fk) => {
          if (cancelled) return;
          const tcp = fk.tcp_0 ?? fk.tcp;
          const world: Vec3 = [
            tcp.x,
            tcp.y + TSL_CARRIAGE_TOP_Y,
            tcp.z + railTravel,
          ];
          setLiveTcpMm(world.map((v) => (v * 1000).toFixed(1)).join(", "));
        })
        .catch(() => {
          if (!cancelled) setLiveTcpMm(null);
        });
    }, 120);
    return () => {
      cancelled = true;
      window.clearTimeout(timer);
    };
  }, [manualJog, joints, railTravel]);

  const wpProgress = useMemo(() => {
    if (!program) return [] as number[];
    const cum = [0];
    for (const d of program.segDurations) cum.push(cum[cum.length - 1] + d);
    return cum.map((c) => c / program.cycleSec);
  }, [program]);

  function applyProgram(prog: WeldProgram, mountZ: number, stationIdx: 0 | 1) {
    setProgram(prog);
    setSimT(0);
    setSimPlaying(false);
    setManualJog(true);
    setModelId("ma2010");
    setRailTravel(Math.max(-1.6, Math.min(1.6, mountZ)));
    setStation(stationIdx, { rotate: 0, tilt: 0 });
    setSelectedStation(stationIdx);
    setSelectedWp(0);
    const seamsFound = discoverSeams(prog);
    setSelectedSeamId(seamsFound[0]?.id ?? null);
    if (seamsFound[0]?.weaveNo) setWeaveNo(seamsFound[0].weaveNo);
    setLeftTab("welds");
    setStatus(
      t("status.programReady", {
        mm: prog.weldLenMm.toFixed(0),
        sec: prog.cycleSec.toFixed(1),
      }),
    );
  }

  function selectWaypoint(index: number) {
    if (!program) return;
    setSelectedWp(index);
    setManualJog(false);
    setSimPlaying(false);
    setSimT(wpProgress[index] ?? 0);
    const seamsFound = discoverSeams(program);
    const hit = seamsFound.find(
      (s) => index >= s.approachIdx && index <= s.retractIdx,
    );
    if (hit) {
      setSelectedSeamId(hit.id);
      setLeftTab("welds");
    }
  }

  function patchSeam(edit: SeamEdit, reverse = false) {
    if (!program) return;
    const next = applySeamEdit(program, edit, { reverse });
    setProgram(next);
    setSelectedSeamId(edit.id);
    if (edit.weaveNo !== weaveNo) setWeaveNo(edit.weaveNo || DEFAULT_WEAVE_NO);
    setStatus(
      t("status.seamUpdated", {
        seam: edit.label,
        spin: edit.spinOffsetDeg.toFixed(0),
      }),
    );
  }

  function removeSelectedPoint() {
    if (!program || selectedWp == null) return;
    if (program.waypoints.length <= 2) {
      setStatus(t("status.wpMinPoints"));
      return;
    }
    const idx = selectedWp;
    const next = deleteWaypoint(program, idx);
    setProgram(next);
    const newIdx = Math.min(idx, next.waypoints.length - 1);
    setSelectedWp(newIdx);
    setSimT(0);
    setStatus(t("status.wpDeleted", { c: informCLabel(idx) }));
  }

  function insertPointAfterSelected() {
    if (!program) return;
    const after = selectedWp ?? program.waypoints.length - 1;
    const next = insertIdleWaypoint(program, after, insertKind);
    setProgram(next);
    const newIdx = Math.min(after + 1, next.waypoints.length - 1);
    setSelectedWp(newIdx);
    setManualJog(false);
    setSimPlaying(false);
    // Seek after durations rebuild — approximate mid progress.
    const cum = [0];
    for (const d of next.segDurations) cum.push(cum[cum.length - 1] + d);
    const total = next.cycleSec || 1;
    setSimT((cum[newIdx] ?? 0) / total);
    setStatus(
      t("status.wpInserted", {
        kind: t(`program.kind.${insertKind}`),
        c: informCLabel(newIdx),
      }),
    );
  }

  function insertPointBeforeSelected() {
    if (!program || selectedWp == null) return;
    const after = selectedWp - 1;
    const next = insertIdleWaypoint(program, after, insertKind);
    setProgram(next);
    setSelectedWp(selectedWp);
    setStatus(
      t("status.wpInserted", {
        kind: t(`program.kind.${insertKind}`),
        c: informCLabel(selectedWp),
      }),
    );
  }

  function duplicateSelectedPoint() {
    if (!program || selectedWp == null) return;
    const next = duplicateWaypoint(program, selectedWp, insertKind);
    setProgram(next);
    setSelectedWp(selectedWp + 1);
    setStatus(t("status.wpDuplicated", { c: informCLabel(selectedWp + 1) }));
  }

  /** Live TCP from joint jog (robot base → world → program frame). */
  async function captureRobotProgramPos(): Promise<Vec3 | null> {
    if (!program) return null;
    setManualJog(true);
    setSimPlaying(false);
    const q = [joints.S, joints.L, joints.U, joints.R, joints.B, joints.T];
    try {
      const fk = await forwardKinematics(q);
      const tcp = fk.tcp_0 ?? fk.tcp;
      const world: Vec3 = [
        tcp.x,
        tcp.y + TSL_CARRIAGE_TOP_Y,
        tcp.z + railTravel,
      ];
      const stIdx = mountStationIndex(program.mount);
      const tableRot = stations[stIdx]?.rotate ?? 0;
      return worldToProgramFrame(world, program.mount, tableRot);
    } catch (err) {
      setStatus(t("status.fkFail", { err: (err as Error).message }));
      return null;
    }
  }

  async function teachReplaceSelected() {
    if (!program || selectedWp == null) return;
    const pos = await captureRobotProgramPos();
    if (!pos) return;
    const next = setWaypointPosition(program, selectedWp, pos);
    setProgram(next);
    setStatus(
      t("status.wpTaught", {
        c: informCLabel(selectedWp),
        xyz: pos.map((v) => (v * 1000).toFixed(1)).join(", "),
      }),
    );
  }

  async function teachInsertAfter() {
    if (!program) return;
    const pos = await captureRobotProgramPos();
    if (!pos) return;
    const after = selectedWp ?? program.waypoints.length - 1;
    const next = insertIdleWaypoint(program, after, insertKind, {
      pos,
      liftMm: 0,
    });
    setProgram(next);
    const newIdx = Math.min(after + 1, next.waypoints.length - 1);
    setSelectedWp(newIdx);
    setStatus(
      t("status.wpTaughtInsert", {
        kind: t(`program.kind.${insertKind}`),
        c: informCLabel(newIdx),
      }),
    );
  }

  function editSelectedXyzMm(axis: 0 | 1 | 2, mm: number) {
    if (!program || selectedWp == null) return;
    const cur = program.waypoints[selectedWp].pos;
    const pos: Vec3 = [...cur] as Vec3;
    pos[axis] = mm / 1000;
    setProgram(setWaypointPosition(program, selectedWp, pos));
  }

  function makeProgram(cond: number) {
    applyProgram(buildDemoProgram(DEMO_MOUNT, cond), DEMO_MOUNT[2], 0);
  }

  function loadDemo() {
    makeProgram(weldCondition);
  }

  function loadHardDemo() {
    applyProgram(buildHardDemoProgram(DEMO_HARD_MOUNT, weldCondition), DEMO_HARD_MOUNT[2], 1);
  }

  function identifyWelds() {
    if (!program) {
      makeProgram(weldCondition);
      setStatus(t("status.identifyDemo"));
    } else {
      setLeftTab("welds");
      setStatus(t("status.identifyExisting", { n: program.seam.length }));
    }
  }

  function planWelds() {
    if (!program) {
      setStatus(t("status.planNeedPart"));
      return;
    }
    const planned = planMotions(program);
    setProgram(planned);
    setLeftTab("program");
    setStatus(
      t("status.planDone", {
        collision: planned.meta?.collisionFree ? t("arcnc.yes") : t("arcnc.no"),
      }),
    );
  }

  function addTouchSense() {
    if (!program) {
      setStatus(t("status.planNeedPart"));
      return;
    }
    const next = ensureTouchSense(program);
    setProgram(next);
    setLeftTab("program");
    setStatus(t("status.touchSenseAdded"));
  }

  function optimizeWeldSequence() {
    if (!program) {
      setStatus(t("status.planNeedPart"));
      return;
    }
    const next = optimizeSequence(program);
    setProgram(next);
    setLeftTab("program");
    setStatus(t("status.sequenceOptimized"));
  }

  function changeCondition(c: number) {
    setWeldCondition(c);
    if (program) makeProgram(c);
  }

  async function downloadJbi() {
    if (!program) return;
    const stIdx = selectedStation ?? 0;
    const stationAngle = stations[stIdx]?.rotate ?? 0;
    let jointAnglesDeg: number[][] | undefined;
    try {
      setStatus(t("status.exportIk"));
      const ik = await calculateIk(waypointsToIkPoints(program));
      if (ik?.all_reachable && Array.isArray(ik.joint_angles_deg)) {
        jointAnglesDeg = ik.joint_angles_deg as number[][];
      }
    } catch {
      // Backend offline — fall back to local RECTAN review export.
    }
    const text = generateJbi(program, {
      stationDeg: stationAngle,
      jointAnglesDeg,
      weaveNo,
    });
    const blob = new Blob([text], { type: "text/plain" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${program.name}.JBI`;
    a.click();
    URL.revokeObjectURL(url);
    setStatus(
      jointAnglesDeg
        ? t("status.exportPulse", { n: jointAnglesDeg.length })
        : t("status.exportRectan"),
    );
  }

  function togglePlay() {
    if (!program) return;
    setManualJog(false);
    if (simT >= 1) setSimT(0);
    setSimPlaying((p) => !p);
  }

  function onJointsChange(next: Joints) {
    setJoints(next);
    if (!manualJog) {
      setSimPlaying(false);
      setManualJog(true);
    }
  }

  function homeJoints() {
    setSimPlaying(false);
    setManualJog(true);
    setJoints(cellHomeJoints());
    setStatus(t("status.home"));
  }

  function zeroJoints() {
    setSimPlaying(false);
    setManualJog(true);
    setJoints(defaultJoints());
    setStatus(t("status.zero"));
  }

  function stepTo(dir: 1 | -1) {
    if (!program) return;
    setSimPlaying(false);
    setManualJog(false);
    const eps = 1e-4;
    if (dir === 1) {
      const idx = wpProgress.findIndex((p) => p > simT + eps);
      const nextT = idx >= 0 ? wpProgress[idx] : 1;
      setSimT(nextT);
      setSelectedWp(idx >= 0 ? idx : program.waypoints.length - 1);
    } else {
      let idx = -1;
      for (let i = wpProgress.length - 1; i >= 0; i--) {
        if (wpProgress[i] < simT - eps) {
          idx = i;
          break;
        }
      }
      setSimT(idx >= 0 ? wpProgress[idx] : 0);
      setSelectedWp(Math.max(0, idx));
    }
  }

  async function onFile(e: React.ChangeEvent<HTMLInputElement>) {
    const file = e.target.files?.[0];
    if (!file) return;
    setStatus(t("status.importing"));
    try {
      const result = await analyzeCad(file);
      setSeams(result.segments ?? []);
      setLeftTab("workspace");
      setStatus(t("status.imported", { name: file.name, n: result.seam_count ?? 0 }));
    } catch (err) {
      makeProgram(weldCondition);
      setStatus(
        t("status.cadFallback", { err: (err as Error).message }),
      );
    }
  }

  const activeModel = ROBOT_MODELS.find((m) => m.id === modelId);
  const activePositioner = POSITIONER_MODELS.find((p) => p.id === positionerId);
  const sample = program ? sampleProgram(program, simT) : null;
  const curWp = program && sample ? program.waypoints[sample.wpIndex] : null;
  const curC = sample ? informCLabel(sample.wpIndex) : "—";
  const instruction = curWp
    ? `${curWp.move} ${curC}  ${curWp.speedLabel}${curWp.tag ? "  " + curWp.tag : ""}${
        curWp.bc != null ? `  BC=${curWp.bc}` : ""
      }`
    : "—";

  const seamList = useMemo(
    () => (program ? discoverSeams(program) : []),
    [program],
  );
  const activeSeam =
    seamList.find((s) => s.id === selectedSeamId) ?? seamList[0] ?? null;
  const wpDiags = useMemo(
    () => (program ? diagnoseWaypoints(program) : []),
    [program],
  );
  const timelinePhases = useMemo(
    () => (program ? buildTimelinePhases(program) : []),
    [program],
  );
  const timelineMarkers = useMemo(
    () => (program ? buildTimelineMarkers(program, wpDiags) : []),
    [program, wpDiags],
  );
  const prevCLabel =
    sample && sample.wpIndex > 0
      ? informCLabel(sample.wpIndex - 1)
      : informCLabel(0);
  const nextCLabel =
    program && sample
      ? informCLabel(Math.min(program.waypoints.length - 1, sample.wpIndex + 1))
      : informCLabel(0);

  const jbiPreview = useMemo(() => {
    if (!program) return "";
    const stIdx = selectedStation ?? 0;
    return generateJbi(program, {
      stationDeg: stations[stIdx]?.rotate ?? 0,
      weaveNo,
    });
  }, [program, selectedStation, stations, weaveNo]);

  const activeSchedule =
    LORCH_S8_SCHEDULES.find((s) => s.condition === weldCondition) ?? LORCH_S8_SCHEDULES[0];

  const robotStatus = manualJog
    ? { ok: true, label: t("robot.statusManual") }
    : program
    ? { ok: true, label: t("robot.statusPath") }
    : { ok: true, label: t("robot.statusIdle") };

  const leftTabBtn = (id: LeftTab, label: string) => (
    <button
      key={id}
      onClick={() => setLeftTab(id)}
      className={`flex-1 border-b-2 px-2 py-1.5 text-xs font-semibold transition ${
        leftTab === id
          ? "border-[#e87722] text-[#e87722] bg-orange-50"
          : "border-transparent text-slate-500 hover:text-slate-700"
      }`}
    >
      {label}
    </button>
  );

  const ribbonTab = (id: Ribbon, label: string) => (
    <button
      key={id}
      onClick={() => setRibbon(id)}
      className={`px-4 py-1 text-xs font-semibold uppercase tracking-wide transition ${
        ribbon === id
          ? "border-b-2 border-[#e87722] text-[#e87722]"
          : "text-slate-500 hover:text-slate-700"
      }`}
    >
      {label}
    </button>
  );

  const mobileNavBtn = (id: MobileTab, label: string, icon: string) => (
    <button
      key={id}
      type="button"
      onClick={() => setMobileTab(id)}
      className={`flex min-h-[3.25rem] flex-1 flex-col items-center justify-center gap-0.5 px-1 text-[10px] font-semibold transition ${
        mobileTab === id ? "text-[#e87722]" : "text-slate-500"
      }`}
    >
      <span className="text-base leading-none">{icon}</span>
      <span className="max-w-[4.5rem] truncate">{label}</span>
    </button>
  );

  return (
    <main className="flex h-[100dvh] flex-col overflow-hidden bg-[#f4f6f8] text-slate-800">
      {/* Title bar */}
      <header className="flex shrink-0 items-center justify-between gap-2 border-b border-slate-200 bg-white px-2 py-1.5 sm:px-3">
        <div className="flex min-w-0 items-center gap-2 sm:gap-3">
          <span className="flex h-6 w-6 shrink-0 items-center justify-center rounded bg-[#e87722] text-[11px] font-bold text-white">
            VW
          </span>
          <div className="min-w-0">
            <div className="truncate text-sm font-semibold leading-tight">
              {t("app.title")}
              <span className="ml-2 hidden text-[10px] font-normal text-slate-500 sm:inline">
                {t("app.styleHint")}
              </span>
            </div>
            <div className="hidden text-[10px] text-slate-500 sm:block">{t("app.subtitle")}</div>
          </div>
        </div>
        <div className="flex shrink-0 items-center gap-1.5 sm:gap-2">
          <LanguageToggle />
          <button onClick={loadDemo} className={`${btnGhost} hidden !px-2 !py-1 text-xs sm:inline-flex`}>
            {t("header.newProject")}
          </button>
          <button
            onClick={downloadJbi}
            disabled={!program}
            className={`${btnPrimary} !px-2 !py-1 text-xs sm:!px-3 sm:!py-1.5 sm:text-sm`}
          >
            {t("header.generateCode")}
          </button>
        </div>
      </header>

      {/* Ribbon */}
      <div className="shrink-0 border-b border-slate-200 bg-white">
        <div className="flex gap-1 px-2 pt-1">
          {ribbonTab("plan", t("ribbon.plan"))}
          {ribbonTab("settings", t("ribbon.settings"))}
          {ribbonTab("view", t("ribbon.view"))}
        </div>
        <div className="flex items-center gap-1 overflow-x-auto whitespace-nowrap no-scrollbar px-2 py-1.5 sm:py-2">
          {ribbon === "plan" && (
            <>
              <label className={btnRibbon}>
                <span className="text-base">📂</span>{t("plan.importPart")}<input
                  type="file"
                  accept=".step,.stp,.stl,.obj,.ply,.glb"
                  onChange={onFile}
                  className="hidden"
                />
              </label>
              <button onClick={loadDemo} className={btnRibbon}>
                <span className="text-base">🧱</span>{t("plan.loadDemo")}</button>
              <button onClick={loadHardDemo} className={btnRibbon}>
                <span className="text-base">🧱</span>{t("plan.loadHardDemo")}</button>
              <div className="mx-1 hidden h-8 w-px shrink-0 bg-slate-200 sm:block" />
              <button onClick={identifyWelds} className={btnRibbon}>
                <span className="text-base">🔍</span>{t("plan.identifyWelds")}</button>
              <button
                onClick={() => {
                  setRibbon("settings");
                  setLeftTab("welds");
                  setMobileTab("cad");
                }}
                className={btnRibbon}
              >
                <span className="text-base">⚙️</span>{t("plan.weldSettings")}</button>
              <button onClick={planWelds} className={btnRibbon} disabled={!program}>
                <span className="text-base">🛤</span>
                {t("arcnc.planMotion")}
              </button>
              <button onClick={addTouchSense} className={btnRibbon} disabled={!program}>
                <span className="text-base">👆</span>
                {t("arcnc.addSense")}
              </button>
              <button onClick={optimizeWeldSequence} className={btnRibbon} disabled={!program}>
                <span className="text-base">🔀</span>
                {t("arcnc.optimizeSeq")}
              </button>
              <div className="mx-1 hidden h-8 w-px shrink-0 bg-slate-200 sm:block" />
              <button
                onClick={() => {
                  setLeftTab("program");
                  setMobileTab("scene");
                  togglePlay();
                }}
                className={btnRibbon}
                disabled={!program}
              >
                <span className="text-base">▶</span>{t("plan.simulate")}</button>
              <button
                onClick={() => {
                  setMobileTab("jbi");
                  void downloadJbi();
                }}
                className={btnRibbon}
                disabled={!program}
              >
                <span className="text-base">💾</span>
                {t("plan.generateCode")}
              </button>
            </>
          )}
          {ribbon === "settings" && (
            <>
              <div className="flex shrink-0 items-center gap-2 px-2 text-xs text-slate-700">
                <span>{t("settings.weldCondition")}</span>
                <select
                  value={weldCondition}
                  onChange={(e) => changeCondition(Number(e.target.value))}
                  className="rounded bg-slate-100 px-2 py-1"
                >
                  {LORCH_S8_SCHEDULES.map((s) => (
                    <option key={s.condition} value={s.condition}>
                      ARGF{s.job} — {s.currentA}A / {s.voltageV.toFixed(1)}V / {s.travelCmMin}{" "}
                      cm/min
                    </option>
                  ))}
                </select>
              </div>
              <div className="flex shrink-0 items-center gap-2 px-2 text-xs text-slate-700">
                <span>WEV#</span>
                <select
                  value={weaveNo}
                  onChange={(e) => setWeaveNo(Number(e.target.value))}
                  className="rounded bg-slate-100 px-2 py-1"
                >
                  {WEAVE_FILES.map((w) => (
                    <option key={w.wevNo} value={w.wevNo}>
                      {w.label}
                    </option>
                  ))}
                </select>
              </div>
              <div className="flex shrink-0 items-center gap-2 px-2 text-xs text-slate-700">
                <span>{t("settings.robot")}</span>
                <select
                  value={modelId}
                  onChange={(e) => setModelId(e.target.value)}
                  className="rounded bg-slate-100 px-2 py-1"
                >
                  {ROBOT_MODELS.map((m) => (
                    <option key={m.id} value={m.id}>
                      {m.label}
                    </option>
                  ))}
                </select>
              </div>
              <button
                type="button"
                className={btnRibbon}
                onClick={() => setMobileTab("robot")}
                title={t("torch.panelTitle")}
              >
                <span className="text-base">🔧</span>
                {t("torch.panelTitle")}
              </button>
              <div className="flex shrink-0 items-center gap-2 px-2 text-xs text-slate-700">
                <span>{t("settings.positioner")}</span>
                <select
                  value={positionerId ?? ""}
                  onChange={(e) => setPositionerId(e.target.value || null)}
                  className="rounded bg-slate-100 px-2 py-1"
                >
                  <option value="">{t("settings.none")}</option>
                  {POSITIONER_MODELS.map((p) => (
                    <option key={p.id} value={p.id}>
                      {p.label}
                    </option>
                  ))}
                </select>
              </div>
            </>
          )}
          {ribbon === "view" && (
            <div className="px-2 text-xs text-slate-500">{t("view.help")}</div>
          )}
        </div>
      </div>

      <div className="hidden shrink-0 bg-amber-50 px-3 py-0.5 text-[11px] text-amber-800 sm:block">
        {t("app.warning")}
      </div>

      {/* ArcNC-style workflow strip — horizontal scroll on phones */}
      <div className="flex shrink-0 items-center gap-1 overflow-x-auto whitespace-nowrap no-scrollbar border-b border-slate-200 bg-white px-2 py-1.5 text-[11px] sm:px-3">
        <span className="mr-1 shrink-0 font-semibold uppercase tracking-wide text-slate-500 sm:mr-2">
          {t("arcnc.workflow")}
        </span>
        {[
          ["arcnc.cad", !!program],
          ["arcnc.detect", !!program],
          ["arcnc.plan", !!program?.meta?.collisionFree],
          ["arcnc.sense", !!program?.meta?.touchSense],
          ["arcnc.sim", simT > 0 || simPlaying],
          ["arcnc.export", !!program],
          ["arcnc.jbi", !!program],
        ].map(([key, done], i) => (
          <span
            key={String(key)}
            className={`inline-flex shrink-0 rounded-full border px-2 py-0.5 font-medium sm:px-2.5 ${
              done
                ? "border-emerald-300 bg-emerald-50 text-emerald-800"
                : "border-slate-200 bg-slate-50 text-slate-500"
            }`}
          >
            {done ? "✓ " : `${i + 1}. `}
            {t(String(key)).replace(/^\d+\.\s*/, "")}
          </span>
        ))}
      </div>

      {/* Main layout: stacked mobile panels / 3-column desktop */}
      <div className="flex min-h-0 flex-1">
        {/* LEFT DOCK — CAD / Welds (mobile tab + always on lg) */}
        <aside
          className={`${
            mobileTab === "cad" ? "flex" : "hidden"
          } w-full min-w-0 flex-col border-r border-slate-200 bg-white lg:flex lg:w-72 lg:shrink-0`}
        >
          <div className="flex border-b border-slate-200">
            {leftTabBtn("workspace", t("left.workspace"))}
            {leftTabBtn("welds", t("left.welds"))}
            {leftTabBtn("program", t("left.program"))}
          </div>

          <div className="flex-1 overflow-y-auto p-2 text-sm">
            {leftTab === "workspace" && (
              <div className="space-y-1">
                <div className="px-1 text-[10px] font-semibold uppercase text-slate-500">{t("workspace.cell")}</div>
                <button
                  onClick={() => setSelectedStation(null)}
                  className="flex w-full items-center gap-2 rounded px-2 py-1 text-left hover:bg-slate-100"
                >
                  <span className="text-[#e87722]">●</span> {t("workspace.robot")} ·{" "}
                  {activeModel?.label ?? modelId}
                </button>
                <div className="flex items-center gap-2 px-2 py-1 text-slate-700">
                  <span className="text-slate-500">●</span>
                  <span className="shrink-0">{t("mobile.railBs1")}</span>
                  <input
                    type="range"
                    min={-1.6}
                    max={1.6}
                    step={0.01}
                    value={railTravel}
                    onChange={(e) => setRailTravel(Number(e.target.value))}
                    className="touch-range touch-pan-y ml-auto min-w-0 flex-1 accent-[#e87722]"
                  />
                  <span className="w-12 text-right font-mono text-[10px]">
                    {railTravel.toFixed(2)}m
                  </span>
                </div>
                {[0, 1].map((i) => (
                  <button
                    key={i}
                    onClick={() => setSelectedStation(i as 0 | 1)}
                    className={`flex w-full items-center gap-2 rounded px-2 py-2 text-left hover:bg-slate-100 ${
                      selectedStation === i ? "bg-slate-100 text-[#e87722]" : ""
                    }`}
                  >
                    <span className="text-slate-500">●</span>{" "}
                    {t("mobile.stStation", { n: i + 1 })} —{" "}
                    {activePositioner?.label ?? t("workspace.positioner")}
                  </button>
                ))}
                <div className="mt-3 px-1 text-[10px] font-semibold uppercase text-slate-500">{t("workspace.parts")}</div>
                <div className="rounded px-2 py-1 text-slate-700">
                  {program ? (
                    <>
                      <div className="font-medium text-slate-800">{program.name}</div>
                      <div className="text-[11px] text-slate-500">{t("workspace.mountedOn")}</div>
                    </>
                  ) : (
                    <span className="text-slate-500">{t("workspace.noPart")}</span>
                  )}
                </div>
                {seams.length > 0 && (
                  <div className="text-[11px] text-slate-500">
                    {t("workspace.extraSeams", { n: seams.length })}
                  </div>
                )}
              </div>
            )}

            {leftTab === "welds" && (
              <div className="space-y-2">
                {program ? (
                  <>
                    <div className="space-y-1">
                      {seamList.map((s) => (
                        <button
                          key={s.id}
                          type="button"
                          onClick={() => {
                            setSelectedSeamId(s.id);
                            selectWaypoint(s.weldStart);
                          }}
                          className={`flex w-full items-center justify-between rounded border px-2 py-1.5 text-left text-xs ${
                            activeSeam?.id === s.id
                              ? "border-[#e87722] bg-orange-50 text-[#e87722]"
                              : "border-slate-200 bg-white text-slate-700 hover:bg-slate-50"
                          }`}
                        >
                          <span className="font-medium">{s.label}</span>
                          <span className="rounded bg-orange-50 px-1.5 py-0.5 text-[10px] text-orange-700">
                            {t("welds.fillet")}
                          </span>
                        </button>
                      ))}
                    </div>
                    <div className="text-[11px] text-slate-500">
                      {t("welds.length", {
                        mm: program.weldLenMm.toFixed(0),
                        n: program.waypoints.length,
                      })}
                    </div>
                    <SeamInspector
                      seam={activeSeam}
                      onChange={(next) => patchSeam(next)}
                      onReverse={() => activeSeam && patchSeam(activeSeam, true)}
                      labels={{
                        title: t("inspector.title"),
                        reverse: t("inspector.reverse"),
                        spin: t("inspector.spin"),
                        travel: t("inspector.travel"),
                        approach: t("inspector.approach"),
                        weave: t("inspector.weave"),
                        weaveNone: t("inspector.weaveNone"),
                        speed: t("inspector.speed"),
                        empty: t("inspector.empty"),
                      }}
                    />
                    <label className="block text-[10px] text-slate-500">
                      {t("welds.process")}
                      <select
                        value={weldCondition}
                        onChange={(e) => changeCondition(Number(e.target.value))}
                        className="mt-0.5 w-full rounded bg-slate-100 px-2 py-2 text-xs"
                      >
                        {LORCH_S8_SCHEDULES.map((s) => (
                          <option key={s.condition} value={s.condition}>
                            Motoweld ARGF{s.job} · {s.currentA}A / {s.voltageV.toFixed(1)}V · V=
                            {s.travelCmMin}
                          </option>
                        ))}
                      </select>
                    </label>
                    <div className="grid grid-cols-2 gap-2">
                      <label className="block text-[10px] text-slate-500">
                        {t("mobile.argf")}
                        <div className="mt-0.5 rounded border border-slate-200 bg-white px-2 py-2 font-mono text-xs text-slate-800">
                          ARGF{activeSchedule.job}
                        </div>
                      </label>
                      <label className="block text-[10px] text-slate-500">
                        {t("mobile.wev")}
                        <input
                          type="number"
                          min={0}
                          max={99}
                          value={weaveNo}
                          onChange={(e) => {
                            const n = Math.max(0, Number(e.target.value) || 0);
                            setWeaveNo(n);
                            if (activeSeam) patchSeam({ ...activeSeam, weaveNo: n });
                          }}
                          className="mt-0.5 w-full rounded border border-slate-200 bg-white px-2 py-2 font-mono text-xs text-slate-800"
                        />
                      </label>
                    </div>
                  </>
                ) : (
                  <p className="px-1 text-xs text-slate-500">{t("welds.empty")}</p>
                )}
              </div>
            )}

            {leftTab === "program" && (
              <div className="flex h-full min-h-0 flex-col gap-1">
                {program ? (
                  <>
                    <div className="shrink-0 space-y-1.5 rounded border border-slate-200 bg-slate-50 p-1.5">
                      <div className="text-[10px] font-semibold uppercase text-slate-500">
                        {t("program.editTitle")}
                      </div>
                      <label className="block text-[10px] text-slate-500">
                        {t("program.insertKind")}
                        <select
                          value={insertKind}
                          onChange={(e) =>
                            setInsertKind(e.target.value as IdleInsertKind)
                          }
                          className="mt-0.5 w-full rounded bg-white px-1.5 py-1 text-xs"
                        >
                          <option value="clearance">{t("program.kind.clearance")}</option>
                          <option value="travel">{t("program.kind.travel")}</option>
                          <option value="approach">{t("program.kind.approach")}</option>
                          <option value="retract">{t("program.kind.retract")}</option>
                          <option value="index_safe">{t("program.kind.index_safe")}</option>
                        </select>
                      </label>
                      <div className="grid grid-cols-2 gap-1">
                        <button
                          type="button"
                          disabled={selectedWp == null}
                          onClick={insertPointBeforeSelected}
                          className={`${btnGhost} !px-1.5 !py-1 text-[10px]`}
                          title={t("program.insertBefore")}
                        >
                          + {t("program.before")}
                        </button>
                        <button
                          type="button"
                          onClick={insertPointAfterSelected}
                          className={`${btnGhost} !px-1.5 !py-1 text-[10px]`}
                          title={t("program.insertAfter")}
                        >
                          + {t("program.after")}
                        </button>
                        <button
                          type="button"
                          disabled={selectedWp == null}
                          onClick={duplicateSelectedPoint}
                          className={`${btnGhost} !px-1.5 !py-1 text-[10px]`}
                        >
                          {t("program.duplicate")}
                        </button>
                        <button
                          type="button"
                          disabled={selectedWp == null || program.waypoints.length <= 2}
                          onClick={removeSelectedPoint}
                          className="rounded border border-red-200 bg-white px-1.5 py-1 text-[10px] text-red-700 transition hover:bg-red-50 disabled:cursor-not-allowed disabled:opacity-40"
                        >
                          {t("program.delete")}
                        </button>
                      </div>
                      {selectedWp != null && program.waypoints[selectedWp] && (
                        <>
                          <label className="block text-[10px] text-slate-500">
                            {t("program.changeKind")} ({informCLabel(selectedWp)})
                            <select
                              value={program.waypoints[selectedWp].kind}
                              onChange={(e) => {
                                const next = setWaypointKind(
                                  program,
                                  selectedWp,
                                  e.target.value as Waypoint["kind"],
                                );
                                setProgram(next);
                                setStatus(
                                  t("status.wpKindChanged", {
                                    c: informCLabel(selectedWp),
                                    kind: t(`program.kind.${e.target.value}`),
                                  }),
                                );
                              }}
                              className="mt-0.5 w-full rounded bg-white px-1.5 py-1 text-xs"
                            >
                              {(
                                [
                                  "approach",
                                  "sense",
                                  "weld",
                                  "travel",
                                  "clearance",
                                  "retract",
                                  "index_safe",
                                ] as const
                              ).map((k) => (
                                <option key={k} value={k}>
                                  {t(`program.kind.${k}`)}
                                </option>
                              ))}
                            </select>
                          </label>
                          <div className="text-[10px] text-slate-500">
                            {t("program.xyzMm")} ({informCLabel(selectedWp)})
                          </div>
                          <div className="grid grid-cols-3 gap-1">
                            {(["X", "Y", "Z"] as const).map((axis, ai) => (
                              <label
                                key={axis}
                                className="block text-[9px] text-slate-500"
                              >
                                {axis}
                                <input
                                  type="number"
                                  step={1}
                                  value={Number(
                                    (
                                      program.waypoints[selectedWp].pos[ai] * 1000
                                    ).toFixed(1),
                                  )}
                                  onChange={(e) =>
                                    editSelectedXyzMm(
                                      ai as 0 | 1 | 2,
                                      Number(e.target.value) || 0,
                                    )
                                  }
                                  className="mt-0.5 w-full rounded border border-slate-200 bg-white px-1 py-1 font-mono text-[10px]"
                                />
                              </label>
                            ))}
                          </div>
                        </>
                      )}
                      <div className="grid grid-cols-1 gap-1">
                        <button
                          type="button"
                          disabled={selectedWp == null}
                          onClick={() => void teachReplaceSelected()}
                          className={`${btnPrimary} !px-1.5 !py-1.5 text-[10px]`}
                          title={t("program.teachReplaceHint")}
                        >
                          {t("program.teachReplace")}
                        </button>
                        <button
                          type="button"
                          onClick={() => void teachInsertAfter()}
                          className={`${btnGhost} !px-1.5 !py-1.5 text-[10px]`}
                          title={t("program.teachInsertHint")}
                        >
                          {t("program.teachInsert")}
                        </button>
                      </div>
                      <p className="text-[9px] leading-snug text-slate-400">
                        {t("program.teachHelp")}
                      </p>
                    </div>
                    <div className="min-h-0 flex-1 overflow-y-auto font-mono text-[11px]">
                      {program.waypoints.map((wp, i) => {
                        const diag = wpDiags[i];
                        return (
                          <div
                            key={`${wp.id}-${i}`}
                            className={`flex w-full items-center gap-1 rounded px-1 py-0.5 ${
                              selectedWp === i || (sample && i === sample.wpIndex)
                                ? "bg-[#fff1e6] text-[#e87722]"
                                : "text-slate-700 hover:bg-slate-100"
                            }`}
                          >
                            <button
                              type="button"
                              onClick={() => selectWaypoint(i)}
                              className="flex min-w-0 flex-1 items-center gap-2 text-left"
                            >
                              <span className="w-12 shrink-0 text-slate-500">
                                {informCLabel(i)}
                              </span>
                              <span className="w-10 shrink-0">{wp.move}</span>
                              <span className="w-14 shrink-0 truncate text-[10px] uppercase text-slate-400">
                                {t(`program.kind.${wp.kind}`)}
                              </span>
                              <span className="min-w-0 flex-1 truncate">{wp.speedLabel}</span>
                              <span
                                className={
                                  diag?.ik === "collision"
                                    ? "text-red-600"
                                    : diag?.ik === "near_limit"
                                      ? "text-amber-600"
                                      : wp.tag === "TOUCH"
                                        ? "text-blue-600"
                                        : "text-orange-600"
                                }
                              >
                                {diag?.ik !== "ok"
                                  ? "!"
                                  : wp.tag === "TOUCH"
                                    ? t("welds.tagTouch")
                                    : wp.tag ?? ""}
                              </span>
                            </button>
                            <button
                              type="button"
                              title={t("program.delete")}
                              disabled={program.waypoints.length <= 2}
                              onClick={(e) => {
                                e.stopPropagation();
                                setSelectedWp(i);
                                const next = deleteWaypoint(program, i);
                                setProgram(next);
                                setSelectedWp(Math.min(i, next.waypoints.length - 1));
                                setStatus(t("status.wpDeleted", { c: informCLabel(i) }));
                              }}
                              className="shrink-0 rounded px-1.5 py-0.5 text-[10px] text-red-600 hover:bg-red-50 disabled:opacity-30"
                            >
                              ×
                            </button>
                          </div>
                        );
                      })}
                    </div>
                  </>
                ) : (
                  <p className="px-1 text-xs text-slate-500">{t("program.empty")}</p>
                )}
              </div>
            )}
          </div>

          {/* Details */}
          <div className="border-t border-slate-200 bg-[#f7f8fa] p-2">
            <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">{t("details.title")}</div>
            {selectedStation != null && activePositioner ? (
              <div className="space-y-2 text-xs">
                <div className="font-medium">Stół {selectedStation + 1}</div>
                {activePositioner.hasTilt && (
                  <label className="flex items-center gap-2">
                    <span className="w-12 text-slate-500">{t("details.tilt")}</span>
                    <input
                      type="range"
                      min={-135}
                      max={135}
                      step={0.5}
                      value={stations[selectedStation].tilt}
                      onChange={(e) =>
                        setStation(selectedStation, { tilt: Number(e.target.value) })
                      }
                      className="touch-range touch-pan-y flex-1 accent-[#e87722]"
                    />
                    <span className="w-10 text-right font-mono">
                      {stations[selectedStation].tilt.toFixed(0)}°
                    </span>
                  </label>
                )}
                {activePositioner.hasRotate && (
                  <label className="flex items-center gap-2">
                    <span className="w-12 text-slate-500">{t("details.rotate")}</span>
                    <input
                      type="range"
                      min={-360}
                      max={360}
                      step={0.5}
                      value={stations[selectedStation].rotate}
                      onChange={(e) =>
                        setStation(selectedStation, { rotate: Number(e.target.value) })
                      }
                      className="touch-range touch-pan-y flex-1 accent-[#e87722]"
                    />
                    <span className="w-10 text-right font-mono">
                      {stations[selectedStation].rotate.toFixed(0)}°
                    </span>
                  </label>
                )}
              </div>
            ) : program ? (
              <dl className="grid grid-cols-2 gap-x-2 gap-y-0.5 text-[11px] text-slate-700">
                <dt className="text-slate-500">{t("details.part")}</dt>
                <dd className="font-mono">{program.name}</dd>
                <dt className="text-slate-500">{t("details.weldLen")}</dt>
                <dd className="font-mono">{program.weldLenMm.toFixed(0)} mm</dd>
                <dt className="text-slate-500">{t("details.cycle")}</dt>
                <dd className="font-mono">{program.cycleSec.toFixed(1)} s</dd>
                <dt className="text-slate-500">{t("details.mode")}</dt>
                <dd>{manualJog ? t("details.modeManual") : t("details.modePath")}</dd>
              </dl>
            ) : (
              <p className="text-[11px] text-slate-500">{t("details.empty")}</p>
            )}
            {status && <p className="mt-2 text-[10px] text-slate-500">{status}</p>}
          </div>
        </aside>

        {/* CENTER — Viewer + sim bar (mobile scene tab + always on lg) */}
        <div
          className={`${
            mobileTab === "scene" ? "flex" : "hidden"
          } relative min-w-0 flex-1 flex-col lg:flex`}
        >
          <div className="relative min-h-0 flex-1 touch-none">
            <RobotWorkspace
              modelId={modelId}
              joints={joints}
              seams={seams}
              positionerId={positionerId}
              railTravel={railTravel}
              stations={stations}
              program={program}
              simT={simT}
              manualJog={manualJog}
              torchCalib={torchCalib}
              selectedWp={selectedWp ?? sample?.wpIndex ?? null}
              onSelectWp={selectWaypoint}
              weaveNo={activeSeam?.weaveNo ?? weaveNo}
            />

            {program && sample && (
              <div className="pointer-events-none absolute left-2 top-2 rounded border border-slate-200 bg-white/90 px-2 py-1 font-mono text-[10px] text-slate-700 sm:left-3 sm:top-3 sm:px-2.5 sm:py-1.5 sm:text-[11px]">
                <div className="mb-0.5 font-sans text-[10px] font-semibold uppercase text-slate-500">
                  {program.name}
                </div>
                <div className="flex items-center gap-2">
                  <span
                    className={`inline-block h-2 w-2 rounded-full ${
                      sample.arcOn ? "animate-pulse bg-orange-400" : "bg-slate-200"
                    }`}
                  />
                  ARC {sample.arcOn ? "ON" : "OFF"}
                  <span className="text-slate-500">·</span>
                  {sample.elapsedSec.toFixed(1)}/{program.cycleSec.toFixed(1)}s
                </div>
              </div>
            )}

            {/* Floating mini player on phones — sits above bottom nav */}
            <div className="pointer-events-auto absolute inset-x-2 bottom-2 z-10 rounded-lg border border-slate-200 bg-white/95 p-2 shadow-sm backdrop-blur lg:hidden">
              <div className="mb-1 truncate px-0.5 font-mono text-[10px] text-slate-500">
                {instruction}
              </div>
              <SimTimeline
                phases={timelinePhases}
                markers={timelineMarkers}
                simT={simT}
                disabled={!program}
                onSeek={(t) => {
                  setSimPlaying(false);
                  setManualJog(false);
                  setSimT(t);
                }}
              />
              <div className="mt-1.5 flex items-center gap-1.5">
                <button
                  onClick={() => stepTo(-1)}
                  disabled={!program}
                  className={`${btnGhost} !min-h-10 !px-2 !py-1 text-[10px]`}
                  title={t("sim.prevC", { c: prevCLabel })}
                >
                  |&lt; {prevCLabel}
                </button>
                <button
                  onClick={togglePlay}
                  disabled={!program}
                  className={`${btnPrimary} !min-h-10 !px-3 !py-1 text-xs`}
                >
                  {simPlaying ? t("sim.pause") : simT >= 1 ? t("sim.replay") : t("sim.play")}
                </button>
                <button
                  onClick={() => stepTo(1)}
                  disabled={!program}
                  className={`${btnGhost} !min-h-10 !px-2 !py-1 text-[10px]`}
                  title={t("sim.nextC", { c: nextCLabel })}
                >
                  {nextCLabel} &gt;|
                </button>
              </div>
            </div>
          </div>

          {/* Desktop / tablet landscape sim bar */}
          <div className="hidden flex-col gap-1 border-t border-slate-200 bg-white px-3 py-1.5 lg:flex">
            <div className="flex items-center gap-2">
              <button
                onClick={() => stepTo(-1)}
                disabled={!program}
                className={`${btnGhost} !px-2 !py-1 text-[11px]`}
                title={t("sim.prevC", { c: prevCLabel })}
              >
                |&lt; {t("sim.prev")} {prevCLabel}
              </button>
              <button
                onClick={togglePlay}
                disabled={!program}
                className={`${btnPrimary} !px-4 !py-1`}
              >
                {simPlaying ? t("sim.pause") : simT >= 1 ? t("sim.replay") : t("sim.play")}
              </button>
              <button
                onClick={() => stepTo(1)}
                disabled={!program}
                className={`${btnGhost} !px-2 !py-1 text-[11px]`}
                title={t("sim.nextC", { c: nextCLabel })}
              >
                {t("sim.next")} {nextCLabel} &gt;|
              </button>
              <button
                onClick={() => {
                  setSimPlaying(false);
                  setSimT(0);
                  setSelectedWp(0);
                }}
                disabled={!program}
                className={`${btnGhost} !px-2 !py-1`}
                title={t("sim.reset")}
              >
                ⟲
              </button>
              <div className="min-w-0 flex-1 truncate px-2 font-mono text-[11px] text-slate-500">
                {instruction}
              </div>
              <select
                value={simSpeed}
                onChange={(e) => setSimSpeed(Number(e.target.value))}
                className="rounded bg-slate-100 px-2 py-1 text-xs"
              >
                <option value={0.5}>0.5×</option>
                <option value={1}>1×</option>
                <option value={2}>2×</option>
                <option value={4}>4×</option>
              </select>
              <input
                type="range"
                min={0}
                max={1}
                step={0.001}
                value={simT}
                disabled={!program}
                onChange={(e) => {
                  setSimPlaying(false);
                  setManualJog(false);
                  setSimT(Number(e.target.value));
                }}
                className="touch-range touch-pan-y w-40 accent-[#e87722]"
              />
            </div>
            <SimTimeline
              phases={timelinePhases}
              markers={timelineMarkers}
              simT={simT}
              disabled={!program}
              onSeek={(t) => {
                setSimPlaying(false);
                setManualJog(false);
                setSimT(t);
              }}
            />
          </div>
        </div>

        {/* RIGHT DOCK — Robot (mobile tab + always on lg) */}
        <aside
          className={`${
            mobileTab === "robot" ? "flex" : "hidden"
          } w-full min-w-0 flex-col border-l border-slate-200 bg-white lg:flex lg:w-80 lg:shrink-0`}
        >
          <div className="border-b border-slate-200 px-3 py-2">
            <div className="text-xs font-semibold uppercase tracking-wide text-slate-700">{t("robot.title")}</div>
            <div className="text-[10px] text-slate-500">
              {activeModel?.label ?? "AR2010"} · {t("robot.jointJog")}
            </div>
          </div>

          <div className="flex-1 overflow-y-auto p-3">
            <div
              className={`mb-3 rounded px-2 py-1.5 text-center text-xs font-semibold ${
                manualJog
                  ? "bg-[#fff1e6] text-[#e87722]"
                  : "bg-slate-100 text-slate-500"
              }`}
            >
              {manualJog ? t("robot.modeManual") : t("robot.modePath")}
            </div>

            <div className="mb-2 flex gap-2">
              <button onClick={homeJoints} className={`${btnGhost} flex-1 !py-1 text-xs`}>{t("robot.home")}</button>
              <button onClick={zeroJoints} className={`${btnGhost} flex-1 !py-1 text-xs`}>{t("robot.zero")}</button>
              {!manualJog && (
                <button
                  onClick={() => {
                    setSimPlaying(false);
                    setManualJog(true);
                    setStatus(t("status.jogOn"));
                  }}
                  className={`${btnPrimary} flex-1 !py-1 text-xs`}
                >{t("robot.takeJog")}</button>
              )}
            </div>

            <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">{t("robot.joints")}</div>
            <AxisSliders joints={joints} onChange={onJointsChange} />

            <TorchCalibrationPanel calib={torchCalib} onChange={onTorchCalibChange} />

            <div className="mb-1 mt-4 text-[10px] font-semibold uppercase text-slate-500">{t("robot.tool")}</div>
            <dl className="grid grid-cols-[1fr_auto] gap-x-2 gap-y-0.5 font-mono text-[11px] text-slate-700">
              <dt className="text-slate-500">{t("robot.frame")}</dt>
              <dd>{t("robot.frameWorld")}</dd>
              <dt className="text-slate-500">{t("robot.tcp")}</dt>
              <dd className="text-right">
                {manualJog && liveTcpMm
                  ? `${liveTcpMm} mm`
                  : sample
                    ? `${(sample.pos[0] * 1000).toFixed(1)}, ${(sample.pos[1] * 1000).toFixed(1)}, ${(sample.pos[2] * 1000).toFixed(1)} mm`
                    : "—"}
              </dd>
              <dt className="text-slate-500">{t("robot.rail")}</dt>
              <dd>{railTravel.toFixed(3)} m</dd>
            </dl>

            <div className="mb-1 mt-4 text-[10px] font-semibold uppercase text-slate-500">{t("robot.status")}</div>
            <div
              className={`rounded px-2 py-1.5 text-xs font-medium ${
                robotStatus.ok
                  ? "bg-emerald-50 text-emerald-700"
                  : "bg-red-50 text-red-700"
              }`}
            >
              {robotStatus.ok ? "●" : "⚠"} {robotStatus.label}
            </div>

            <div className="mb-1 mt-4 text-[10px] font-semibold uppercase text-slate-500">
              {t("exec.title")}
            </div>
            <p className="mb-1 text-[10px] leading-snug text-slate-500">{t("exec.summary")}</p>
            <Dx200MotorosChecklist
              title={t("exec.checklistTitle")}
              firmwareWarn={t("exec.firmwareWarn")}
              firmwareOkLabel={t("exec.firmwareOk")}
              firmwareBadLabel={t("exec.firmwareBad")}
              s2cTitle={t("exec.s2cTitle")}
              groupsTitle={t("exec.groupsTitle")}
              tfTitle={t("exec.tfTitle")}
              jbiNote={t("exec.jbiNote")}
              limitsTitle={t("exec.limitsTitle")}
            />

            {program?.meta && (
              <div className="mt-3 rounded border border-slate-200 bg-slate-50 p-2">
                <div className="mb-1.5 text-[10px] font-semibold uppercase text-slate-500">
                  {t("arcnc.planner")}
                </div>
                <ul className="space-y-1 text-[11px]">
                  {(
                    [
                      ["arcnc.collisionFree", program.meta.collisionFree],
                      ["arcnc.touchSense", program.meta.touchSense],
                      ["arcnc.sequence", program.meta.sequenceOptimized],
                      ["arcnc.singularity", program.meta.singularitySafe],
                    ] as const
                  ).map(([k, ok]) => (
                    <li key={k} className="flex items-center justify-between gap-2">
                      <span className="text-slate-600">{t(k)}</span>
                      <span className={ok ? "font-semibold text-emerald-700" : "font-semibold text-amber-700"}>
                        {ok ? t("arcnc.yes") : t("arcnc.no")}
                      </span>
                    </li>
                  ))}
                </ul>
              </div>
            )}

            <div className="mb-1 mt-4 text-[10px] font-semibold uppercase text-slate-500">
              {t("mobile.railBs1")} · ST1–ST4
            </div>
            <div className="mb-2 flex flex-wrap gap-1">
              {[1, 2, 3, 4].map((n) => (
                <button
                  key={n}
                  type="button"
                  onClick={() => {
                    if (n <= 2) setSelectedStation((n - 1) as 0 | 1);
                    setMobileTab("cad");
                    setLeftTab("workspace");
                  }}
                  className={`rounded border px-2.5 py-2 text-xs font-semibold ${
                    selectedStation === n - 1
                      ? "border-[#e87722] bg-orange-50 text-[#e87722]"
                      : "border-slate-200 text-slate-600"
                  }`}
                >
                  ST{n}
                </button>
              ))}
            </div>

            <p className="mt-2 text-[10px] leading-snug text-slate-500">
              {t("robot.help")}
            </p>
          </div>
        </aside>

        {/* JBI CODE — mobile-only panel */}
        <aside
          className={`${
            mobileTab === "jbi" ? "flex" : "hidden"
          } w-full min-w-0 flex-col bg-white lg:hidden`}
        >
          <div className="flex items-center justify-between gap-2 border-b border-slate-200 px-3 py-2">
            <div>
              <div className="text-xs font-semibold uppercase tracking-wide text-slate-700">
                {t("mobile.jbi")}
              </div>
              <div className="text-[10px] text-slate-500">
                INFORM · ARGF{activeSchedule.job} · WEV#({weaveNo})
              </div>
            </div>
            <button
              type="button"
              onClick={() => void downloadJbi()}
              disabled={!program}
              className={`${btnPrimary} !px-3 !py-2 text-xs`}
            >
              {t("mobile.jbiDownload")}
            </button>
          </div>
          <div className="min-h-0 flex-1 overflow-auto p-2">
            {program ? (
              <pre className="overflow-x-auto whitespace-pre rounded border border-slate-200 bg-[#0f172a] p-3 font-mono text-[11px] leading-relaxed text-emerald-300">
                {jbiPreview}
              </pre>
            ) : (
              <p className="px-1 text-xs text-slate-500">{t("mobile.jbiEmpty")}</p>
            )}
          </div>
        </aside>
      </div>

      {/* Bottom navigation — phones / tablets below lg */}
      <nav
        className="flex shrink-0 border-t border-slate-200 bg-white pb-[env(safe-area-inset-bottom)] lg:hidden"
        aria-label="Mobile"
      >
        {mobileNavBtn("scene", t("mobile.scene"), "◈")}
        {mobileNavBtn("cad", t("mobile.cad"), "◫")}
        {mobileNavBtn("robot", t("mobile.robot"), "⚙")}
        {mobileNavBtn("jbi", t("mobile.jbi"), "{ }")}
      </nav>
    </main>
  );
}
