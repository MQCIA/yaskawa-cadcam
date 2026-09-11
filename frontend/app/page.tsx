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

import { useState, useEffect, useRef, useMemo } from "react";
import dynamic from "next/dynamic";
import AxisSliders, { defaultJoints, type Joints } from "@/components/AxisSliders";
import { useI18n, LanguageToggle } from "@/lib/i18n";
import type { SeamSegment } from "@/components/RobotWorkspace";
import { analyzeCad } from "@/lib/api";
import { ROBOT_MODELS, POSITIONER_MODELS } from "@/lib/models";
import {
  buildDemoProgram,
  sampleProgram,
  generateJbi,
  DEMO_MOUNT,
  LORCH_S8_SCHEDULES,
  type WeldProgram,
} from "@/lib/weldProgram";

const RobotWorkspace = dynamic(() => import("@/components/RobotWorkspace"), {
  ssr: false,
});

const PLAYBACK_BASE_SEC = 12;

type LeftTab = "workspace" | "welds" | "program";
type Ribbon = "plan" | "settings" | "view";

const btnPrimary =
  "rounded bg-[#e87722] px-3 py-1.5 text-sm font-semibold text-white transition hover:brightness-110 disabled:cursor-not-allowed disabled:opacity-40";
const btnGhost =
  "rounded border border-slate-500 bg-slate-800 px-3 py-1.5 text-sm text-slate-100 transition hover:bg-slate-700 disabled:cursor-not-allowed disabled:opacity-40";
const btnRibbon =
  "flex cursor-pointer flex-col items-center gap-0.5 rounded px-3 py-1.5 text-[11px] text-slate-200 transition hover:bg-slate-700/80 disabled:opacity-40";

export default function Home() {
  const { t } = useI18n();
  const [modelId, setModelId] = useState("ar2010");
  const [joints, setJoints] = useState<Joints>(defaultJoints());
  const [seams, setSeams] = useState<SeamSegment[]>([]);
  const [status, setStatus] = useState("");

  const [positionerId, setPositionerId] = useState<string | null>("h1000d");
  const [railTravel, setRailTravel] = useState(0);
  const [stations, setStations] = useState([
    { tilt: 0, rotate: 0 },
    { tilt: 0, rotate: 0 },
  ]);

  const [program, setProgram] = useState<WeldProgram | null>(null);
  const [weldCondition, setWeldCondition] = useState(1);
  const [simT, setSimT] = useState(0);
  const [simPlaying, setSimPlaying] = useState(false);
  const [simSpeed, setSimSpeed] = useState(1);

  // Verbotics-style: joint jog is always available; ON until the user plays a plan.
  const [manualJog, setManualJog] = useState(true);

  const [ribbon, setRibbon] = useState<Ribbon>("plan");
  const [leftTab, setLeftTab] = useState<LeftTab>("workspace");
  const [selectedStation, setSelectedStation] = useState<0 | 1 | null>(null);

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

  const wpProgress = useMemo(() => {
    if (!program) return [] as number[];
    const cum = [0];
    for (const d of program.segDurations) cum.push(cum[cum.length - 1] + d);
    return cum.map((c) => c / program.cycleSec);
  }, [program]);

  function makeProgram(cond: number) {
    const prog = buildDemoProgram(DEMO_MOUNT, cond);
    setProgram(prog);
    setSimT(0);
    setSimPlaying(false);
    setModelId("ar2010");
    setRailTravel(Math.max(-1.6, Math.min(1.6, DEMO_MOUNT[2])));
    setStation(0, { rotate: 0 });
    setLeftTab("welds");
    setStatus(
      t("status.programReady", {
        mm: prog.weldLenMm.toFixed(0),
        sec: prog.cycleSec.toFixed(1),
      }),
    );
  }

  function loadDemo() {
    makeProgram(weldCondition);
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
    setLeftTab("program");
    setStatus(t("status.planDone"));
  }

  function changeCondition(c: number) {
    setWeldCondition(c);
    if (program) makeProgram(c);
  }

  function downloadJbi() {
    if (!program) return;
    const blob = new Blob([generateJbi(program)], { type: "text/plain" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${program.name}.JBI`;
    a.click();
    URL.revokeObjectURL(url);
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
    setJoints(defaultJoints());
    setStatus(t("status.home"));
  }

  function zeroJoints() {
    homeJoints();
    setStatus(t("status.zero"));
  }

  function stepTo(dir: 1 | -1) {
    if (!program) return;
    setSimPlaying(false);
    setManualJog(false);
    const eps = 1e-4;
    if (dir === 1) {
      const next = wpProgress.find((p) => p > simT + eps);
      setSimT(next ?? 1);
    } else {
      const prev = [...wpProgress].reverse().find((p) => p < simT - eps);
      setSimT(prev ?? 0);
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
  const instruction = curWp
    ? `${curWp.move} ${curWp.id}  ${curWp.speedLabel}${curWp.tag ? "  " + curWp.tag : ""}`
    : "—";

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
          ? "border-[#e87722] text-[#e87722]"
          : "border-transparent text-slate-400 hover:text-slate-200"
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
          ? "border-b-2 border-[#e87722] text-white"
          : "text-slate-400 hover:text-slate-200"
      }`}
    >
      {label}
    </button>
  );

  return (
    <main className="flex h-screen flex-col bg-[#1b1e24] text-slate-100">
      {/* Title bar */}
      <header className="flex items-center justify-between border-b border-slate-700 bg-[#12141a] px-3 py-1.5">
        <div className="flex items-center gap-3">
          <span className="flex h-6 w-6 items-center justify-center rounded bg-[#e87722] text-[11px] font-bold text-white">
            VW
          </span>
          <div>
            <div className="text-sm font-semibold leading-tight">
              {t("app.title")}
              <span className="ml-2 text-[10px] font-normal text-slate-500">
                {t("app.styleHint")}
              </span>
            </div>
            <div className="text-[10px] text-slate-500">{t("app.subtitle")}</div>
          </div>
        </div>
        <div className="flex items-center gap-2">
          <LanguageToggle />
          <button onClick={loadDemo} className={btnGhost}>{t("header.newProject")}</button>
          <button onClick={downloadJbi} disabled={!program} className={btnPrimary}>{t("header.generateCode")}</button>
        </div>
      </header>

      {/* Ribbon */}
      <div className="border-b border-slate-700 bg-[#1f232b]">
        <div className="flex gap-1 px-2 pt-1">
          {ribbonTab("plan", t("ribbon.plan"))}
          {ribbonTab("settings", t("ribbon.settings"))}
          {ribbonTab("view", t("ribbon.view"))}
        </div>
        <div className="flex flex-wrap items-center gap-1 px-2 py-2">
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
              <div className="mx-1 h-8 w-px bg-slate-600" />
              <button onClick={identifyWelds} className={btnRibbon}>
                <span className="text-base">🔍</span>{t("plan.identifyWelds")}</button>
              <button
                onClick={() => {
                  setRibbon("settings");
                  setLeftTab("welds");
                }}
                className={btnRibbon}
              >
                <span className="text-base">⚙️</span>{t("plan.weldSettings")}</button>
              <button onClick={planWelds} className={btnRibbon} disabled={!program}>
                <span className="text-base">🛤</span>
                {t("plan.plan")}
              </button>
              <div className="mx-1 h-8 w-px bg-slate-600" />
              <button
                onClick={() => {
                  setLeftTab("program");
                  togglePlay();
                }}
                className={btnRibbon}
                disabled={!program}
              >
                <span className="text-base">▶</span>{t("plan.simulate")}</button>
              <button onClick={downloadJbi} className={btnRibbon} disabled={!program}>
                <span className="text-base">💾</span>
                {t("plan.generateCode")}
              </button>
            </>
          )}
          {ribbon === "settings" && (
            <>
              <div className="flex items-center gap-2 px-2 text-xs text-slate-300">
                <span>{t("settings.weldCondition")}</span>
                <select
                  value={weldCondition}
                  onChange={(e) => changeCondition(Number(e.target.value))}
                  className="rounded bg-slate-800 px-2 py-1"
                >
                  {LORCH_S8_SCHEDULES.map((s) => (
                    <option key={s.condition} value={s.condition}>
                      #{s.condition} — {s.currentA}A / {s.voltageV.toFixed(1)}V / {s.travelCmMin}{" "}
                      cm/min
                    </option>
                  ))}
                </select>
              </div>
              <div className="flex items-center gap-2 px-2 text-xs text-slate-300">
                <span>{t("settings.robot")}</span>
                <select
                  value={modelId}
                  onChange={(e) => setModelId(e.target.value)}
                  className="rounded bg-slate-800 px-2 py-1"
                >
                  {ROBOT_MODELS.map((m) => (
                    <option key={m.id} value={m.id}>
                      {m.label}
                    </option>
                  ))}
                </select>
              </div>
              <div className="flex items-center gap-2 px-2 text-xs text-slate-300">
                <span>{t("settings.positioner")}</span>
                <select
                  value={positionerId ?? ""}
                  onChange={(e) => setPositionerId(e.target.value || null)}
                  className="rounded bg-slate-800 px-2 py-1"
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
            <div className="px-2 text-xs text-slate-400">
              LMB rotate · RMB pan · scroll zoom. Robot joint jog is always on the right
              (Verbotics-style).
            </div>
          )}
        </div>
      </div>

      <div className="bg-amber-900/30 px-3 py-0.5 text-[11px] text-amber-200">
        {t("app.warning")}</div>

      {/* Main 3-column layout */}
      <div className="flex min-h-0 flex-1">
        {/* LEFT DOCK */}
        <aside className="flex w-72 shrink-0 flex-col border-r border-slate-700 bg-[#181b21]">
          <div className="flex border-b border-slate-700">
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
                  className="flex w-full items-center gap-2 rounded px-2 py-1 text-left hover:bg-slate-800"
                >
                  <span className="text-[#e87722]">●</span> {t("workspace.robot")} ·{" "}
                  {activeModel?.label ?? modelId}
                </button>
                <div className="flex items-center gap-2 px-2 py-1 text-slate-300">
                  <span className="text-slate-500">●</span>{t("workspace.rail")}<input
                    type="range"
                    min={-1.6}
                    max={1.6}
                    step={0.01}
                    value={railTravel}
                    onChange={(e) => setRailTravel(Number(e.target.value))}
                    className="ml-auto w-24 accent-[#e87722]"
                  />
                  <span className="w-12 text-right font-mono text-[10px]">
                    {railTravel.toFixed(2)}m
                  </span>
                </div>
                {[0, 1].map((i) => (
                  <button
                    key={i}
                    onClick={() => setSelectedStation(i as 0 | 1)}
                    className={`flex w-full items-center gap-2 rounded px-2 py-1 text-left hover:bg-slate-800 ${
                      selectedStation === i ? "bg-slate-800 text-[#e87722]" : ""
                    }`}
                  >
                    <span className="text-slate-500">●</span> {t("workspace.table", { n: i + 1 })} —{" "}
                    {activePositioner?.label ?? t("workspace.positioner")}
                  </button>
                ))}
                <div className="mt-3 px-1 text-[10px] font-semibold uppercase text-slate-500">{t("workspace.parts")}</div>
                <div className="rounded px-2 py-1 text-slate-300">
                  {program ? (
                    <>
                      <div className="font-medium text-slate-100">{program.name}</div>
                      <div className="text-[11px] text-slate-500">
                        Mounted on Stół 1 · T-fillet coupon
                      </div>
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
                  <div className="rounded border border-slate-700 bg-slate-900/50 p-2">
                    <div className="flex items-center justify-between">
                      <span className="font-medium">{t("welds.seam1")}</span>
                      <span className="rounded bg-orange-500/20 px-1.5 py-0.5 text-[10px] text-orange-300">{t("welds.fillet")}</span>
                    </div>
                    <div className="mt-1 text-[11px] text-slate-400">
                      {t("welds.length", { mm: program.weldLenMm.toFixed(0), n: program.waypoints.length })}
                    </div>
                    <label className="mt-2 block text-[10px] text-slate-500">{t("welds.process")}</label>
                    <select
                      value={weldCondition}
                      onChange={(e) => changeCondition(Number(e.target.value))}
                      className="mt-0.5 w-full rounded bg-slate-800 px-2 py-1 text-xs"
                    >
                      {LORCH_S8_SCHEDULES.map((s) => (
                        <option key={s.condition} value={s.condition}>
                          #{s.condition} {s.process} · {s.currentA}A
                        </option>
                      ))}
                    </select>
                  </div>
                ) : (
                  <p className="px-1 text-xs text-slate-500">{t("welds.empty")}</p>
                )}
              </div>
            )}

            {leftTab === "program" && (
              <div className="space-y-1">
                {program ? (
                  <div className="max-h-full overflow-y-auto font-mono text-[11px]">
                    {program.waypoints.map((wp, i) => (
                      <button
                        key={wp.id}
                        onClick={() => {
                          setManualJog(false);
                          setSimPlaying(false);
                          setSimT(wpProgress[i] ?? 0);
                        }}
                        className={`flex w-full items-center gap-2 rounded px-2 py-0.5 text-left ${
                          sample && i === sample.wpIndex
                            ? "bg-[#e87722]/20 text-[#e87722]"
                            : "text-slate-300 hover:bg-slate-800"
                        }`}
                      >
                        <span className="w-8 text-slate-500">{wp.id}</span>
                        <span className="w-10">{wp.move}</span>
                        <span className="flex-1 truncate">{wp.speedLabel}</span>
                        <span className="text-orange-400">{wp.tag ?? ""}</span>
                      </button>
                    ))}
                  </div>
                ) : (
                  <p className="px-1 text-xs text-slate-500">{t("program.empty")}</p>
                )}
              </div>
            )}
          </div>

          {/* Details */}
          <div className="border-t border-slate-700 bg-[#14171c] p-2">
            <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">{t("details.title")}</div>
            {selectedStation != null && activePositioner ? (
              <div className="space-y-2 text-xs">
                <div className="font-medium">Stół {selectedStation + 1}</div>
                {activePositioner.hasTilt && (
                  <label className="flex items-center gap-2">
                    <span className="w-12 text-slate-400">{t("details.tilt")}</span>
                    <input
                      type="range"
                      min={-135}
                      max={135}
                      step={0.5}
                      value={stations[selectedStation].tilt}
                      onChange={(e) =>
                        setStation(selectedStation, { tilt: Number(e.target.value) })
                      }
                      className="flex-1 accent-[#e87722]"
                    />
                    <span className="w-10 text-right font-mono">
                      {stations[selectedStation].tilt.toFixed(0)}°
                    </span>
                  </label>
                )}
                {activePositioner.hasRotate && (
                  <label className="flex items-center gap-2">
                    <span className="w-12 text-slate-400">{t("details.rotate")}</span>
                    <input
                      type="range"
                      min={-360}
                      max={360}
                      step={0.5}
                      value={stations[selectedStation].rotate}
                      onChange={(e) =>
                        setStation(selectedStation, { rotate: Number(e.target.value) })
                      }
                      className="flex-1 accent-[#e87722]"
                    />
                    <span className="w-10 text-right font-mono">
                      {stations[selectedStation].rotate.toFixed(0)}°
                    </span>
                  </label>
                )}
              </div>
            ) : program ? (
              <dl className="grid grid-cols-2 gap-x-2 gap-y-0.5 text-[11px] text-slate-300">
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
            {status && <p className="mt-2 text-[10px] text-slate-400">{status}</p>}
          </div>
        </aside>

        {/* CENTER — Viewer + sim bar */}
        <div className="relative flex min-w-0 flex-1 flex-col">
          <div className="relative min-h-0 flex-1">
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
            />

            {program && sample && (
              <div className="pointer-events-none absolute left-3 top-3 rounded bg-[#12141a]/90 px-2.5 py-1.5 font-mono text-[11px] text-slate-200 shadow">
                <div className="mb-0.5 font-sans text-[10px] font-semibold uppercase text-slate-400">
                  {program.name}
                </div>
                <div className="flex items-center gap-2">
                  <span
                    className={`inline-block h-2 w-2 rounded-full ${
                      sample.arcOn ? "animate-pulse bg-orange-400" : "bg-slate-600"
                    }`}
                  />
                  ARC {sample.arcOn ? "ON" : "OFF"}
                  <span className="text-slate-500">·</span>
                  {sample.elapsedSec.toFixed(1)}/{program.cycleSec.toFixed(1)}s
                </div>
              </div>
            )}
          </div>

          <div className="flex items-center gap-2 border-t border-slate-700 bg-[#12141a] px-3 py-1.5">
            <button
              onClick={() => stepTo(-1)}
              disabled={!program}
              className={`${btnGhost} !px-2 !py-1`}
              title={t("sim.prev")}
            >
              ⏮
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
              className={`${btnGhost} !px-2 !py-1`}
              title={t("sim.next")}
            >
              ⏭
            </button>
            <button
              onClick={() => {
                setSimPlaying(false);
                setSimT(0);
              }}
              disabled={!program}
              className={`${btnGhost} !px-2 !py-1`}
              title={t("sim.reset")}
            >
              ⟲
            </button>
            <div className="min-w-0 flex-1 truncate px-2 font-mono text-[11px] text-slate-400">
              {instruction}
            </div>
            <select
              value={simSpeed}
              onChange={(e) => setSimSpeed(Number(e.target.value))}
              className="rounded bg-slate-800 px-2 py-1 text-xs"
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
              className="w-40 accent-[#e87722]"
            />
          </div>
        </div>

        {/* RIGHT DOCK — Robot (ALWAYS visible) */}
        <aside className="flex w-80 shrink-0 flex-col border-l border-slate-700 bg-[#181b21]">
          <div className="border-b border-slate-700 px-3 py-2">
            <div className="text-xs font-semibold uppercase tracking-wide text-slate-300">{t("robot.title")}</div>
            <div className="text-[10px] text-slate-500">
              {activeModel?.label ?? "AR2010"} · {t("robot.jointJog")}
            </div>
          </div>

          <div className="flex-1 overflow-y-auto p-3">
            <div
              className={`mb-3 rounded px-2 py-1.5 text-center text-xs font-semibold ${
                manualJog
                  ? "bg-[#e87722]/20 text-[#e87722]"
                  : "bg-slate-800 text-slate-400"
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

            <div className="mb-1 mt-4 text-[10px] font-semibold uppercase text-slate-500">{t("robot.tool")}</div>
            <dl className="grid grid-cols-[1fr_auto] gap-x-2 gap-y-0.5 font-mono text-[11px] text-slate-300">
              <dt className="text-slate-500">{t("robot.frame")}</dt>
              <dd>{t("robot.frameWorld")}</dd>
              <dt className="text-slate-500">{t("robot.tcp")}</dt>
              <dd>
                {sample
                  ? `${sample.pos[0].toFixed(3)}, ${sample.pos[1].toFixed(3)}, ${sample.pos[2].toFixed(3)}`
                  : "—"}
              </dd>
              <dt className="text-slate-500">{t("robot.rail")}</dt>
              <dd>{railTravel.toFixed(3)} m</dd>
            </dl>

            <div className="mb-1 mt-4 text-[10px] font-semibold uppercase text-slate-500">{t("robot.status")}</div>
            <div
              className={`rounded px-2 py-1.5 text-xs font-medium ${
                robotStatus.ok
                  ? "bg-emerald-900/40 text-emerald-300"
                  : "bg-red-900/40 text-red-300"
              }`}
            >
              {robotStatus.ok ? "●" : "⚠"} {robotStatus.label}
            </div>
            <p className="mt-2 text-[10px] leading-snug text-slate-500">
              {t("robot.help")}
            </p>
          </div>
        </aside>
      </div>
    </main>
  );
}
