"use client";

import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from "react";

export type Locale = "pl" | "en";

/** Flat string dictionary. Use `{name}` placeholders with `t(key, { name: "…" })`. */
export type Dict = Record<string, string>;

const pl: Dict = {
  // App chrome
  "app.title": "Yaskawa Nawigator Spawania",
  "app.subtitle": "DX200 · AR2010 · H1000D · Lorch S8",
  "app.styleHint": "UI w stylu Verbotics Weld",
  "app.warning":
    "⚠ Prototyp — kinematyka / kolizje / parametry spoiny są orientacyjne. Weryfikuj w MotoSim przed realnym stanowiskiem.",

  // Header
  "header.newProject": "Nowy projekt / Demo",
  "header.generateCode": "Generuj kod",
  "header.lang": "Język",

  // Ribbon tabs
  "ribbon.plan": "Plan",
  "ribbon.settings": "Ustawienia",
  "ribbon.view": "Widok",

  // Plan actions
  "plan.importPart": "Importuj część",
  "plan.loadDemo": "Załaduj demo",
  "plan.identifyWelds": "Wykryj spoiny",
  "plan.weldSettings": "Ustawienia spoiny",
  "plan.plan": "Planuj",
  "plan.simulate": "Symuluj",
  "plan.generateCode": "Generuj kod",

  // Settings
  "settings.weldCondition": "Warunek spoiny (ARCON)",
  "settings.robot": "Robot",
  "settings.positioner": "Pozycjoner",
  "settings.none": "Brak",

  // View help
  "view.help":
    "LPM obrót · PPM przesuw · kółko zoom. Ręczny ruch osi robota jest zawsze po prawej (jak w Verbotics Weld).",

  // Left tabs
  "left.workspace": "Stanowisko",
  "left.welds": "Spoiny",
  "left.program": "Program",

  // Workspace
  "workspace.cell": "Komórka",
  "workspace.robot": "Robot",
  "workspace.rail": "Przejazd szyny",
  "workspace.table": "Stół {n}",
  "workspace.parts": "Części",
  "workspace.noPart": "Brak części — Importuj część lub Załaduj demo",
  "workspace.mountedOn": "Zamocowano na Stole 1 · próbka T-fillet",
  "workspace.extraSeams": "Dodatkowe spoiny z CAD: {n}",
  "workspace.positioner": "pozycjoner",

  // Welds
  "welds.seam1": "Spoina 1",
  "welds.fillet": "pachwinowa",
  "welds.length": "Długość {mm} mm · {n} punktów",
  "welds.process": "Proces spawania",
  "welds.empty": "Brak spoin. Użyj Plan → Wykryj spoiny.",

  // Program
  "program.empty": "Brak programu. Najpierw zaplanuj spoiny.",

  // Details
  "details.title": "Szczegóły",
  "details.tilt": "Pochylenie",
  "details.rotate": "Obrót",
  "details.part": "Część",
  "details.weldLen": "Dł. spoiny",
  "details.cycle": "Cykl",
  "details.mode": "Tryb",
  "details.modeManual": "Ręczny jog",
  "details.modePath": "IK ścieżki",
  "details.empty": "Wybierz element komórki lub załaduj część.",

  // Simulation bar
  "sim.prev": "Poprzedni",
  "sim.play": "Odtwórz",
  "sim.pause": "Pauza",
  "sim.replay": "Ponów",
  "sim.next": "Następny",
  "sim.reset": "Reset",

  // Robot dock
  "robot.title": "Robot",
  "robot.jointJog": "ruch osi",
  "robot.modeManual": "● Ręczny ruch osi",
  "robot.modePath": "Sterowanie ścieżką / IK (Odtwórz)",
  "robot.home": "Baza",
  "robot.zero": "Zero",
  "robot.takeJog": "Przejmij jog",
  "robot.joints": "Osie",
  "robot.tool": "Narzędzie",
  "robot.frame": "Układ",
  "robot.frameWorld": "Świat",
  "robot.tcp": "TCP",
  "robot.rail": "Szyna",
  "robot.status": "Status",
  "robot.statusManual": "OK (ręczny jog)",
  "robot.statusPath": "OK (ścieżka)",
  "robot.statusIdle": "OK",
  "robot.help":
    "Przesuwaj S/L/U/R/B/T, żeby ręcznie ruszać AR2010 (jak panel Robot w Verbotics Weld). Odtwórz przełącza na IK ścieżki. Baza / Zero zerują osie.",

  // Axis help
  "axis.S": "Obrót podstawy",
  "axis.L": "Ramię dolne",
  "axis.U": "Ramię górne",
  "axis.R": "Obrót ramienia",
  "axis.B": "Zgięcie nadgarstka",
  "axis.T": "Obrót uchwytu",
  "axis.home": "Baza (0°)",

  // Status messages
  "status.programReady":
    "Program gotowy: T-fillet · {mm} mm · cykl ≈ {sec} s",
  "status.identifyDemo":
    "Wykryj spoiny (demo): utworzono 1 spoinę pachwinową z próbki T.",
  "status.identifyExisting":
    "Wykryj spoiny: spoina z {n} punktami już jest w projekcie.",
  "status.planNeedPart": "Zaimportuj / załaduj część przed planowaniem.",
  "status.planDone": "Plan ruchu gotowy · bez kolizji: {collision}. Otwórz Program i naciśnij Odtwórz.",
  "status.home": "Robot → pozycja bazowa (wszystkie osie 0°).",
  "status.zero": "Robot → Zero (wszystkie wartości osi = 0°).",
  "status.jogOn": "Ręczny ruch osi WŁĄCZONY.",
  "status.importing": "Import CAD…",
  "status.imported": "Zaimportowano {name}: {n} kandydat(ów) spoiny.",
  "status.cadFallback":
    "Backend CAD niedostępny ({err}). Załadowano demo T-fillet.",
  "status.exportIk": "Liczenie IK → eksport PULSE…",
  "status.exportPulse": "Wyeksportowano PULSE .JBI ({n} punktów). Sprawdź w MotoSim.",
  "status.exportRectan":
    "Wyeksportowano RECTAN review .JBI (brak IK / backend). EC/BC w impulsach komórki.",
  "arcnc.workflow": "Przepływ jak ArcNC",
  "arcnc.cad": "1. CAD",
  "arcnc.detect": "2. Spoiny",
  "arcnc.preset": "3. WPS",
  "arcnc.plan": "4. Plan ruchu",
  "arcnc.sense": "5. TouchSense",
  "arcnc.sim": "6. Symulacja",
  "arcnc.export": "7. Kod .JBI",
  "arcnc.planner": "Wynik planera",
  "arcnc.collisionFree": "Bez kolizji",
  "arcnc.touchSense": "TouchSense",
  "arcnc.sequence": "Sekwencja zoptymalizowana",
  "arcnc.singularity": "Bez singularności",
  "arcnc.yes": "TAK",
  "arcnc.no": "NIE",
  "arcnc.optimizeSeq": "Optymalizuj kolejność",
  "arcnc.addSense": "Dodaj TouchSense",
  "program.kind.retract": "Odlegnięcie",
  "program.kind.weld": "Spoina",
  "program.kind.approach": "Dojazd",
  "status.sequenceOptimized": "Zoptymalizowano kolejność spawania.",
  "status.touchSenseAdded": "Dodano punkty TouchSense przed spoiną.",
  "arcnc.planMotion": "Planuj ruchy (auto)",
  "arcnc.generate": "Generuj program",
  "arcnc.reachable": "W zasięgu (IK)",
  "arcnc.noApi": "Brak publicznego API ArcNC. Robot dostaje natywny program INFORM.",
  "robot.linkTitle": "Wykonanie na robocie",
  "robot.linkJbi": "Pobranie .JBI — język DX200 (RB1, BS1, ST1, ARCON, Motoweld).",
  "robot.linkRosOff": "MotoROS2: agent nie jest skonfigurowany. To osobny strumień osi, nie program spoiny.",
  "robot.linkRosOn": "MotoROS2: agent {host} nasłuchuje.",
  "robot.linkRosDown": "MotoROS2: {host} nie odpowiada ({detail}).",
  "robot.linkLegacy": "Starszy ROS 1: motoman_driver + MotoROS na DX200.",
  "robot.linkOffline": "Backend nie zgłosił stanu połączenia.",
  "welds.accept": "Spawaj",
  "welds.omitted": "Pominięto {n} krótszych spoin (limit planu).",
  "welds.reason.flat": "pachwina, pozycja pozioma",
  "welds.reason.vertical": "pachwina pionowa — niższy prąd",
  "welds.reason.long": "długa spoina pozioma",
  "welds.reason.manual": "wybór operatora",
  "status.importedProgram":
    "Część {name}: {n} spoin, WPS #{cond}. Zostaw te, które mają być spawane, i wygeneruj program.",
  "status.noSeams":
    "Brak wklęsłych krawędzi około 90°. To nie wygląda na spoinę pachwinową.",
  "status.generated": "Program gotowy: ruchy, TouchSense, zasięg IK: {reach}.",
  "status.seamToggle": "Zostało {n} spoin do planu.",
  "welds.tagTouch": "TOUCH",
  "program.kind.sense": "TouchSense",
  "program.kind.travel": "Dojazd",

};

const en: Dict = {
  "app.title": "Yaskawa Welding Navigator",
  "app.subtitle": "DX200 · AR2010 · H1000D · Lorch S8",
  "app.styleHint": "Verbotics Weld–style UI",
  "app.warning":
    "⚠ Prototype — kinematics / collisions / weld params are placeholders. Validate in MotoSim before real hardware.",

  "header.newProject": "New Project / Demo",
  "header.generateCode": "Generate Code",
  "header.lang": "Language",

  "ribbon.plan": "Plan",
  "ribbon.settings": "Settings",
  "ribbon.view": "View",

  "plan.importPart": "Import Part",
  "plan.loadDemo": "Load Demo",
  "plan.identifyWelds": "Identify Welds",
  "plan.weldSettings": "Weld Settings",
  "plan.plan": "Plan",
  "plan.simulate": "Simulate",
  "plan.generateCode": "Generate Code",

  "settings.weldCondition": "Weld condition (ARCON)",
  "settings.robot": "Robot",
  "settings.positioner": "Positioner",
  "settings.none": "None",

  "view.help":
    "LMB rotate · RMB pan · scroll zoom. Robot joint jog is always on the right (Verbotics-style).",

  "left.workspace": "Workspace",
  "left.welds": "Welds",
  "left.program": "Program",

  "workspace.cell": "Cell",
  "workspace.robot": "Robot",
  "workspace.rail": "Rail travel",
  "workspace.table": "Table {n}",
  "workspace.parts": "Parts",
  "workspace.noPart": "No part — Import Part or Load Demo",
  "workspace.mountedOn": "Mounted on Table 1 · T-fillet coupon",
  "workspace.extraSeams": "Extra CAD seams: {n}",
  "workspace.positioner": "positioner",

  "welds.seam1": "Seam 1",
  "welds.fillet": "fillet",
  "welds.length": "Length {mm} mm · {n} waypoints",
  "welds.process": "Weld process",
  "welds.empty": "No welds yet. Use Plan → Identify Welds.",

  "program.empty": "No program. Plan welds first.",

  "details.title": "Details",
  "details.tilt": "Tilt",
  "details.rotate": "Rotate",
  "details.part": "Part",
  "details.weldLen": "Weld len",
  "details.cycle": "Cycle",
  "details.mode": "Mode",
  "details.modeManual": "Manual jog",
  "details.modePath": "Path IK",
  "details.empty": "Select a cell item or load a part.",

  "sim.prev": "Previous",
  "sim.play": "Play",
  "sim.pause": "Pause",
  "sim.replay": "Replay",
  "sim.next": "Next",
  "sim.reset": "Reset",

  "robot.title": "Robot",
  "robot.jointJog": "joint jog",
  "robot.modeManual": "● Manual joint jog",
  "robot.modePath": "Path / IK control (Play)",
  "robot.home": "Home",
  "robot.zero": "Zero",
  "robot.takeJog": "Take jog",
  "robot.joints": "Joints",
  "robot.tool": "Tool",
  "robot.frame": "Frame",
  "robot.frameWorld": "World",
  "robot.tcp": "TCP",
  "robot.rail": "Rail",
  "robot.status": "Status",
  "robot.statusManual": "Valid (manual jog)",
  "robot.statusPath": "Valid (path)",
  "robot.statusIdle": "Valid",
  "robot.help":
    "Move S/L/U/R/B/T to jog the AR2010 by hand (like Verbotics Weld’s Robot panel). Play switches to path IK. Home / Zero reset the axes.",

  "axis.S": "Base rotation",
  "axis.L": "Lower arm",
  "axis.U": "Upper arm",
  "axis.R": "Arm roll",
  "axis.B": "Wrist bend",
  "axis.T": "Torch roll",
  "axis.home": "Home (0°)",

  "status.programReady":
    "Program ready: T-fillet · {mm} mm · cycle ≈ {sec} s",
  "status.identifyDemo":
    "Identify Welds (demo): 1 fillet seam created from T-coupon.",
  "status.identifyExisting":
    "Identify Welds: seam with {n} points already in project.",
  "status.planNeedPart": "Import / load a part before planning.",
  "status.planDone": "Motion plan ready · collision-free: {collision}. Open Program and press Play.",
  "status.home": "Robot → Home pose (all axes 0°).",
  "status.zero": "Robot → Zero (all joint values set to 0°).",
  "status.jogOn": "Manual joint jog ON.",
  "status.importing": "Importing CAD…",
  "status.imported": "Imported {name}: {n} candidate seam(s).",
  "status.cadFallback":
    "CAD backend unavailable ({err}). Loaded demo T-fillet instead.",
  "status.exportIk": "Solving IK → PULSE export…",
  "status.exportPulse": "Exported PULSE .JBI ({n} points). Validate in MotoSim.",
  "status.exportRectan":
    "Exported RECTAN review .JBI (no IK / backend). EC/BC use cell pulse units.",
  "arcnc.workflow": "ArcNC-style flow",
  "arcnc.cad": "1. CAD",
  "arcnc.detect": "2. Seams",
  "arcnc.preset": "3. WPS",
  "arcnc.plan": "4. Motion plan",
  "arcnc.sense": "5. TouchSense",
  "arcnc.sim": "6. Simulate",
  "arcnc.export": "7. .JBI code",
  "arcnc.planner": "Planner result",
  "arcnc.collisionFree": "Collision-free",
  "arcnc.touchSense": "TouchSense",
  "arcnc.sequence": "Sequence optimized",
  "arcnc.singularity": "Singularity-safe",
  "arcnc.yes": "YES",
  "arcnc.no": "NO",
  "arcnc.optimizeSeq": "Optimize sequence",
  "arcnc.addSense": "Add TouchSense",
  "arcnc.planMotion": "Plan motions (auto)",
  "arcnc.generate": "Generate program",
  "arcnc.reachable": "In reach (IK)",
  "arcnc.noApi": "ArcNC has no public API. The robot receives a native INFORM program.",
  "robot.linkTitle": "Run on the robot",
  "robot.linkJbi": "Download .JBI — the DX200 language (RB1, BS1, ST1, ARCON, Motoweld).",
  "robot.linkRosOff": "MotoROS2: no agent configured. That is a separate joint stream, not the weld job.",
  "robot.linkRosOn": "MotoROS2: agent {host} is listening.",
  "robot.linkRosDown": "MotoROS2: {host} did not answer ({detail}).",
  "robot.linkLegacy": "Older ROS 1: motoman_driver + MotoROS on DX200.",
  "robot.linkOffline": "Backend did not report a link status.",
  "welds.accept": "Weld",
  "welds.omitted": "Left out {n} shorter seams (plan limit).",
  "welds.reason.flat": "fillet, flat position",
  "welds.reason.vertical": "vertical fillet — lower current",
  "welds.reason.long": "long flat weld",
  "welds.reason.manual": "operator choice",
  "status.importedProgram":
    "Part {name}: {n} seams, WPS #{cond}. Keep the ones to weld, then generate the program.",
  "status.noSeams": "No concave edges near 90°. This does not look like a fillet.",
  "status.generated": "Program ready: motions, TouchSense, IK reach: {reach}.",
  "status.seamToggle": "{n} seams left in the plan.",
  "welds.tagTouch": "TOUCH",
  "program.kind.sense": "TouchSense",
  "program.kind.travel": "Travel",
  "status.touchSenseAdded": "Added TouchSense points before the seam.",
  "status.sequenceOptimized": "Optimized weld sequence.",
  "program.kind.approach": "Approach",
  "program.kind.weld": "Weld",
  "program.kind.retract": "Retract",

};

const DICTS: Record<Locale, Dict> = { pl, en };

export type TFunc = (key: string, vars?: Record<string, string | number>) => string;

type I18nCtx = {
  locale: Locale;
  setLocale: (l: Locale) => void;
  t: TFunc;
};

const Ctx = createContext<I18nCtx | null>(null);

const STORAGE_KEY = "yaskawa-weld-locale";

function format(template: string, vars?: Record<string, string | number>) {
  if (!vars) return template;
  return template.replace(/\{(\w+)\}/g, (_, k: string) =>
    vars[k] != null ? String(vars[k]) : `{${k}}`,
  );
}

export function I18nProvider({ children }: { children: ReactNode }) {
  const [locale, setLocaleState] = useState<Locale>("pl");

  useEffect(() => {
    try {
      const saved = localStorage.getItem(STORAGE_KEY) as Locale | null;
      if (saved === "pl" || saved === "en") setLocaleState(saved);
    } catch {
      /* ignore */
    }
  }, []);

  const setLocale = useCallback((l: Locale) => {
    setLocaleState(l);
    try {
      localStorage.setItem(STORAGE_KEY, l);
    } catch {
      /* ignore */
    }
    if (typeof document !== "undefined") {
      document.documentElement.lang = l;
    }
  }, []);

  useEffect(() => {
    document.documentElement.lang = locale;
  }, [locale]);

  const t = useCallback<TFunc>(
    (key, vars) => {
      const dict = DICTS[locale] ?? pl;
      const raw = dict[key] ?? DICTS.en[key] ?? key;
      return format(raw, vars);
    },
    [locale],
  );

  const value = useMemo(() => ({ locale, setLocale, t }), [locale, setLocale, t]);

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useI18n() {
  const ctx = useContext(Ctx);
  if (!ctx) throw new Error("useI18n must be used within I18nProvider");
  return ctx;
}

/** Compact PL | EN toggle for the app header. */
export function LanguageToggle() {
  const { locale, setLocale, t } = useI18n();
  return (
    <div
      className="flex items-center rounded border border-slate-300 bg-white p-0.5 text-xs"
      title={t("header.lang")}
      role="group"
      aria-label={t("header.lang")}
    >
      {(["pl", "en"] as Locale[]).map((l) => (
        <button
          key={l}
          type="button"
          onClick={() => setLocale(l)}
          className={`rounded px-2 py-1 font-semibold uppercase transition ${
            locale === l
              ? "bg-[#e87722] text-white"
              : "text-slate-500 hover:text-slate-800"
          }`}
        >
          {l}
        </button>
      ))}
    </div>
  );
}
