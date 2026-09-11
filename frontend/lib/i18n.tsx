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
  "status.planDone":
    "Plan gotowy (ścieżka demo). Otwórz zakładkę Program i naciśnij Odtwórz.",
  "status.home": "Robot → pozycja bazowa (wszystkie osie 0°).",
  "status.zero": "Robot → Zero (wszystkie wartości osi = 0°).",
  "status.jogOn": "Ręczny ruch osi WŁĄCZONY.",
  "status.importing": "Import CAD…",
  "status.imported": "Zaimportowano {name}: {n} kandydat(ów) spoiny.",
  "status.cadFallback":
    "Backend CAD niedostępny ({err}). Załadowano demo T-fillet.",
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
  "status.planDone":
    "Plan complete (demo path). Open Program tab and press Play to simulate.",
  "status.home": "Robot → Home pose (all axes 0°).",
  "status.zero": "Robot → Zero (all joint values set to 0°).",
  "status.jogOn": "Manual joint jog ON.",
  "status.importing": "Importing CAD…",
  "status.imported": "Imported {name}: {n} candidate seam(s).",
  "status.cadFallback":
    "CAD backend unavailable ({err}). Loaded demo T-fillet instead.",
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
      className="flex items-center rounded border border-slate-600 p-0.5 text-xs"
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
              : "text-slate-400 hover:text-slate-100"
          }`}
        >
          {l}
        </button>
      ))}
    </div>
  );
}
