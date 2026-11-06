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
  "plan.loadHardDemo": "Demo trudne (90° + rura)",
  "plan.identifyWelds": "Wykryj spoiny",
  "plan.weldSettings": "Ustawienia spoiny",
  "plan.plan": "Planuj",
  "plan.simulate": "Symuluj",
  "plan.generateCode": "Generuj kod",
  "plan.openJbi": "Otwórz .JBI",
  "plan.editJbi": "Edytuj .JBI",

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
  "left.jbi": ".JBI",

  "jbi.editorTitle": "Edytor INFORM .JBI",
  "jbi.editorHint": "Edytuj tekst → Zastosuj, albo zmieniaj punkty w Programie",
  "jbi.apply": "Zastosuj do programu",
  "jbi.reload": "Przebuduj z programu",
  "jbi.dirty": "Niezapisane zmiany w tekście — kliknij Zastosuj, by odświeżyć punkty / 3D.",

  // Mobile bottom navigation
  "mobile.scene": "Scena 3D",
  "mobile.cad": "CAD i Spoiny",
  "mobile.robot": "Robot i Jog",
  "mobile.jbi": "Kod .JBI",
  "mobile.jbiEmpty": "Brak programu — otwórz .JBI, załaduj demo lub zaimportuj CAD.",
  "mobile.jbiDownload": "Pobierz .JBI",
  "mobile.argf": "Motoweld ARGF",
  "mobile.wev": "WEV# (weave)",
  "mobile.stStation": "Stacja ST{n}",
  "mobile.railBs1": "Tor BS1",
  "arcnc.jbi": "7. Kod .JBI",

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
  "program.editTitle": "Edycja punktów",
  "program.insertKind": "Typ nowego punktu",
  "program.insertBefore": "Wstaw przed wybranym",
  "program.insertAfter": "Wstaw za wybranym",
  "program.before": "Przed",
  "program.after": "Za",
  "program.duplicate": "Duplikuj",
  "program.delete": "Usuń",
  "program.changeKind": "Zmień typ punktu",
  "program.xyzMm": "Końcówka palnika tcp_0 [mm]",
  "program.teachReplace": "Ustaw z pozycji palnika",
  "program.teachInsert": "Dodaj punkt w pozycji palnika",
  "program.teachReplaceHint": "Nadpisz C# aktualną końcówką palnika (tcp_0) + kierunkiem drutu",
  "program.teachInsertHint": "Wstaw punkt jałowy w tcp_0 (końcówka palnika)",
  "program.teachHelp":
    "Jog osi → ustaw palnik w widoku 3D → teach bierze końcówkę tcp_0 ze sceny (to, co widać).",
  "status.wpDeleted": "Usunięto {c}.",
  "status.wpInserted": "Wstawiono {kind} → {c}.",
  "status.wpDuplicated": "Zduplikowano → {c}.",
  "status.wpKindChanged": "{c} → {kind}.",
  "status.wpMinPoints": "Program musi mieć co najmniej 2 punkty.",
  "status.wpTaught": "Nauczono {c} ← palnik ({xyz}) mm.",
  "status.wpTaughtInsert": "Dodano {kind} {c} w pozycji palnika.",
  "status.torchPoseStale": "Brak pozycji palnika ze sceny — włącz widok 3D i jog osi.",
  "status.fkFail": "FK niedostępne ({err}). Uruchom backend.",
  "status.planFail": "Plan ruchu nieudany: {err}",
  "status.optimizeFail": "Optymalizacja nieudana: {err}",

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
  "sim.prev": "Poprzedni punkt",
  "sim.play": "Odtwórz",
  "sim.pause": "Pauza",
  "sim.replay": "Ponów",
  "sim.next": "Następny punkt",
  "sim.reset": "Reset",
  "sim.prevC": "Poprzedni {c}",
  "sim.nextC": "Następny {c}",

  // Seam inspector
  "inspector.title": "Inspektor spoiny",
  "inspector.reverse": "Odwróć kierunek spoiny (Start ↔ End)",
  "inspector.spin": "Kąt obrotu palnika wokół drutu (Spin)",
  "inspector.travel": "Kąt pochylenia (Travel Angle)",
  "inspector.approach": "Approach / Retract Distance",
  "inspector.weave": "Zakosowanie WEV#",
  "inspector.weaveNone": "Brak",
  "inspector.speed": "Prędkość spawania V",
  "inspector.empty": "Wybierz spoinę na liście lub w scenie 3D.",
  "status.seamUpdated": "Zaktualizowano {seam} (spin {spin}°).",
  "sketch.title": "Rysuj spoinę na detalu",
  "sketch.orbit": "Widok / Orbit",
  "sketch.tack": "Punkt (Tack)",
  "sketch.line": "Linia 2-pkt",
  "sketch.arc": "Łuk 3-pkt",
  "sketch.spline": "Splajn",
  "sketch.snap": "Snap do krawędzi",
  "sketch.undo": "Cofnij punkt",
  "sketch.cancel": "Anuluj",
  "sketch.finish": "Zakończ krzywą (Enter)",
  "sketch.hintOrbit": "Obracaj kamerę · wybieraj istniejące spoiny",
  "sketch.hintTack": "Kliknij punkt na powierzchni (sczep TIMER T=2.00)",
  "sketch.hintLine": "Kliknij Start, potem End na detalu (MOVL)",
  "sketch.hintArc": "Start → Mid → End — łuk MOVC",
  "sketch.hintSpline": "Klikaj punkty · Enter / podwójne kliknięcie zatwierdza",
  "sketch.deleteSeam": "Usuń zaznaczoną spoinę",
  "status.sketchAdded": "Dodano spoinę {kind} ({n} próbek).",
  "status.sketchFail": "Kreator spoin: {err}",
  "status.sketchCollinear": "Punkty łuku są współliniowe — wybierz inny Mid.",
  "status.seamDeleted": "Usunięto {seam}.",

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
  "robot.tcp": "Palnik tcp_0",
  "robot.rail": "Szyna",
  "robot.status": "Status",
  "robot.statusManual": "OK (ręczny jog)",
  "robot.statusPath": "OK (ścieżka)",
  "robot.statusIdle": "OK",
  "robot.help":
    "Przesuwaj S/L/U/R/B/T, żeby ręcznie ruszać AR2010 (jak panel Robot w Verbotics Weld). Odtwórz przełącza na IK ścieżki. Baza / Zero zerują osie.",

  // Execution / MotoROS2 checklist
  "exec.title": "Wykonanie na robocie",
  "exec.summary":
    "Spawanie: natywny .JBI (ARCON / WVON / MACRO1). HSES (UnderAutomation) — upload/start joba; MotoROS2 — telemetria po DN2.44+.",
  "hses.title": "HSES · UnderAutomation (opcjonalnie)",
  "hses.summary":
    "Wgrywa wygenerowany .JBI na DX200 (FTP/HSES) i uruchamia job. Wymaga pip UnderAutomation.Yaskawa + .NET 8 oraz YASKAWA_ROBOT_HOST (lub YASKAWA_UA_DRY_RUN=1).",
  "hses.host": "Host",
  "hses.refresh": "Odśwież",
  "hses.upload": "Wgraj .JBI",
  "hses.uploadStart": "Wgraj i Start",
  "hses.startOnly": "Start job",
  "hses.jobName": "Nazwa joba",
  "hses.dryRun": "Tryb DRY-RUN (bez robota)",
  "hses.sdkOk": "SDK gotowe",
  "hses.sdkMissing": "SDK / .NET niedostępne — zainstaluj requirements-hses.txt",
  "hses.needJbi": "Najpierw wygeneruj kod (.JBI).",
  "exec.checklistTitle": "Specyfikacja i Checklista DX200 (JBI vs MotoROS2)",
  "exec.firmwareWarn":
    "Dump komórki jest na DN2.27 — przed MotoROS2 wymagany upgrade do DN2.44.00-00 lub nowszego.",
  "exec.firmwareOk": "Firmware zgodny z MotoROS2",
  "exec.firmwareBad": "Firmware za stary na MotoROS2",
  "exec.s2cTitle": "Parametry S2C (DX200)",
  "exec.groupsTitle": "Mapowanie grup RB1/BS1/ST → r1/b1/s",
  "exec.tfTitle": "Łańcuch TF",
  "exec.limitsTitle": "Ograniczenia MotoROS2",
  "exec.jbiNote":
    "follow_joint_trajectory wymaga INIT_ROS i DN2.44+ — tylko telemetria/dojazdy. Spawanie zawsze przez .JBI.",

  // Torch / TOOL 0 calibration
  "torch.panelTitle": "Palnik (TOOL 0) — Lokalizacja i Model 3D",
  "torch.panelHelp":
    "TCP z dumpa DX200 (TOOL.CND: −54.28, 1.05, 415.70 mm · RPY 1.31/−42.62/−6.84°). Sprzęt: ABICOR BINZEL iCAT mini + ABIROB G/W (~45°). Mesh UI: placeholder (brak publicznego STEP G/W+iCAT).",
  "torch.tcpSection": "TCP TOOL 0 (mm / °)",
  "torch.mountSection": "Kalibracja mocowania na kołnierzu",
  "torch.flangePreset": "Oś wyjściowa kołnierza URDF",
  "torch.meshSection": "Model 3D palnika",
  "torch.meshVendor": "Placeholder mesh (czekamy na STEP ABIROB G/W + iCAT)",
  "torch.meshProcedural": "Proceduralny",
  "torch.meshCustom": "Własny STL/GLB",
  "torch.uploadMesh": "Wgraj plik .stl / .glb",
  "torch.customScale": "Skala siatki",
  "torch.showAxes": "Pokaż osie XYZ (kołnierz + TCP)",
  "torch.resetTcp": "Przywróć TCP z dumpa",
  "torch.resetAll": "Reset całej kalibracji",

  // Axis help
  "axis.S": "Obrót podstawy",
  "axis.L": "Ramię dolne",
  "axis.U": "Ramię górne",
  "axis.R": "Obrót ramienia",
  "axis.B": "Zgięcie nadgarstka",
  "axis.T": "Obrót uchwytu",
  "axis.home": "Baza (C00000)",

  // Status messages
  "status.programReady":
    "Program gotowy: T-fillet · {mm} mm · cykl ≈ {sec} s",
  "status.identifyDemo":
    "Wykryj spoiny (demo): utworzono 1 spoinę pachwinową z próbki T.",
  "status.identifyExisting":
    "Wykryj spoiny: spoina z {n} punktami już jest w projekcie.",
  "status.planNeedPart": "Zaimportuj / załaduj część przed planowaniem.",
  "status.planDone": "Plan ruchu gotowy · bez kolizji: {collision}. Otwórz Program i naciśnij Odtwórz.",
  "status.home": "Robot → home C00000 nad szyną BS1 (TOOL 0).",
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
  "status.jbiOpened":
    "Otwarto {name}.JBI · {n} pkt · {postype}{warn}. Edytuj w zakładce .JBI lub Program.",
  "status.jbiOpenFail": "Nie udało się otworzyć .JBI: {err}",
  "status.jbiApplied": "Zastosowano edycję .JBI ({n} linii) → punkty programu odświeżone.",
  "status.jbiReloaded": "Tekst .JBI przebudowany z aktualnego programu.",
  "status.jbiEmpty": "Brak tekstu .JBI do zastosowania.",
  "arcnc.workflow": "Przepływ ArcNC",
  "arcnc.cad": "1. CAD",
  "arcnc.detect": "2. Spoiny",
  "arcnc.plan": "3. Plan ruchu",
  "arcnc.sense": "4. TouchSense",
  "arcnc.sim": "5. Symulacja",
  "arcnc.export": "6. Kod",
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
  "welds.tagTouch": "TOUCH",
  "program.kind.sense": "TouchSense",
  "program.kind.travel": "Dojazd",
  "program.kind.clearance": "Via / clearance",
  "program.kind.index_safe": "EC safe-index",

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
  "plan.loadHardDemo": "Hard demo (90° + pipe)",
  "plan.identifyWelds": "Identify Welds",
  "plan.weldSettings": "Weld Settings",
  "plan.plan": "Plan",
  "plan.simulate": "Simulate",
  "plan.generateCode": "Generate Code",
  "plan.openJbi": "Open .JBI",
  "plan.editJbi": "Edit .JBI",

  "settings.weldCondition": "Weld condition (ARCON)",
  "settings.robot": "Robot",
  "settings.positioner": "Positioner",
  "settings.none": "None",

  "view.help":
    "LMB rotate · RMB pan · scroll zoom. Robot joint jog is always on the right (Verbotics-style).",

  "left.workspace": "Workspace",
  "left.welds": "Welds",
  "left.program": "Program",
  "left.jbi": ".JBI",

  "jbi.editorTitle": "INFORM .JBI editor",
  "jbi.editorHint": "Edit text → Apply, or change waypoints under Program",
  "jbi.apply": "Apply to program",
  "jbi.reload": "Rebuild from program",
  "jbi.dirty": "Unsaved text edits — click Apply to refresh waypoints / 3D.",

  // Mobile bottom navigation
  "mobile.scene": "3D Scene",
  "mobile.cad": "CAD & Welds",
  "mobile.robot": "Robot & Jog",
  "mobile.jbi": ".JBI Code",
  "mobile.jbiEmpty": "No program — open a .JBI, load demo, or import CAD.",
  "mobile.jbiDownload": "Download .JBI",
  "mobile.argf": "Motoweld ARGF",
  "mobile.wev": "WEV# (weave)",
  "mobile.stStation": "Station ST{n}",
  "mobile.railBs1": "Track BS1",
  "arcnc.jbi": "7. .JBI Code",

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
  "program.editTitle": "Edit points",
  "program.insertKind": "New point type",
  "program.insertBefore": "Insert before selected",
  "program.insertAfter": "Insert after selected",
  "program.before": "Before",
  "program.after": "After",
  "program.duplicate": "Duplicate",
  "program.delete": "Delete",
  "program.changeKind": "Change point type",
  "program.xyzMm": "Torch tip tcp_0 [mm]",
  "program.teachReplace": "Set from torch tip",
  "program.teachInsert": "Add point at torch tip",
  "program.teachReplaceHint": "Overwrite C# with current torch tip (tcp_0) + wire direction",
  "program.teachInsertHint": "Insert idle point at tcp_0 (torch tip)",
  "program.teachHelp":
    "Joint-jog the torch in the 3D view — teach samples the visible tcp_0 tip from the scene.",
  "status.wpDeleted": "Deleted {c}.",
  "status.wpInserted": "Inserted {kind} → {c}.",
  "status.wpDuplicated": "Duplicated → {c}.",
  "status.wpKindChanged": "{c} → {kind}.",
  "status.wpMinPoints": "Program needs at least 2 points.",
  "status.wpTaught": "Taught {c} ← torch ({xyz}) mm.",
  "status.wpTaughtInsert": "Added {kind} {c} at torch tip.",
  "status.torchPoseStale": "No torch pose from the scene — show the 3D view and joint-jog.",
  "status.fkFail": "FK unavailable ({err}). Start the backend.",
  "status.planFail": "Motion plan failed: {err}",
  "status.optimizeFail": "Sequence optimize failed: {err}",

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

  "sim.prev": "Previous point",
  "sim.play": "Play",
  "sim.pause": "Pause",
  "sim.replay": "Replay",
  "sim.next": "Next point",
  "sim.reset": "Reset",
  "sim.prevC": "Previous {c}",
  "sim.nextC": "Next {c}",

  "inspector.title": "Seam inspector",
  "inspector.reverse": "Reverse seam direction (Start ↔ End)",
  "inspector.spin": "Torch spin about wire",
  "inspector.travel": "Travel angle",
  "inspector.approach": "Approach / Retract distance",
  "inspector.weave": "Weave WEV#",
  "inspector.weaveNone": "None",
  "inspector.speed": "Weld speed V",
  "inspector.empty": "Select a seam from the list or 3D scene.",
  "status.seamUpdated": "Updated {seam} (spin {spin}°).",
  "sketch.title": "Draw seam on part",
  "sketch.orbit": "View / Orbit",
  "sketch.tack": "Point (Tack)",
  "sketch.line": "2-pt Line",
  "sketch.arc": "3-pt Arc",
  "sketch.spline": "Spline",
  "sketch.snap": "Edge snap",
  "sketch.undo": "Undo point",
  "sketch.cancel": "Cancel",
  "sketch.finish": "Finish curve (Enter)",
  "sketch.hintOrbit": "Orbit the camera · select existing seams",
  "sketch.hintTack": "Click a surface point (tack TIMER T=2.00)",
  "sketch.hintLine": "Click Start, then End on the part (MOVL)",
  "sketch.hintArc": "Start → Mid → End — MOVC arc",
  "sketch.hintSpline": "Click points · Enter / double-click to commit",
  "sketch.deleteSeam": "Delete selected seam",
  "status.sketchAdded": "Added {kind} seam ({n} samples).",
  "status.sketchFail": "Seam sketcher: {err}",
  "status.sketchCollinear": "Arc points are collinear — pick another Mid.",
  "status.seamDeleted": "Deleted {seam}.",

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
  "robot.tcp": "Torch tip tcp_0",
  "robot.rail": "Rail",
  "robot.status": "Status",
  "robot.statusManual": "Valid (manual jog)",
  "robot.statusPath": "Valid (path)",
  "robot.statusIdle": "Valid",
  "robot.help":
    "Move S/L/U/R/B/T to jog the AR2010 by hand (like Verbotics Weld’s Robot panel). Play switches to path IK. Home / Zero reset the axes.",

  "exec.title": "Robot execution",
  "exec.summary":
    "Welding: native .JBI (ARCON / WVON / MACRO1). HSES (UnderAutomation) — upload/start job; MotoROS2 — telemetry after DN2.44+.",
  "hses.title": "HSES · UnderAutomation (optional)",
  "hses.summary":
    "Pushes the generated .JBI to the DX200 (FTP/HSES) and starts the job. Needs pip UnderAutomation.Yaskawa + .NET 8 and YASKAWA_ROBOT_HOST (or YASKAWA_UA_DRY_RUN=1).",
  "hses.host": "Host",
  "hses.refresh": "Refresh",
  "hses.upload": "Upload .JBI",
  "hses.uploadStart": "Upload & Start",
  "hses.startOnly": "Start job",
  "hses.jobName": "Job name",
  "hses.dryRun": "DRY-RUN mode (no robot)",
  "hses.sdkOk": "SDK ready",
  "hses.sdkMissing": "SDK / .NET missing — install requirements-hses.txt",
  "hses.needJbi": "Generate code (.JBI) first.",
  "exec.checklistTitle": "DX200 Spec & Checklist (JBI vs MotoROS2)",
  "exec.firmwareWarn":
    "Cell dump is DN2.27 — upgrade to DN2.44.00-00 or newer before installing MotoROS2.",
  "exec.firmwareOk": "Firmware OK for MotoROS2",
  "exec.firmwareBad": "Firmware too old for MotoROS2",
  "exec.s2cTitle": "S2C parameters (DX200)",
  "exec.groupsTitle": "Group map RB1/BS1/ST → r1/b1/s",
  "exec.tfTitle": "TF chain",
  "exec.limitsTitle": "MotoROS2 limitations",
  "exec.jbiNote":
    "follow_joint_trajectory needs INIT_ROS and DN2.44+ — telemetry/air only. Welding always via .JBI.",

  // Torch / TOOL 0 calibration
  "torch.panelTitle": "Torch (TOOL 0) — Location & 3D Model",
  "torch.panelHelp":
    "TCP from DX200 TOOL.CND (−54.28, 1.05, 415.70 mm · RPY 1.31/−42.62/−6.84°). Hardware: ABICOR BINZEL iCAT mini + ABIROB G/W (~45°). UI mesh: placeholder (no public G/W+iCAT STEP yet).",
  "torch.tcpSection": "TCP TOOL 0 (mm / °)",
  "torch.mountSection": "Flange mount calibration",
  "torch.flangePreset": "URDF flange outgoing axis",
  "torch.meshSection": "Torch 3D model",
  "torch.meshVendor": "Placeholder mesh (awaiting ABIROB G/W + iCAT STEP)",
  "torch.meshProcedural": "Procedural",
  "torch.meshCustom": "Custom STL/GLB",
  "torch.uploadMesh": "Upload .stl / .glb",
  "torch.customScale": "Mesh scale",
  "torch.showAxes": "Show XYZ axes (flange + TCP)",
  "torch.resetTcp": "Restore dump TCP",
  "torch.resetAll": "Reset all calibration",

  "axis.S": "Base rotation",
  "axis.L": "Lower arm",
  "axis.U": "Upper arm",
  "axis.R": "Arm roll",
  "axis.B": "Wrist bend",
  "axis.T": "Torch roll",
  "axis.home": "Home (C00000)",

  "status.programReady":
    "Program ready: T-fillet · {mm} mm · cycle ≈ {sec} s",
  "status.identifyDemo":
    "Identify Welds (demo): 1 fillet seam created from T-coupon.",
  "status.identifyExisting":
    "Identify Welds: seam with {n} points already in project.",
  "status.planNeedPart": "Import / load a part before planning.",
  "status.planDone": "Motion plan ready · collision-free: {collision}. Open Program and press Play.",
  "status.home": "Robot → home C00000 over RECT-X (TOOL 0).",
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
  "status.jbiOpened":
    "Opened {name}.JBI · {n} pts · {postype}{warn}. Edit under .JBI or Program.",
  "status.jbiOpenFail": "Failed to open .JBI: {err}",
  "status.jbiApplied": "Applied .JBI edits ({n} lines) → program waypoints refreshed.",
  "status.jbiReloaded": ".JBI text rebuilt from the current program.",
  "status.jbiEmpty": "No .JBI text to apply.",
  "arcnc.workflow": "ArcNC workflow",
  "arcnc.cad": "1. CAD",
  "arcnc.detect": "2. Seams",
  "arcnc.plan": "3. Motion plan",
  "arcnc.sense": "4. TouchSense",
  "arcnc.sim": "5. Simulate",
  "arcnc.export": "6. Code",
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
  "welds.tagTouch": "TOUCH",
  "program.kind.sense": "TouchSense",
  "program.kind.travel": "Travel",
  "status.touchSenseAdded": "Added TouchSense points before the seam.",
  "status.sequenceOptimized": "Optimized weld sequence.",
  "program.kind.approach": "Approach",
  "program.kind.weld": "Weld",
  "program.kind.retract": "Retract",
  "program.kind.clearance": "Via / clearance",
  "program.kind.index_safe": "EC safe-index",

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
