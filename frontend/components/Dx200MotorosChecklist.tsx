"use client";

/**
 * Collapsible DX200 / MotoROS2 readiness checklist (JBI vs ROS).
 * Spec from Yaskawa-Global/motoros2 README.md + doc/ros_api.md.
 */

const CELL_FW = "DN2.27.00A(US/PL)-00";
const MIN_FW = "DN2.44.00-00";

const S2C: { key: string; value: number; note?: string }[] = [
  { key: "S2C541", value: 0 },
  { key: "S2C542", value: 0 },
  { key: "S2C1102", value: 2 },
  { key: "S2C1104", value: 2 },
  { key: "S2C1117", value: 1, note: "DX200 Speed FB → alarm 8001[10], reboot" },
  { key: "S2C1250", value: 1 },
  { key: "S2C1402", value: 3 },
];

const GROUPS: { inform: string; ros: string; unit: string; role: string }[] = [
  { inform: "RB1", ros: "r1", unit: "rad", role: "MA2010" },
  { inform: "BS1", ros: "b1", unit: "m", role: "RECT-X" },
  { inform: "ST1", ros: "s1", unit: "rad", role: "TURN 1" },
  { inform: "ST2", ros: "s2", unit: "rad", role: "TURN 2" },
  { inform: "ST3", ros: "s3", unit: "rad", role: "TURN 3" },
  { inform: "ST4", ros: "s4", unit: "rad", role: "TURN 4" },
];

export default function Dx200MotorosChecklist({
  title,
  firmwareWarn,
  firmwareOkLabel,
  firmwareBadLabel,
  s2cTitle,
  groupsTitle,
  tfTitle,
  jbiNote,
  limitsTitle,
}: {
  title: string;
  firmwareWarn: string;
  firmwareOkLabel: string;
  firmwareBadLabel: string;
  s2cTitle: string;
  groupsTitle: string;
  tfTitle: string;
  jbiNote: string;
  limitsTitle: string;
}) {
  const fwOk = false; // dump is DN2.27 — below DN2.44

  return (
    <details className="mt-4 rounded border border-slate-200 bg-slate-50 open:bg-white">
      <summary className="cursor-pointer select-none px-2 py-2 text-[11px] font-semibold uppercase tracking-wide text-slate-700">
        {title}
      </summary>
      <div className="space-y-3 border-t border-slate-200 px-2 py-2 text-[11px] text-slate-700">
        <div>
          <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">
            Firmware
          </div>
          <div
            className={`rounded px-2 py-1.5 font-medium ${
              fwOk ? "bg-emerald-50 text-emerald-800" : "bg-amber-50 text-amber-900"
            }`}
          >
            {fwOk ? firmwareOkLabel : firmwareBadLabel}
            <div className="mt-0.5 font-mono text-[10px] opacity-90">
              cell {CELL_FW} · need ≥ {MIN_FW}
            </div>
          </div>
          <p className="mt-1 text-[10px] leading-snug text-slate-500">{firmwareWarn}</p>
        </div>

        <div>
          <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">
            {s2cTitle}
          </div>
          <ul className="space-y-0.5 font-mono text-[10px]">
            {S2C.map((p) => (
              <li key={p.key} className="flex flex-wrap items-baseline gap-x-2">
                <span className="text-slate-800">
                  {p.key}={p.value}
                </span>
                {p.note && <span className="text-slate-500">{p.note}</span>}
              </li>
            ))}
          </ul>
          <p className="mt-1 text-[10px] text-slate-500">
            USB <span className="font-mono">motoros2_config.yaml</span> →{" "}
            <span className="font-mono">CN106</span> (alarm 8013 if missing).
            Binaries: <span className="font-mono">mr2_dx2_h.out</span> /
            <span className="font-mono"> mr2_dx2_j.out</span> (MotoROS2 0.2.1).
          </p>
        </div>

        <div>
          <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">
            {groupsTitle}
          </div>
          <table className="w-full text-left font-mono text-[10px]">
            <thead>
              <tr className="text-slate-500">
                <th className="py-0.5 font-medium">JBI</th>
                <th className="py-0.5 font-medium">ROS</th>
                <th className="py-0.5 font-medium">unit</th>
                <th className="py-0.5 font-medium">role</th>
              </tr>
            </thead>
            <tbody>
              {GROUPS.map((g) => (
                <tr key={g.inform} className="border-t border-slate-100">
                  <td className="py-0.5">{g.inform}</td>
                  <td className="py-0.5">ctrl_groups/{g.ros}/joint_states</td>
                  <td className="py-0.5">{g.unit}</td>
                  <td className="py-0.5 text-slate-600">{g.role}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>

        <div>
          <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">
            {tfTitle}
          </div>
          <p className="font-mono text-[10px] leading-relaxed text-slate-700">
            world → base → flange (REP-103 X+) → tool0 (Z+) → tcp_0
            <br />
            TOOL.CND: −54.280, 1.052, 415.702 mm · RPY 1.3119/−42.6215/−6.8387°
          </p>
        </div>

        <div>
          <div className="mb-1 text-[10px] font-semibold uppercase text-slate-500">
            {limitsTitle}
          </div>
          <ul className="list-inside list-disc space-y-0.5 text-[10px] text-slate-600">
            <li>FastDDS only · no Cartesian / async / partial goals</li>
            <li>Maintenance: MotoPlus FUNC=USED, MOTOMAN DRIVER=USED</li>
            <li>No Simple Connect · FSU CRC check off during install</li>
          </ul>
        </div>

        <p className="rounded border border-orange-200 bg-orange-50 px-2 py-1.5 text-[10px] leading-snug text-amber-900">
          {jbiNote}
        </p>
      </div>
    </details>
  );
}
