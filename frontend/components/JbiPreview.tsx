"use client";

import { summarizeJbi, type JbiPreviewSummary } from "@/lib/weldProgram";

function SummaryChips({
  summary,
  ikPulse,
  t,
}: {
  summary: JbiPreviewSummary;
  ikPulse: boolean;
  t: (key: string, vars?: Record<string, string | number>) => string;
}) {
  const mode = ikPulse && summary.pulse && !summary.rectan
    ? t("jbi.modePulse")
    : t("jbi.modeRectan");
  const chips: { label: string; value: string }[] = [
    { label: "NPOS", value: summary.npos ?? "—" },
    { label: t("jbi.mode"), value: mode },
    { label: "GROUP2", value: summary.group2 ?? "—" },
    {
      label: "MACRO1 ARGF",
      value: summary.macroArgs.length
        ? summary.macroArgs.map((n) => `#${n}`).join(", ")
        : "—",
    },
    {
      label: "WVON WEV#",
      value: summary.weaveNos.length
        ? summary.weaveNos.map((n) => String(n)).join(", ")
        : t("jbi.noWeave"),
    },
    {
      label: "BC#",
      value:
        summary.bcMin != null && summary.bcMax != null
          ? `${summary.bcMin} … ${summary.bcMax}`
          : "—",
    },
    {
      label: "EC#",
      value: summary.ecSamples.length ? summary.ecSamples.join(" · ") : "—",
    },
    { label: t("jbi.seams"), value: String(summary.seamCount) },
  ];
  return (
    <div className="flex flex-wrap gap-1.5">
      {chips.map((c) => (
        <div
          key={c.label}
          className="rounded border border-slate-200 bg-slate-50 px-2 py-1 text-[10px]"
        >
          <div className="font-semibold uppercase tracking-wide text-slate-500">
            {c.label}
          </div>
          <div className="font-mono text-slate-800">{c.value}</div>
        </div>
      ))}
      <div
        className={`rounded border px-2 py-1 text-[10px] ${
          ikPulse
            ? "border-emerald-300 bg-emerald-50 text-emerald-800"
            : "border-amber-300 bg-amber-50 text-amber-800"
        }`}
      >
        <div className="font-semibold uppercase tracking-wide opacity-70">
          {t("jbi.ikStatus")}
        </div>
        <div className="font-medium">
          {ikPulse ? t("jbi.ikOk") : t("jbi.ikMissing")}
        </div>
      </div>
    </div>
  );
}

/**
 * Full-text INFORM .JBI preview with line numbers + header summary chips.
 */
export default function JbiPreview({
  text,
  ikPulse,
  onClose,
  onDownload,
  t,
}: {
  text: string;
  ikPulse: boolean;
  onClose?: () => void;
  onDownload?: () => void;
  t: (key: string, vars?: Record<string, string | number>) => string;
}) {
  const summary = summarizeJbi(text);
  const lines = text.replace(/\r\n/g, "\n").replace(/\n$/, "").split("\n");

  return (
    <div className="flex h-full min-h-0 flex-col">
      <div className="flex items-start justify-between gap-2 border-b border-slate-200 px-3 py-2">
        <div>
          <div className="text-xs font-semibold uppercase tracking-wide text-slate-700">
            {t("jbi.previewTitle")}
          </div>
          <div className="text-[10px] text-slate-500">
            {t("jbi.previewSub", { n: summary.lineCount })}
          </div>
        </div>
        <div className="flex shrink-0 gap-1">
          {onDownload && (
            <button
              type="button"
              onClick={onDownload}
              className="rounded bg-[#e87722] px-2.5 py-1 text-[11px] font-semibold text-white hover:brightness-110"
            >
              {t("jbi.download")}
            </button>
          )}
          {onClose && (
            <button
              type="button"
              onClick={onClose}
              className="rounded border border-slate-300 bg-white px-2.5 py-1 text-[11px] text-slate-700 hover:bg-slate-50"
            >
              {t("jbi.close")}
            </button>
          )}
        </div>
      </div>

      <div className="border-b border-slate-200 bg-white px-3 py-2">
        <SummaryChips summary={summary} ikPulse={ikPulse} t={t} />
      </div>

      <div className="min-h-0 flex-1 overflow-auto bg-[#0f172a]">
        <pre className="m-0 grid grid-cols-[minmax(2.5rem,auto)_1fr] gap-x-0 font-mono text-[11px] leading-5 text-slate-100">
          {lines.map((line, i) => (
            <div key={i} className="contents">
              <span className="select-none border-r border-slate-700 bg-[#111827] px-2 text-right text-slate-500">
                {i + 1}
              </span>
              <code className="whitespace-pre-wrap break-all px-3 text-slate-100">
                {line || " "}
              </code>
            </div>
          ))}
        </pre>
      </div>
    </div>
  );
}
