"use client";

type JbiEditorPanelProps = {
  text: string;
  dirty: boolean;
  disabled?: boolean;
  onChange: (text: string) => void;
  onApply: () => void;
  onReloadFromProgram: () => void;
  onDownload: () => void;
  labels: {
    title: string;
    hint: string;
    apply: string;
    reload: string;
    download: string;
    dirty: string;
    empty: string;
  };
};

export default function JbiEditorPanel({
  text,
  dirty,
  disabled,
  onChange,
  onApply,
  onReloadFromProgram,
  onDownload,
  labels,
}: JbiEditorPanelProps) {
  if (!text && disabled) {
    return <p className="px-1 text-xs text-slate-500">{labels.empty}</p>;
  }

  return (
    <div className="flex h-full min-h-0 flex-col gap-2">
      <div className="flex flex-wrap items-center justify-between gap-2">
        <div>
          <div className="text-[10px] font-semibold uppercase tracking-wide text-slate-600">
            {labels.title}
          </div>
          <div className="text-[10px] text-slate-500">{labels.hint}</div>
        </div>
        <div className="flex flex-wrap gap-1">
          <button
            type="button"
            disabled={disabled || !dirty}
            onClick={onApply}
            className="rounded bg-[#e87722] px-2.5 py-1.5 text-[11px] font-semibold text-white disabled:opacity-40"
          >
            {labels.apply}
          </button>
          <button
            type="button"
            disabled={disabled}
            onClick={onReloadFromProgram}
            className="rounded border border-slate-300 bg-white px-2.5 py-1.5 text-[11px] text-slate-700 disabled:opacity-40"
          >
            {labels.reload}
          </button>
          <button
            type="button"
            disabled={disabled || !text}
            onClick={onDownload}
            className="rounded border border-slate-300 bg-white px-2.5 py-1.5 text-[11px] text-slate-700 disabled:opacity-40"
          >
            {labels.download}
          </button>
        </div>
      </div>
      {dirty && (
        <div className="rounded border border-amber-200 bg-amber-50 px-2 py-1 text-[10px] text-amber-800">
          {labels.dirty}
        </div>
      )}
      <textarea
        value={text}
        disabled={disabled}
        spellCheck={false}
        onChange={(e) => onChange(e.target.value)}
        className="min-h-[16rem] w-full flex-1 resize-y rounded border border-slate-200 bg-[#0f172a] p-3 font-mono text-[11px] leading-relaxed text-emerald-300 outline-none focus:border-[#e87722] disabled:opacity-50"
      />
    </div>
  );
}
