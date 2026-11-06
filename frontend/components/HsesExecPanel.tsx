"use client";

import { useCallback, useEffect, useState } from "react";
import {
  hsesInfo,
  hsesStartJob,
  hsesStatus,
  hsesUploadJbi,
  type HsesInfo,
} from "@/lib/api";

type Labels = {
  title: string;
  summary: string;
  host: string;
  refresh: string;
  upload: string;
  uploadStart: string;
  startOnly: string;
  jobName: string;
  dryRun: string;
  sdkOk: string;
  sdkMissing: string;
  needJbi: string;
};

export default function HsesExecPanel({
  jbiText,
  defaultJobName = "CADCAM",
  labels,
}: {
  jbiText: string | null;
  defaultJobName?: string;
  labels: Labels;
}) {
  const [info, setInfo] = useState<HsesInfo | null>(null);
  const [status, setStatus] = useState<Record<string, unknown> | null>(null);
  const [jobName, setJobName] = useState(defaultJobName);
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);

  const refresh = useCallback(async () => {
    try {
      const i = await hsesInfo();
      setInfo(i);
      setMsg(null);
    } catch (e) {
      setMsg((e as Error).message);
    }
  }, []);

  useEffect(() => {
    void refresh();
  }, [refresh]);

  async function onStatus() {
    setBusy(true);
    try {
      setStatus(await hsesStatus());
      setMsg(null);
    } catch (e) {
      setMsg((e as Error).message);
    } finally {
      setBusy(false);
    }
  }

  async function onUpload(start: boolean) {
    if (!jbiText) {
      setMsg(labels.needJbi);
      return;
    }
    setBusy(true);
    try {
      const res = await hsesUploadJbi({
        jbi: jbiText,
        job_name: jobName,
        start,
      });
      setMsg(JSON.stringify(res));
      await refresh();
    } catch (e) {
      setMsg((e as Error).message);
    } finally {
      setBusy(false);
    }
  }

  async function onStartOnly() {
    setBusy(true);
    try {
      const res = await hsesStartJob({ job_name: jobName });
      setMsg(JSON.stringify(res));
    } catch (e) {
      setMsg((e as Error).message);
    } finally {
      setBusy(false);
    }
  }

  const sdkOk = Boolean(info?.sdk?.available) || Boolean(info?.dry_run);

  return (
    <details className="mt-3 rounded border border-slate-200 bg-slate-50 open:bg-white">
      <summary className="cursor-pointer select-none px-2 py-2 text-[11px] font-semibold uppercase tracking-wide text-slate-700">
        {labels.title}
      </summary>
      <div className="space-y-2 border-t border-slate-200 px-2 py-2 text-[11px] text-slate-700">
        <p className="text-[10px] leading-snug text-slate-500">{labels.summary}</p>
        <div
          className={`rounded px-2 py-1.5 text-[10px] font-medium ${
            sdkOk ? "bg-emerald-50 text-emerald-800" : "bg-amber-50 text-amber-900"
          }`}
        >
          {info?.dry_run
            ? labels.dryRun
            : info?.sdk?.available
              ? labels.sdkOk
              : labels.sdkMissing}
          {info?.host ? (
            <div className="mt-0.5 font-mono opacity-90">
              {labels.host}: {info.host}
            </div>
          ) : null}
          {info?.sdk?.hint && !info.sdk.available ? (
            <div className="mt-1 font-mono text-[9px] opacity-80">{info.sdk.hint}</div>
          ) : null}
        </div>

        <label className="block text-[10px] text-slate-500">
          {labels.jobName}
          <input
            value={jobName}
            onChange={(e) => setJobName(e.target.value)}
            className="mt-0.5 w-full rounded border border-slate-200 bg-white px-2 py-1 font-mono text-xs text-slate-800"
          />
        </label>

        <div className="flex flex-wrap gap-1.5">
          <button
            type="button"
            disabled={busy}
            onClick={() => void refresh()}
            className="rounded border border-slate-200 bg-white px-2 py-1 text-[10px] hover:bg-slate-50 disabled:opacity-40"
          >
            {labels.refresh}
          </button>
          <button
            type="button"
            disabled={busy}
            onClick={() => void onStatus()}
            className="rounded border border-slate-200 bg-white px-2 py-1 text-[10px] hover:bg-slate-50 disabled:opacity-40"
          >
            Status
          </button>
          <button
            type="button"
            disabled={busy || !jbiText}
            onClick={() => void onUpload(false)}
            className="rounded border border-slate-200 bg-white px-2 py-1 text-[10px] hover:bg-slate-50 disabled:opacity-40"
          >
            {labels.upload}
          </button>
          <button
            type="button"
            disabled={busy || !jbiText}
            onClick={() => void onUpload(true)}
            className="rounded bg-[#e87722] px-2 py-1 text-[10px] font-semibold text-white hover:brightness-110 disabled:opacity-40"
          >
            {labels.uploadStart}
          </button>
          <button
            type="button"
            disabled={busy}
            onClick={() => void onStartOnly()}
            className="rounded border border-slate-200 bg-white px-2 py-1 text-[10px] hover:bg-slate-50 disabled:opacity-40"
          >
            {labels.startOnly}
          </button>
        </div>

        {status && (
          <pre className="max-h-24 overflow-auto rounded bg-slate-100 p-1.5 font-mono text-[9px] text-slate-700">
            {JSON.stringify(status, null, 2)}
          </pre>
        )}
        {msg && (
          <pre className="max-h-28 overflow-auto rounded bg-slate-100 p-1.5 font-mono text-[9px] text-slate-700">
            {msg}
          </pre>
        )}
      </div>
    </details>
  );
}
