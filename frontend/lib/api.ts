const BASE = process.env.NEXT_PUBLIC_API_URL ?? "http://localhost:8000";

export type TcpPoint = {
  x: number;
  y: number;
  z: number;
  rx?: number;
  ry?: number;
  rz?: number;
};

export async function calculateIk(points: TcpPoint[], qSeedDeg?: number[]) {
  const res = await fetch(`${BASE}/api/calculate-ik`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ points, q_seed_deg: qSeedDeg ?? null }),
  });
  if (!res.ok) throw new Error(`IK failed: ${res.status}`);
  return res.json();
}

export type FkPose = {
  x: number;
  y: number;
  z: number;
  rx: number;
  ry: number;
  rz: number;
};

/** FK bundle: flange / tool0 / tcp_0 for the current joint vector (deg). */
export async function forwardKinematics(qDeg: number[]) {
  const res = await fetch(`${BASE}/api/forward-kinematics`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(qDeg),
  });
  if (!res.ok) throw new Error(`FK failed: ${res.status}`);
  return res.json() as Promise<{
    flange: FkPose;
    tool0: FkPose;
    tcp_0: FkPose;
    tcp: FkPose;
  }>;
}

export async function analyzeCad(file: File) {
  const form = new FormData();
  form.append("file", file);
  const res = await fetch(`${BASE}/api/analyze-cad`, {
    method: "POST",
    body: form,
  });
  if (!res.ok) throw new Error(`CAD analysis failed: ${res.status}`);
  return res.json();
}

export async function generateJbi(body: unknown) {
  const res = await fetch(`${BASE}/api/generate-jbi`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(`JBI generation failed: ${res.status}`);
  return res.text();
}

export type HsesInfo = {
  provider: string;
  host: string | null;
  host_configured: boolean;
  dry_run: boolean;
  license_registered: boolean;
  sdk: {
    available: boolean;
    package?: string;
    runtime?: string;
    hint?: string | null;
    error?: string;
  };
  role?: string;
};

export async function hsesInfo() {
  const res = await fetch(`${BASE}/api/hses`);
  if (!res.ok) throw new Error(`HSES info failed: ${res.status}`);
  return res.json() as Promise<HsesInfo>;
}

export async function hsesStatus() {
  const res = await fetch(`${BASE}/api/hses/status`);
  if (!res.ok) {
    const t = await res.text();
    throw new Error(t || `HSES status failed: ${res.status}`);
  }
  return res.json() as Promise<Record<string, unknown>>;
}

export async function hsesUploadJbi(body: {
  jbi: string;
  job_name?: string;
  start?: boolean;
  line?: number;
}) {
  const res = await fetch(`${BASE}/api/hses/upload-jbi`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!res.ok) {
    const t = await res.text();
    throw new Error(t || `HSES upload failed: ${res.status}`);
  }
  return res.json();
}

export async function hsesStartJob(body: {
  job_name: string;
  line?: number;
  servo_on?: boolean;
}) {
  const res = await fetch(`${BASE}/api/hses/start-job`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!res.ok) {
    const t = await res.text();
    throw new Error(t || `HSES start failed: ${res.status}`);
  }
  return res.json();
}
