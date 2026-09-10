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
