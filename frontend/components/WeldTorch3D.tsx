"use client";

import { useMemo, Suspense } from "react";
import { useLoader } from "@react-three/fiber";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { withBase } from "@/lib/models";
import {
  TORCH_STICK_OUT_M,
  TOOL_RPY_ORDER,
  tcpMetres,
  tcpRpyRad,
  type TorchCalibration,
} from "@/lib/tool0";
import {
  TORCH_ESAB_URL,
  TORCH_MESH_ALIGN_QUAT,
  TORCH_MESH_TIP_MM,
} from "@/lib/torchMesh";

const deg2rad = (d: number) => (d * Math.PI) / 180;

const ESAB_URL = withBase(TORCH_ESAB_URL);

// Warm the STL cache so Suspense resolves quickly inside the flange portal.
if (typeof window !== "undefined") {
  useLoader.preload(STLLoader, ESAB_URL);
}

/**
 * MIG torch in the Motoman flange / TOOL frame (+Z out of flange).
 * Default mesh: bundled ESAB `rm62_36deg.stl` with tip at calibrated TOOL 0 TCP.
 */
export default function WeldTorch3D({
  calib,
  arcOn = false,
}: {
  calib: TorchCalibration;
  arcOn?: boolean;
}) {
  return (
    <Suspense fallback={<ProceduralTorch calib={calib} arcOn={arcOn} />}>
      <TorchBody calib={calib} arcOn={arcOn} />
    </Suspense>
  );
}

function TorchBody({
  calib,
  arcOn,
}: {
  calib: TorchCalibration;
  arcOn: boolean;
}) {
  // Always load the vendor STL (cached) so hooks stay unconditional.
  const esabGeo = useLoader(STLLoader, ESAB_URL);

  if (calib.meshMode === "custom" && calib.customMeshUrl) {
    return (
      <group>
        <CustomTorchMesh calib={calib} />
        {calib.showAxes && <Axes calib={calib} />}
        {arcOn && <ArcGlow tcp={tcpMetres(calib)} />}
      </group>
    );
  }

  if (calib.meshMode === "procedural") {
    return <ProceduralTorch calib={calib} arcOn={arcOn} />;
  }

  return (
    <group>
      <VendorTorchAtTcp calib={calib} geometry={esabGeo} />
      {calib.showAxes && <Axes calib={calib} />}
      {arcOn && <ArcGlow tcp={tcpMetres(calib)} />}
    </group>
  );
}

/** Place the tip-at-origin ESAB mesh at the Motoman TOOL 0 pose. */
function VendorTorchAtTcp({
  calib,
  geometry,
}: {
  calib: TorchCalibration;
  geometry: THREE.BufferGeometry;
}) {
  const tcp = tcpMetres(calib);
  const [rx, ry, rz] = tcpRpyRad(calib);
  const quat = useMemo(
    () =>
      new THREE.Quaternion().setFromEuler(
        new THREE.Euler(rx, ry, rz, TOOL_RPY_ORDER),
      ),
    [rx, ry, rz],
  );

  // Clone once per loader geometry. Do NOT dispose in an effect — React
  // Strict Mode remounts would reuse a disposed BufferGeometry and the torch
  // disappears from the flange.
  const geo = useMemo(() => {
    const g = geometry.clone();
    g.computeVertexNormals();
    return g;
  }, [geometry]);

  const [tx, ty, tz] = TORCH_MESH_TIP_MM;
  const [qx, qy, qz, qw] = TORCH_MESH_ALIGN_QUAT;

  return (
    <group position={tcp} quaternion={quat}>
      {/* mm → m after tip→origin and wire→+Z (lib/torchMesh.ts). */}
      <group scale={[0.001, 0.001, 0.001]}>
        <group quaternion={[qx, qy, qz, qw]}>
          <mesh geometry={geo} position={[-tx, -ty, -tz]} castShadow receiveShadow>
            <meshStandardMaterial
              color={0x4a5563}
              metalness={0.5}
              roughness={0.4}
              side={THREE.DoubleSide}
            />
          </mesh>
        </group>
      </group>
      <mesh>
        <sphereGeometry args={[0.004, 14, 14]} />
        <meshStandardMaterial
          color="#ff7a1a"
          emissive="#ff6a00"
          emissiveIntensity={1.1}
        />
      </mesh>
    </group>
  );
}

function ProceduralTorch({
  calib,
  arcOn,
}: {
  calib: TorchCalibration;
  arcOn: boolean;
}) {
  const g = useMemo(() => buildTorchGeom(calib), [calib]);

  return (
    <group>
      <mesh position={[0, 0, 0.006]} rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[0.0375, 0.0375, 0.012, 28]} />
        <meshStandardMaterial color={0x2a3038} metalness={0.55} roughness={0.4} />
      </mesh>

      <mesh position={[0, 0, 0.07]} rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[0.028, 0.028, 0.14, 24]} />
        <meshStandardMaterial color={0x15191f} metalness={0.4} roughness={0.5} />
      </mesh>
      <mesh position={[0.03, 0, 0.05]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.008, 0.008, 0.04, 10]} />
        <meshStandardMaterial color={0x0d1116} roughness={0.85} />
      </mesh>

      <mesh position={[0, 0, 0.19]} rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[0.019, 0.019, 0.1, 20]} />
        <meshStandardMaterial color={0x1f2937} metalness={0.45} roughness={0.45} />
      </mesh>

      <mesh geometry={g.neckGeo} castShadow>
        <meshStandardMaterial color={0xc0c6ce} metalness={0.7} roughness={0.28} />
      </mesh>

      <mesh position={g.nozzleMid.toArray()} quaternion={g.wireQuat} castShadow>
        <cylinderGeometry args={[0.008, 0.012, 0.065, 16]} />
        <meshStandardMaterial color={0xb87333} metalness={0.8} roughness={0.3} />
      </mesh>

      <mesh position={g.wireMid.toArray()} quaternion={g.wireQuat}>
        <cylinderGeometry args={[0.0006, 0.0006, TORCH_STICK_OUT_M, 8]} />
        <meshStandardMaterial color={0xe8b060} metalness={0.85} roughness={0.2} />
      </mesh>

      <mesh position={g.tcp.toArray()}>
        <sphereGeometry args={[0.0035, 14, 14]} />
        <meshStandardMaterial
          color="#ff7a1a"
          emissive="#ff6a00"
          emissiveIntensity={1.1}
        />
      </mesh>

      {calib.showAxes && <Axes calib={calib} />}
      {arcOn && <ArcGlow tcp={g.tcp.toArray() as [number, number, number]} />}
    </group>
  );
}

function buildTorchGeom(calib: TorchCalibration) {
  const tcp = new THREE.Vector3(...tcpMetres(calib));
  const [rx, ry, rz] = tcpRpyRad(calib);
  const qTool = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(rx, ry, rz, TOOL_RPY_ORDER),
  );
  const wireDir = new THREE.Vector3(0, 0, 1).applyQuaternion(qTool).normalize();

  const stick = TORCH_STICK_OUT_M;
  const nozzleLen = 0.065;
  const nozzleTip = tcp.clone().addScaledVector(wireDir, -stick);
  const nozzleBase = nozzleTip.clone().addScaledVector(wireDir, -nozzleLen);
  const nozzleMid = nozzleBase.clone().addScaledVector(wireDir, nozzleLen / 2);
  const wireMid = tcp.clone().addScaledVector(wireDir, -stick / 2);

  const p2 = new THREE.Vector3(0, 0, 0.24);
  const p3 = nozzleBase.clone();
  const bend = p2
    .clone()
    .lerp(p3, 0.45)
    .add(new THREE.Vector3(-0.04, 0, 0.02));

  const curve = new THREE.CatmullRomCurve3([p2, bend, p3]);
  const neckGeo = new THREE.TubeGeometry(curve, 32, 0.01, 12, false);
  const yUp = new THREE.Vector3(0, 1, 0);
  const wireQuat = new THREE.Quaternion().setFromUnitVectors(yUp, wireDir);

  return { tcp, wireDir, nozzleMid, wireMid, neckGeo, wireQuat };
}

function Axes({ calib }: { calib: TorchCalibration }) {
  const tcp = tcpMetres(calib);
  const [rx, ry, rz] = tcpRpyRad(calib);
  const quat = useMemo(
    () =>
      new THREE.Quaternion().setFromEuler(
        new THREE.Euler(rx, ry, rz, TOOL_RPY_ORDER),
      ),
    [rx, ry, rz],
  );
  return (
    <group>
      <axesHelper args={[0.08]} />
      <group position={tcp} quaternion={quat}>
        <axesHelper args={[0.05]} />
      </group>
    </group>
  );
}

function ArcGlow({ tcp }: { tcp: [number, number, number] }) {
  return (
    <group position={tcp}>
      <mesh>
        <sphereGeometry args={[0.012, 12, 12]} />
        <meshStandardMaterial
          color="#fff2b0"
          emissive="#ff8a00"
          emissiveIntensity={2}
          transparent
          opacity={0.85}
        />
      </mesh>
      <pointLight color="#ff9a3c" intensity={2.5} distance={0.55} />
    </group>
  );
}

function CustomTorchMesh({ calib }: { calib: TorchCalibration }) {
  const url = calib.customMeshUrl!;
  const kind = calib.customMeshKind ?? "stl";
  if (kind === "glb") {
    return <GltbTorch url={url} calib={calib} />;
  }
  return <StlTorch url={url} calib={calib} />;
}

function StlTorch({ url, calib }: { url: string; calib: TorchCalibration }) {
  const geo = useLoader(STLLoader, url);
  const pos: [number, number, number] = [
    calib.customXmm / 1000,
    calib.customYmm / 1000,
    calib.customZmm / 1000,
  ];
  const rot: [number, number, number] = [
    deg2rad(calib.customRxDeg),
    deg2rad(calib.customRyDeg),
    deg2rad(calib.customRzDeg),
  ];
  return (
    <group position={pos} rotation={rot} scale={calib.customScale}>
      <mesh geometry={geo} castShadow>
        <meshStandardMaterial color={0x3a4553} metalness={0.45} roughness={0.45} />
      </mesh>
      <mesh position={tcpMetres(calib)}>
        <sphereGeometry args={[0.0035, 12, 12]} />
        <meshStandardMaterial color="#ff7a1a" emissive="#ff6a00" emissiveIntensity={1} />
      </mesh>
    </group>
  );
}

function GltbTorch({ url, calib }: { url: string; calib: TorchCalibration }) {
  const gltf = useLoader(GLTFLoader, url);
  const pos: [number, number, number] = [
    calib.customXmm / 1000,
    calib.customYmm / 1000,
    calib.customZmm / 1000,
  ];
  const rot: [number, number, number] = [
    deg2rad(calib.customRxDeg),
    deg2rad(calib.customRyDeg),
    deg2rad(calib.customRzDeg),
  ];
  return (
    <group position={pos} rotation={rot} scale={calib.customScale}>
      <primitive object={gltf.scene.clone()} />
      <mesh position={tcpMetres(calib)}>
        <sphereGeometry args={[0.0035, 12, 12]} />
        <meshStandardMaterial color="#ff7a1a" emissive="#ff6a00" emissiveIntensity={1} />
      </mesh>
    </group>
  );
}
