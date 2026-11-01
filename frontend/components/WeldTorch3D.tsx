"use client";

import { useMemo, Suspense } from "react";
import { useLoader } from "@react-three/fiber";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import {
  TORCH_STICK_OUT_M,
  tcpMetres,
  tcpRpyRad,
  type TorchCalibration,
} from "@/lib/tool0";

const deg2rad = (d: number) => (d * Math.PI) / 180;

/**
 * CAD-like MIG torch in the Motoman flange / TOOL frame (+Z out of flange).
 * Wire tip ends exactly at calibrated TOOL 0 TCP.
 */
export default function WeldTorch3D({
  calib,
  arcOn = false,
}: {
  calib: TorchCalibration;
  arcOn?: boolean;
}) {
  if (calib.meshMode === "custom" && calib.customMeshUrl) {
    return (
      <group>
        <Suspense fallback={<ProceduralTorch calib={calib} arcOn={arcOn} />}>
          <CustomTorchMesh calib={calib} />
        </Suspense>
        {calib.showAxes && <Axes gizmos calib={calib} />}
        {arcOn && <ArcGlow tcp={tcpMetres(calib)} />}
      </group>
    );
  }
  return <ProceduralTorch calib={calib} arcOn={arcOn} />;
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
      {/* Flange adapter plate */}
      <mesh position={[0, 0, 0.006]} rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[0.0375, 0.0375, 0.012, 28]} />
        <meshStandardMaterial color={0x2a3038} metalness={0.55} roughness={0.4} />
      </mesh>

      {/* Shock-sensor / anti-collision body Ø56 × 140 mm along +Z */}
      <mesh position={[0, 0, 0.07]} rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[0.028, 0.028, 0.14, 24]} />
        <meshStandardMaterial color={0x15191f} metalness={0.4} roughness={0.5} />
      </mesh>
      {/* Cable stub */}
      <mesh position={[0.03, 0, 0.05]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.008, 0.008, 0.04, 10]} />
        <meshStandardMaterial color={0x0d1116} roughness={0.85} />
      </mesh>

      {/* Torch body Ø38 from z=0.14 → 0.24 */}
      <mesh position={[0, 0, 0.19]} rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[0.019, 0.019, 0.1, 20]} />
        <meshStandardMaterial color={0x1f2937} metalness={0.45} roughness={0.45} />
      </mesh>

      {/* Swan-neck TubeGeometry */}
      <mesh geometry={g.neckGeo} castShadow>
        <meshStandardMaterial color={0xc0c6ce} metalness={0.7} roughness={0.28} />
      </mesh>

      {/* Gas nozzle — copper frustum along tool Z */}
      <mesh position={g.nozzleMid.toArray()} quaternion={g.wireQuat} castShadow>
        <cylinderGeometry args={[0.008, 0.012, 0.065, 16]} />
        <meshStandardMaterial color={0xb87333} metalness={0.8} roughness={0.3} />
      </mesh>

      {/* Contact tip + stick-out wire ending at TCP */}
      <mesh position={g.wireMid.toArray()} quaternion={g.wireQuat}>
        <cylinderGeometry args={[0.0006, 0.0006, TORCH_STICK_OUT_M, 8]} />
        <meshStandardMaterial color={0xe8b060} metalness={0.85} roughness={0.2} />
      </mesh>

      {/* TCP gizmo */}
      <mesh position={g.tcp.toArray()}>
        <sphereGeometry args={[0.0035, 14, 14]} />
        <meshStandardMaterial
          color="#ff7a1a"
          emissive="#ff6a00"
          emissiveIntensity={1.1}
        />
      </mesh>

      {calib.showAxes && <Axes gizmos calib={calib} />}
      {arcOn && <ArcGlow tcp={g.tcp.toArray() as [number, number, number]} />}
    </group>
  );
}

function buildTorchGeom(calib: TorchCalibration) {
  const tcp = new THREE.Vector3(...tcpMetres(calib));
  const [rx, ry, rz] = tcpRpyRad(calib);
  const qTool = new THREE.Quaternion().setFromEuler(new THREE.Euler(rx, ry, rz, "XYZ"));
  const wireDir = new THREE.Vector3(0, 0, 1).applyQuaternion(qTool).normalize();

  const stick = TORCH_STICK_OUT_M;
  const nozzleLen = 0.065;
  // Nozzle ends stick-out before TCP; nozzle base further along −wireDir.
  const nozzleTip = tcp.clone().addScaledVector(wireDir, -stick);
  const nozzleBase = nozzleTip.clone().addScaledVector(wireDir, -nozzleLen);
  const nozzleMid = nozzleBase.clone().addScaledVector(wireDir, nozzleLen / 2);
  const wireMid = tcp.clone().addScaledVector(wireDir, -stick / 2);

  const p0 = new THREE.Vector3(0, 0, 0);
  const p1 = new THREE.Vector3(0, 0, 0.14);
  const p2 = new THREE.Vector3(0, 0, 0.24);
  // Swan-neck ends at nozzle base (p3 in the brief).
  const p3 = nozzleBase.clone();
  // Mid bend control — bias toward −X like Motoman Ry≈−42.6°.
  const bend = p2
    .clone()
    .lerp(p3, 0.45)
    .add(new THREE.Vector3(-0.04, 0, 0.02));

  const curve = new THREE.CatmullRomCurve3([p2, bend, p3]);
  const neckGeo = new THREE.TubeGeometry(curve, 32, 0.01, 12, false);

  const yUp = new THREE.Vector3(0, 1, 0);
  const wireQuat = new THREE.Quaternion().setFromUnitVectors(yUp, wireDir);

  return {
    tcp,
    wireDir,
    nozzleMid,
    wireMid,
    neckGeo,
    wireQuat,
    path: [p0, p1, p2, p3],
  };
}

function Axes({
  calib,
}: {
  gizmos?: boolean;
  calib: TorchCalibration;
}) {
  const tcp = tcpMetres(calib);
  const [rx, ry, rz] = tcpRpyRad(calib);
  return (
    <group>
      <axesHelper args={[0.08]} />
      <group position={tcp} rotation={[rx, ry, rz]}>
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
