"use client";

import { useEffect, useMemo } from "react";
import { useLoader } from "@react-three/fiber";
import * as THREE from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";

const deg2rad = (d: number) => (d * Math.PI) / 180;

/**
 * Loads a ROS-Industrial-style URDF (relative STL mesh paths) and drives its
 * joints from a name->degrees map. URDF is Z-up in metres; we rotate -90° about
 * X so it stands upright in the Y-up scene.
 */
export default function UrdfModel({
  url,
  jointValuesDeg,
  position = [0, 0, 0],
  color = 0x2e5cb8,
}: {
  url: string;
  jointValuesDeg: Record<string, number>;
  position?: [number, number, number];
  color?: number;
}) {
  const robot = useLoader(
    URDFLoader as unknown as new () => THREE.Loader,
    url,
    (loader) => {
      const urdfLoader = loader as unknown as InstanceType<typeof URDFLoader>;
      urdfLoader.loadMeshCb = (path, manager, material, done) => {
        new STLLoader(manager).load(
          path,
          (geometry) => {
            const mat = new THREE.MeshStandardMaterial({
              color,
              metalness: 0.25,
              roughness: 0.6,
            });
            const mesh = new THREE.Mesh(geometry, mat);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            done(mesh);
          },
          undefined,
          (err) => done(undefined as unknown as THREE.Object3D, err as Error),
        );
      };
    },
  ) as unknown as URDFRobot;

  // Clone so multiple instances (e.g. robot + positioner) don't share state.
  const instance = useMemo(() => robot.clone() as URDFRobot, [robot]);

  useEffect(() => {
    for (const [name, deg] of Object.entries(jointValuesDeg)) {
      if (instance.joints[name]) {
        instance.setJointValue(name, deg2rad(deg));
      }
    }
  }, [instance, jointValuesDeg]);

  return (
    <group position={position} rotation={[-Math.PI / 2, 0, 0]}>
      <primitive object={instance} />
    </group>
  );
}
