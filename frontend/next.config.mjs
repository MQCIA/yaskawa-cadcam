/** @type {import('next').NextConfig} */

// When deploying to GitHub Pages (a project site served from a sub-path),
// set NEXT_PUBLIC_BASE_PATH=/yaskawa-cadcam and STATIC_EXPORT=1.
const basePath = process.env.NEXT_PUBLIC_BASE_PATH || "";
const staticExport = process.env.STATIC_EXPORT === "1";

const nextConfig = {
  reactStrictMode: true,
  ...(staticExport ? { output: "export" } : {}),
  basePath: basePath || undefined,
  images: { unoptimized: true },
};

export default nextConfig;
