import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "Yaskawa CAD/CAM Prototype",
  description: "Prototype welding path planner for Yaskawa AR/MA robots (DX200)",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
