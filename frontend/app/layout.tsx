import type { Metadata } from "next";
import "./globals.css";
import Providers from "@/components/Providers";

export const metadata: Metadata = {
  title: "Yaskawa Welding Navigator",
  description:
    "Prototypowy planer ścieżek spawania dla robotów Yaskawa AR (DX200) — UI w stylu Verbotics Weld",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="pl">
      <body>
        <Providers>{children}</Providers>
      </body>
    </html>
  );
}
