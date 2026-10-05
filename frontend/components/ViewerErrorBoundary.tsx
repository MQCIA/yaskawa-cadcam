"use client";

import { Component, type ReactNode } from "react";

type Props = { children: ReactNode; fallback?: ReactNode };
type State = { error: Error | null };

/**
 * Isolates WebGL / R3F failures so the mobile chrome (bottom nav, sheets)
 * stays usable when a device cannot create a GL context.
 */
export default class ViewerErrorBoundary extends Component<Props, State> {
  state: State = { error: null };

  static getDerivedStateFromError(error: Error): State {
    return { error };
  }

  render() {
    if (this.state.error) {
      if (this.props.fallback) return this.props.fallback;
      return (
        <div className="flex h-full w-full flex-col items-center justify-center gap-2 bg-slate-50 px-4 text-center text-sm text-slate-600">
          <p className="font-semibold text-slate-800">Podgląd 3D niedostępny</p>
          <p className="max-w-sm text-xs leading-relaxed text-slate-500">
            Ta przeglądarka nie utworzyła kontekstu WebGL. Otwórz aplikację w Chrome /
            Safari na urządzeniu z GPU albo użyj pulpitu.
          </p>
          <button
            type="button"
            className="mobile-touch mt-2 rounded border border-slate-300 bg-white px-3 text-xs font-semibold text-[#e87722]"
            onClick={() => this.setState({ error: null })}
          >
            Spróbuj ponownie
          </button>
        </div>
      );
    }
    return this.props.children;
  }
}
