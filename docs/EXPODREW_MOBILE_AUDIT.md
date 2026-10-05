# Expodrew mobile audit — Yaskawa CAD/CAM

Applied checklist from `.cursor/skills/expodrew-local-site-audit/SKILL.md`
(ported from `MQCIA/strona_expodrew` mobile-responsive + local-site-audit work).

## Findings → fixes

| Check | Before | After |
|-------|--------|-------|
| Pinch-zoom | Locked / missing viewport export | Allowed (`viewportFit: cover`) |
| Phone layout | Side docks crushed 3D stage | Full-bleed 3D + Cell/Robot bottom sheets |
| Touch targets | Mixed / &lt;40px | ≥40px on mobile nav, sheet close, jog ±, ranges |
| Inputs | Could be &lt;16px | `font-size: 16px` on inputs/selects |
| Sheet dismiss | — | Backdrop + ESC + close; body scroll locked |
| Focus / motion | Weak | `:focus-visible` + `prefers-reduced-motion` |
| Responsive CSS | Utilities only | End-of-file responsive layer in `globals.css` |
| 3D gestures | Mouse-only | OrbitControls: 1 finger rotate, 2 finger pinch/pan |

## Verify

- Desktop (≥1024): Verbotics 3-column layout unchanged.
- Phone (~390): Preview | Cell | Robot bottom nav; 3D keeps full width.
