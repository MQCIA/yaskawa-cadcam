# Expodrew mobile audit — Yaskawa CAD/CAM

Applied checklist from `.cursor/skills/expodrew-local-site-audit/SKILL.md`
(ported from `MQCIA/strona_expodrew` mobile-responsive + local-site-audit work).

## Findings → fixes

| Check | Before | After |
|-------|--------|-------|
| Pinch-zoom | Locked (`maximumScale:1`, `userScalable:false`) | Allowed |
| Phone layout | Side docks crushed 3D stage (prior PR) | Full-bleed 3D + Cell/Robot bottom sheets |
| Touch targets | Mixed | ≥40px on mobile nav / sheet close / sheet controls |
| Inputs | Could be &lt;16px | `font-size: 16px` on inputs/selects |
| Sheet dismiss | Backdrop only | Backdrop + ESC + close |
| Focus / motion | Weak | `:focus-visible` + `prefers-reduced-motion` |
| Responsive CSS | Utilities only | End-of-file responsive layer in `globals.css` |

## Verify

- Desktop (≥1024): Verbotics 3-column layout unchanged.
- Phone (~390): Preview | Cell | Robot bottom nav; 3D keeps full width.
