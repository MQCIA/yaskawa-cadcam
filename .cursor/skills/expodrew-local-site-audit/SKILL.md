---
name: expodrew-local-site-audit
description: >-
  Local site / mobile-responsive audit checklist ported from MQCIA/strona_expodrew
  (commits: responsive layer, mobile drawer, pinch-zoom, touch targets). Use when
  auditing or fixing phone/tablet layout for any MQCIA web UI (Expodrew storefront
  or Yaskawa CAD/CAM), or when the user mentions Expodrew skill, mobile audit,
  or "czy działa na telefonie".
---

# Expodrew local site audit (mobile)

Source of truth: `MQCIA/strona_expodrew` branch `cursor/mobile-responsive-fixes-4a05`
(`fix(css): responsive layer…`, `feat(header): mobile menu drawer…`) and the
local-site-audit commit (`Make enquiries honest and apply the local site audit`).

Breakpoints (match Tailwind `lg` / `sm`):

| Name | Width |
|------|-------|
| `lg` | 1024px |
| `sm` | 640px |

## Checklist — run and fix automatically

### 1. Viewport / zoom
- [ ] `viewport` allows pinch-zoom (`userScalable` true or omit `maximumScale: 1`)
- [ ] `viewportFit: cover` + safe-area padding on fixed chrome
- [ ] `100dvh` (not only `100vh`) for phone browser chrome

### 2. Layout on phones (&lt; lg)
- [ ] No forced multi-column docks that crush the main stage to ~0 width
- [ ] Primary content (3D / hero / gallery) is full viewport width
- [ ] Side panels become **bottom sheets** or a **drawer**, not persistent columns
- [ ] Sticky/fixed header has a mobile nav replacement (tabs or hamburger)
- [ ] End-of-file **responsive CSS layer** so later base rules cannot override `@media`

### 3. Touch
- [ ] Interactive targets ≥ **40px / 2.5rem** min-height (nav, close, dots, links)
- [ ] Form `input` / `select` / `textarea` font-size ≥ **16px** (stops iOS focus zoom)
- [ ] `touch-action` does not block pinch where zoom is allowed

### 4. Sheets / drawers
- [ ] Backdrop click closes
- [ ] ESC closes
- [ ] Visible close control ≥ 40px
- [ ] Body scroll locked while open (or sheet scrolls internally)
- [ ] Labels / Html overlays in 3D do not paint over open sheets

### 5. A11y / honesty (from Expodrew local audit)
- [ ] Visible `:focus-visible` rings on controls
- [ ] `prefers-reduced-motion: reduce` respected for non-essential motion
- [ ] Status / empty states tell the truth (no fake “sent” without a send)

### 6. Verify
- [ ] Screenshot ~390×844 (phone) and ~1440×900 (desktop)
- [ ] Phone: main stage visible; docks via bottom nav
- [ ] Desktop: 3-column Verbotics layout unchanged

## Apply to this CAD/CAM app

- Desktop: left dock + 3D + right Robot jog (Verbotics-style).
- Phone (`&lt; lg`): full-bleed 3D + bottom nav `Preview | Cell | Robot` opening sheets.
- Keep ribbon/planner strip desktop-only or horizontally scrollable — never squeeze the canvas.
