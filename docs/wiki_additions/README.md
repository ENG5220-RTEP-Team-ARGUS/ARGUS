# ARGUS Wiki Additions (Non-Destructive)

Purpose:
- provide a more complete manual in the wiki
- keep `README.md` short and practical
- **build on top of the current wiki, not replace it**

## Usage Model
1. Keep your current wiki home page and existing pages.
2. Add links from home to the new pages listed below.
3. Copy these files into the wiki repository as new pages.
4. Only merge overlapping content where needed; do not delete existing pages unless they are confirmed stale.

## Suggested New Wiki Pages
- `Wiki-Home-Additions.md` (snippet to append to current Home page)
- `01-Setup-and-Build-Pi.md`
- `02-Hardware-Wiring-and-Power.md`
- `03-Run-Modes-and-Controls.md`
- `04-Latency-and-Validation.md`
- `05-Troubleshooting.md`

## Existing Wiki Protection Rules
- Do not overwrite existing page titles/content blindly.
- Prefer additive sections:
  - `## ARGUS v1.2 Additions`
  - `## Current validated baseline`
- Keep historical content if still useful; mark stale blocks with a short note rather than deleting immediately.

## Source of Truth
For technical truth, use code + scripts in this repository:
- `src/`
- `include/`
- `scripts/`
- `docs/compliance_matrix.md`
- `docs/adr/001-compliance-strategy.md`
