# Flash Drive Setup Notes

## Recommended drive formatting
- Best: **exFAT** (cross-platform Windows/macOS/Linux)
- Better-for-Linux: **ext4** (not Windows-friendly without extra drivers)

## Git portability tips
- Use LF line endings (repo enforces via .gitattributes)
- Avoid OS-specific symlinks
- Keep paths short and ASCII

## Suggested usage
- Keep this repo on the flash drive as the single source of truth.
- Clone/copy to a local SSD for heavy work, then push/pull changes back.
