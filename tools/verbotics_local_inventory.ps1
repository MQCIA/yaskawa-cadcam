# Run on the Windows PC that has Verbotics Weld 2026.
# Writes frontend/public/models/from-verbotics/inventory.json for a cloud agent.
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) { $py = Get-Command python3 -ErrorAction SilentlyContinue }
if (-not $py) { throw "Python not found. Install Python 3 and retry." }

$install = "C:\Program Files\Verbotics Weld 2026"
$argsList = @(
    (Join-Path $PSScriptRoot "verbotics_local_inventory.py"),
    "--extract-json"
)
if (Test-Path $install) {
    $argsList += @("--root", $install)
}

& $py.Source @argsList
Write-Host "Next: commit frontend/public/models/from-verbotics/ (inventory.json + extracted-json) and tell the cloud agent."
