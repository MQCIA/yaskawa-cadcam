# Full Verbotics Weld dump for the self-hosted Windows worker (verbotics-pc).
# Writes frontend/public/models/from-verbotics/{inventory.json,SUMMARY.md,extracted-json/,meshes/}
$ErrorActionPreference = "Continue"
$repo = Split-Path -Parent $PSScriptRoot
$outDir = Join-Path $repo "frontend\public\models\from-verbotics"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) { $py = Get-Command python3 -ErrorAction SilentlyContinue }
if (-not $py) { throw "Python not found. Install Python 3 and retry." }

function Find-VerboticsRoots {
    $roots = New-Object System.Collections.Generic.List[string]
    $candidates = @(
        "${env:ProgramFiles}\Verbotics Weld 2026",
        "${env:ProgramFiles}\Verbotics Weld",
        "${env:ProgramFiles}\Verbotics",
        "${env:ProgramFiles}\Verbotics Cell Editor 2026",
        "${env:ProgramFiles}\Verbotics Cell Editor",
        "${env:ProgramFiles(x86)}\Verbotics Weld 2026",
        "${env:ProgramFiles(x86)}\Verbotics Weld",
        "${env:ProgramFiles(x86)}\Verbotics",
        "$env:LOCALAPPDATA\Verbotics",
        "$env:LOCALAPPDATA\Verbotics Weld",
        "$env:LOCALAPPDATA\VerboticsWeld",
        "$env:APPDATA\Verbotics",
        "$env:APPDATA\Verbotics Weld",
        "$env:USERPROFILE\Documents\Verbotics",
        "$env:USERPROFILE\Documents\Verbotics Weld",
        "$env:USERPROFILE\Documents\VerboticsWeld"
    )
    foreach ($c in $candidates) {
        if ($c -and (Test-Path -LiteralPath $c)) { [void]$roots.Add($c) }
    }

    foreach ($base in @($env:ProgramFiles, ${env:ProgramFiles(x86)}, $env:LOCALAPPDATA, $env:APPDATA, "$env:USERPROFILE\Documents")) {
        if (-not $base -or -not (Test-Path -LiteralPath $base)) { continue }
        Get-ChildItem -LiteralPath $base -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match 'verbotic' } |
            ForEach-Object { [void]$roots.Add($_.FullName) }
    }

    # Uninstall registry -> InstallLocation / DisplayIcon
    $uninstallKeys = @(
        "HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*",
        "HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*",
        "HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*"
    )
    foreach ($pattern in $uninstallKeys) {
        Get-ItemProperty $pattern -ErrorAction SilentlyContinue |
            Where-Object { $_.DisplayName -match 'verbotic' } |
            ForEach-Object {
                if ($_.InstallLocation -and (Test-Path $_.InstallLocation)) {
                    [void]$roots.Add($_.InstallLocation.TrimEnd('\'))
                }
                if ($_.DisplayIcon) {
                    $icon = ($_.DisplayIcon -split ',')[0].Trim('"')
                    if (Test-Path -LiteralPath $icon) {
                        [void]$roots.Add((Split-Path -Parent $icon))
                    }
                }
            }
    }

    # Start Menu shortcuts
    $smRoots = @(
        "$env:ProgramData\Microsoft\Windows\Start Menu\Programs",
        "$env:APPDATA\Microsoft\Windows\Start Menu\Programs"
    )
    foreach ($sm in $smRoots) {
        if (-not (Test-Path $sm)) { continue }
        Get-ChildItem -Path $sm -Recurse -Filter *.lnk -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match 'verbotic' -or $_.DirectoryName -match 'verbotic' } |
            ForEach-Object {
                try {
                    $sh = New-Object -ComObject WScript.Shell
                    $target = $sh.CreateShortcut($_.FullName).TargetPath
                    if ($target -and (Test-Path -LiteralPath $target)) {
                        [void]$roots.Add((Split-Path -Parent $target))
                    }
                } catch {}
            }
    }

    $roots | Select-Object -Unique
}

function Get-FileVersions([string[]]$Roots) {
    $versions = @()
    foreach ($root in $Roots) {
        if (-not (Test-Path -LiteralPath $root)) { continue }
        Get-ChildItem -LiteralPath $root -Recurse -Include *.exe,*.dll -ErrorAction SilentlyContinue |
            Where-Object {
                $_.Name -match 'verbotic|weld|cell|editor' -or
                $_.DirectoryName -match '\\bin($|\\)' -or
                $_.Directory.Name -eq (Split-Path $root -Leaf)
            } |
            Select-Object -First 120 |
            ForEach-Object {
                try {
                    $vi = [System.Diagnostics.FileVersionInfo]::GetVersionInfo($_.FullName)
                    $versions += [ordered]@{
                        path           = $_.FullName
                        size           = $_.Length
                        FileVersion    = $vi.FileVersion
                        ProductVersion = $vi.ProductVersion
                        ProductName    = $vi.ProductName
                        CompanyName    = $vi.CompanyName
                        FileDescription = $vi.FileDescription
                    }
                } catch {
                    $versions += [ordered]@{ path = $_.FullName; size = $_.Length; error = "$_" }
                }
            }
    }
    $versions
}

Write-Host "Discovering Verbotics install roots..."
$found = @(Find-VerboticsRoots)
Write-Host ("Found {0} root(s):" -f $found.Count)
$found | ForEach-Object { Write-Host "  $_" }

$versionsPath = Join-Path $outDir "exe-versions.json"
$versions = @(Get-FileVersions $found)
$versions | ConvertTo-Json -Depth 6 | Set-Content -Encoding UTF8 -Path $versionsPath
Write-Host "Wrote $versionsPath ($($versions.Count) entries)"

$argsList = @(
    (Join-Path $PSScriptRoot "verbotics_full_dump.py"),
    "--copy-torch-meshes",
    "--versions-json", $versionsPath
)
foreach ($r in $found) {
    $argsList += @("--root", $r)
}

Write-Host "Running: $($py.Source) $($argsList -join ' ')"
& $py.Source @argsList
$code = $LASTEXITCODE
Write-Host "Exit code: $code"
Write-Host "Outputs under: $outDir"
exit $code
