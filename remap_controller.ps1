# Kirby Super Star Recompiled - Controller Remapping Utility
# Interactive preset selector and controller customizer
param([string]$Preset = "")

$ErrorActionPreference = "Stop"
$jsonPath = Join-Path $PSScriptRoot "launch_settings.json"

function Get-CurrentPresetName {
    if (-not (Test-Path $jsonPath)) { return "Xbox Standard (Preset 0)" }
    $content = Get-Content $jsonPath -Raw
    if ($content -match '"preset":\s*1') { return "Nintendo Switch / Japanese (Preset 1)" }
    if ($content -match '"preset":\s*2') { return "Trigger Action (Preset 2: RT=Attack, LT=Jump)" }
    if ($content -match '"preset":\s*3') { return "Classic 1:1 Hardware (Preset 3)" }
    return "Xbox Standard (Preset 0: A=Jump, B=Attack, LT/RT=Guard)"
}

function Set-Preset($presetNum) {
    if (-not (Test-Path $jsonPath)) {
        @{
            fullscreen = $true
            bezel = $true
            preset = [int]$presetNum
            native_widescreen = $true
            aspect_ratio = "16:9"
            aspect_mode = "16:9"
            scale = 3
            upscaling_filter = "nearest"
            gamepad_enabled = $true
            deadzone = 7849
        } | ConvertTo-Json | Set-Content $jsonPath
        return
    }

    $raw = Get-Content $jsonPath -Raw
    if ($raw -match '"preset":\s*\d+') {
        $updated = $raw -replace '"preset":\s*\d+', "`"preset`": $presetNum"
    } else {
        $updated = $raw -replace '(?<=\{)', "`n  `"preset`": $presetNum,"
    }
    Set-Content -Path $jsonPath -Value $updated -NoNewline
}

Clear-Host
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "   Kirby Super Star Recompiled - Controller Configuration" -ForegroundColor Yellow
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Current Active Preset: $(Get-CurrentPresetName)" -ForegroundColor Green
Write-Host ""
Write-Host "Select an optimized gamepad layout preset:" -ForegroundColor White
Write-Host ""
Write-Host "  [1] Xbox Standard (Recommended)" -ForegroundColor Yellow
Write-Host "      - A = Jump / Float (SNES B)"
Write-Host "      - B = Attack / Inhale / Eat (SNES A)"
Write-Host "      - X = Dash / Secondary (SNES Y)"
Write-Host "      - Y = Ability Discard (SNES X)"
Write-Host "      - LB / LT = Guard (SNES L)"
Write-Host "      - RB / RT = Guard (SNES R)"
Write-Host "      - Left Stick / D-Pad = 8-Way Movement"
Write-Host ""
Write-Host "  [2] Nintendo Switch Layout (Position Matched)" -ForegroundColor Yellow
Write-Host "      - B = Jump, A = Attack, Y = Dash, X = Discard"
Write-Host ""
Write-Host "  [3] Trigger Action Layout" -ForegroundColor Yellow
Write-Host "      - RT = Attack / Inhale, LT = Jump"
Write-Host "      - A = Jump, B = Attack"
Write-Host ""
Write-Host "  [4] Classic 1:1 SNES Hardware Layout" -ForegroundColor Yellow
Write-Host "      - Direct literal 1:1 button matching"
Write-Host ""
Write-Host "  [Q] Exit without changing" -ForegroundColor Gray
Write-Host ""

$choice = Read-Host "Enter selection [1-4 or Q]"
switch ($choice) {
    "1" { Set-Preset 0; Write-Host "`nPreset 1 (Xbox Standard) applied successfully!" -ForegroundColor Green }
    "2" { Set-Preset 1; Write-Host "`nPreset 2 (Nintendo Switch) applied successfully!" -ForegroundColor Green }
    "3" { Set-Preset 2; Write-Host "`nPreset 3 (Trigger Action) applied successfully!" -ForegroundColor Green }
    "4" { Set-Preset 3; Write-Host "`nPreset 4 (Classic 1:1) applied successfully!" -ForegroundColor Green }
    Default { Write-Host "`nNo changes made." -ForegroundColor Gray }
}

Write-Host "`nTip: You can also cycle controller presets in real-time during" -ForegroundColor DarkCyan
Write-Host "gameplay by pressing F9 or holding Back + Start simultaneously!" -ForegroundColor DarkCyan
Write-Host ""
Start-Sleep -Seconds 2
