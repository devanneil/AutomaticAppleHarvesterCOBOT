function Attach-UsbDevice {
    param (
        [string]$BusId
    )

    Write-Host "`nAttaching USB device $BusId to WSL..." -ForegroundColor Yellow

    usbipd attach --busid $BusId --wsl

    if ($LASTEXITCODE -eq 0) {
        Write-Host "USB device attached successfully." -ForegroundColor Green
        return $true
    }

    Write-Host "Failed to attach USB device." -ForegroundColor Red
    return $false
}


# --------------------------------------------------
# Find currently connected CP210x device
# --------------------------------------------------
wsl.exe -d Ubuntu-22.04 -- bash -c "exit"
$device = usbipd list |
    ForEach-Object {
        if ($_ -match '^\s*(\d+-\d+)\s+.*CP210') {
            $matches[1]
        }
    } |
    Select-Object -First 1


if (-not $device) {
    Write-Host "`nNo CP210x USB device is currently connected." -ForegroundColor Red
    exit 1
}


Write-Host "Found CP210x device at BUSID: $device" -ForegroundColor Cyan


# --------------------------------------------------
# Attach USB device
# --------------------------------------------------

if (-not (Attach-UsbDevice -BusId $device)) {
    exit 1
}

# --------------------------------------------------
# Configuration
# --------------------------------------------------

$Distro = "Ubuntu-22.04"
$Workspace = "/home/rnall/AutomaticAppleHarvesterCOBOT/ws"
$LogRoot = Join-Path $PSScriptRoot "logs"

# Create a unique directory for this run
$RunTimestamp = Get-Date -Format "yyyy-MM-dd_HH-mm-ss"
$RunLogDir = Join-Path $LogRoot "run_$RunTimestamp"

New-Item -ItemType Directory -Path $RunLogDir -Force | Out-Null

Write-Host "ROS logs will be stored in:" -ForegroundColor Cyan
Write-Host "  $RunLogDir"

# --------------------------------------------------
# Start ROS system
# --------------------------------------------------

Write-Host "`nStarting ROS system..." -ForegroundColor Green
Write-Host "Press Ctrl+C to stop ROS.`n"

$RosCommand = "cd $Workspace && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch launch_vis launch_sys.launch.py "

# Run ROS and capture everything
wsl.exe -d $Distro -- bash -c $RosCommand 2>&1 |
    Tee-Object -FilePath (Join-Path $RunLogDir "system.log")


# -----------------------
# Parse logs into human readable data
# -----------------------

function Split-RosLogByNode {
    param (
        [string]$LogFile,
        [string]$OutputDirectory
    )

    if (-not (Test-Path $LogFile)) {
        Write-Host "Log file not found: $LogFile" `
            -ForegroundColor Red
        return
    }

    New-Item -ItemType Directory `
        -Path $OutputDirectory `
        -Force | Out-Null

    $nodeFiles = @{}

    foreach ($line in Get-Content $LogFile) {

        # Match:
        # [INFO] [timestamp] [node_name]:
        if ($line -match '\[[A-Z]+\]\s+\[[0-9]+\.[0-9]+\]\s+\[([^\]]+)\]:') {

            $node = $matches[1]

            # Make sure the filename is safe
            $safeNode = $node -replace '[^\w.-]', '_'

            if (-not $nodeFiles.ContainsKey($node)) {

                $nodeFiles[$node] = Join-Path `
                    $OutputDirectory `
                    "$safeNode.log"

                # Create the file
                New-Item -ItemType File `
                    -Path $nodeFiles[$node] `
                    -Force | Out-Null
            }

            Add-Content `
                -Path $nodeFiles[$node] `
                -Value $line
        }
    }

    Write-Host "`nLogs parsed by node:" -ForegroundColor Cyan

    foreach ($node in $nodeFiles.Keys) {
        Write-Host "  $node -> $($nodeFiles[$node])"
    }
}

# --------------------------------------------------
# ROS exited
# --------------------------------------------------

$SystemLog = Join-Path $RunLogDir "system.log"
$NodeLogDir = Join-Path $RunLogDir "nodes"

Split-RosLogByNode `
    -LogFile $SystemLog `
    -OutputDirectory $NodeLogDir

Write-Host "`nROS system stopped." -ForegroundColor Yellow
Write-Host "Logs saved to:"
Write-Host "  $RunLogDir"