# Function to attach USB device
function Attach-UsbDevice {
    param (
        [string]$BusId,
        [string]$Distro
    )

    try {
        Write-Host "`nAttaching USB device $BusId to WSL distro '$Distro'..." -ForegroundColor Yellow
        usbipd attach --busid $BusId --wsl --distribution $Distro
        Write-Host "✅ USB device attached successfully." -ForegroundColor Green
    }
    catch {
        Write-Error "Failed to attach USB device: $_"
    }
}

$device = usbipd list | Select-String "CP210" | ForEach-Object { ($_ -split '\s+')[0] }
Attach-UsbDevice -BusId $device -Distro Ubuntu

# --------------------------------------------------
# Start ROS system
# --------------------------------------------------

Write-Host "[4/4] Starting ROS system..."
wsl.exe bash -c "cd /home/rnall/AutomaticAppleHarvesterCOBOT/ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch launch_vis launch_sys.launch.py"