$RosDomainId = if ($args.Count -ge 1) { $args[0] } else { "0" }
$WindowsRosDir = "C:\.ros"
$TargetFile = Join-Path $WindowsRosDir "fastdds.xml"
$SourceFile = "C:\Users\33648\Desktop\STAGE_ROS\isaac_sim\fastdds\fastdds.xml"

if (-not (Test-Path $WindowsRosDir)) {
    New-Item -ItemType Directory -Path $WindowsRosDir -Force | Out-Null
}

if (-not (Test-Path $SourceFile)) {
    Write-Error "Template not found at $SourceFile. Update the path in install_fastdds_windows.ps1 if needed."
    exit 1
}

Copy-Item -Path $SourceFile -Destination $TargetFile -Force

[Environment]::SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", $TargetFile, "User")
[Environment]::SetEnvironmentVariable("ROS_DOMAIN_ID", $RosDomainId, "User")

$env:FASTRTPS_DEFAULT_PROFILES_FILE = $TargetFile
$env:ROS_DOMAIN_ID = $RosDomainId

Write-Host "Windows Fast DDS config installed:"
Write-Host "  $TargetFile"
Write-Host ""
Write-Host "User environment variables updated:"
Write-Host "  FASTRTPS_DEFAULT_PROFILES_FILE=$TargetFile"
Write-Host "  ROS_DOMAIN_ID=$RosDomainId"
Write-Host ""
Write-Host "Open a new PowerShell before launching Isaac Sim."
