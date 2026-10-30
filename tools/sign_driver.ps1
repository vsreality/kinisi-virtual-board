<# Signs a local development package. Does not trust or install a certificate/driver. #>
param(
    [Parameter(Mandatory=$true)][string]$Wdk,
    [string]$Package = "$PSScriptRoot/../build/driver",
    [string]$Sdk = "${env:ProgramFiles(x86)}/Windows Kits/10",
    [string]$Thumbprint
)
$ErrorActionPreference = 'Stop'
$packagePath = (Resolve-Path -LiteralPath $Package).Path
$signTool = Get-ChildItem -LiteralPath "$Sdk/bin" -Filter signtool.exe -Recurse |
    Where-Object { $_.FullName -match '\\x64\\' } | Sort-Object FullName -Descending | Select-Object -First 1
$inf2cat = Get-ChildItem -LiteralPath $Wdk -Filter Inf2Cat.exe -Recurse | Select-Object -First 1
if (!$signTool -or !$inf2cat) { throw 'SDK SignTool and WDK Inf2Cat are required' }
foreach ($file in @('kinisiport.dll', 'kinisiport.inf')) {
    if (!(Test-Path -LiteralPath "$packagePath/$file")) { throw "Missing ${file}: build the driver first" }
}
if ($Thumbprint) {
    $certificate = Get-Item -LiteralPath "Cert:/CurrentUser/My/$Thumbprint"
} else {
    $certificate = New-SelfSignedCertificate -Type CodeSigningCert -Subject 'CN=Kinisi Simulator Local Development' `
        -CertStoreLocation Cert:/CurrentUser/My -NotAfter (Get-Date).AddMonths(3) -KeyExportPolicy NonExportable
}
& $signTool.FullName sign /fd SHA256 /s My /sha1 $certificate.Thumbprint "$packagePath/kinisiport.dll"
if ($LASTEXITCODE) { throw 'DLL signing failed' }
& $inf2cat.FullName "/driver:$packagePath" /os:10_X64 /uselocaltime
if ($LASTEXITCODE) { throw 'Catalog generation failed' }
& $signTool.FullName sign /fd SHA256 /s My /sha1 $certificate.Thumbprint "$packagePath/kinisiport.cat"
if ($LASTEXITCODE) { throw 'Catalog signing failed' }
Export-Certificate -Cert $certificate -FilePath "$packagePath/kinisi-test.cer" | Out-Null
Write-Output "Signed local-development package: $packagePath"
Write-Output "Certificate thumbprint: $($certificate.Thumbprint)"
Write-Output 'This script does not change certificate trust or install the driver. Keep the private key local.'
