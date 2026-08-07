param(
    [string]$RepoRoot = "D:\CUBE",
    [switch]$SummaryOnly
)

$ErrorActionPreference = "Stop"

function Invoke-GitText {
    param([string[]]$Arguments)

    $previousErrorAction = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $output = & git -C $RepoRoot @Arguments 2>$null
    $exitCode = $LASTEXITCODE
    $ErrorActionPreference = $previousErrorAction
    if ($exitCode -ne 0) {
        throw "git $($Arguments -join ' ') failed with exit code $exitCode"
    }
    return @($output)
}

function Get-Count {
    param([object[]]$Items)
    return @($Items | Where-Object { $_ -ne $null -and $_.ToString().Length -gt 0 }).Count
}

function Get-VersionString {
    param([string]$Path, [string]$Macro)

    $line = & rg -N $Macro $Path 2>$null | Select-Object -First 1
    if ($line -match 'V[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+') {
        return $Matches[0]
    }
    return "UNKNOWN"
}

if (-not (Test-Path -LiteralPath $RepoRoot -PathType Container)) {
    throw "Repository path does not exist: $RepoRoot"
}

$inside = Invoke-GitText -Arguments @("rev-parse", "--is-inside-work-tree")
if (($inside -join "").Trim() -ne "true") {
    throw "Target path is not a Git worktree: $RepoRoot"
}

$branch = ((Invoke-GitText -Arguments @("branch", "--show-current")) -join "").Trim()
$head = ((Invoke-GitText -Arguments @("rev-parse", "--short=12", "HEAD")) -join "").Trim()
$hookPath = ((Invoke-GitText -Arguments @("config", "--get", "core.hooksPath")) -join "").Trim()
$status = @(Invoke-GitText -Arguments @("status", "--short", "--untracked-files=all"))
$staged = @(Invoke-GitText -Arguments @("diff", "--cached", "--name-only"))
$unstaged = @(Invoke-GitText -Arguments @("diff", "--name-only"))
$untracked = @(Invoke-GitText -Arguments @("ls-files", "--others", "--exclude-standard"))

$cpu2Version = Get-VersionString `
    -Path "$RepoRoot\LTD_MAIN_CPU2\Application\Inc\app_version.h" `
    -Macro "CPU2_APP_VERSION_STRING"
$cpu3Version = Get-VersionString `
    -Path "$RepoRoot\LTD_DISPLAY_CPU3\Application\app_version.h" `
    -Macro "CPU3_APP_VERSION_STRING"
$protocolVersions = & rg -N --no-filename -o '#define DEVICE_PROTOCOL_VERSION[ \t]+[0-9]+[uU]?' `
    "$RepoRoot\LTD_MAIN_CPU2\Services\ParamStorage\system_parameter.h" `
    "$RepoRoot\LTD_DISPLAY_CPU3\Application\system_param\system_parameter.h" 2>$null
$cpu2ParamVersion = & rg -N --no-filename -o '#define DEVICE_PARAM_VERSION[ \t]+\([^\r\n]+\)' `
    "$RepoRoot\LTD_MAIN_CPU2\Services\ParamStorage\system_parameter.c" 2>$null
$cpu3ParamVersion = & rg -N --no-filename -o '#define CPU3_PARAM_VERSION[ \t]+0x[0-9A-Fa-f]+[uU]?' `
    "$RepoRoot\LTD_DISPLAY_CPU3\Application\system_param\cpu3_comm_display_params.c" 2>$null

$flowRoot = Get-ChildItem -LiteralPath (Join-Path $RepoRoot "docs") -Directory -ErrorAction SilentlyContinue |
    Where-Object {
        (Test-Path -LiteralPath (Join-Path $_.FullName "CPU2")) -and
        (Test-Path -LiteralPath (Join-Path $_.FullName "CPU3"))
    } |
    Select-Object -First 1

$localOnlyTargets = @(
    [PSCustomObject]@{ Label = "tools"; RelativePath = "tools" },
    [PSCustomObject]@{ Label = "docs-site"; RelativePath = "docs-site" },
    [PSCustomObject]@{ Label = "outputs"; RelativePath = "outputs" }
)
if ($null -ne $flowRoot) {
    $flowRelative = $flowRoot.FullName.Substring($RepoRoot.TrimEnd('\').Length + 1)
    $localOnlyTargets += [PSCustomObject]@{ Label = "flow-docs"; RelativePath = $flowRelative }
}

Write-Output "CUBE_WORKSPACE_SNAPSHOT"
Write-Output "repo=$RepoRoot"
Write-Output "branch=$branch"
Write-Output "head=$head"
Write-Output "hooksPath=$hookPath"
Write-Output "statusCount=$(Get-Count $status)"
Write-Output "stagedCount=$(Get-Count $staged)"
Write-Output "unstagedCount=$(Get-Count $unstaged)"
Write-Output "untrackedCount=$(Get-Count $untracked)"
Write-Output "cpu2Version=$cpu2Version"
Write-Output "cpu3Version=$cpu3Version"
Write-Output "deviceProtocol=$(($protocolVersions -join '; ').Trim())"
Write-Output "cpu2ParamVersion=$(($cpu2ParamVersion -join '; ').Trim())"
Write-Output "cpu3ParamVersion=$(($cpu3ParamVersion -join '; ').Trim())"

foreach ($target in $localOnlyTargets) {
    $fullPath = Join-Path $RepoRoot $target.RelativePath
    $exists = Test-Path -LiteralPath $fullPath
    $tracked = @(Invoke-GitText -Arguments @("ls-files", "--", $target.RelativePath))
    Write-Output "localOnly[$($target.Label)].path=$($target.RelativePath)"
    Write-Output "localOnly[$($target.Label)].exists=$exists"
    Write-Output "localOnly[$($target.Label)].trackedCount=$(Get-Count $tracked)"
}

if (-not $SummaryOnly) {
    Write-Output "STATUS_BEGIN"
    $status | ForEach-Object { Write-Output $_ }
    Write-Output "STATUS_END"
    Write-Output "STAGED_BEGIN"
    $staged | ForEach-Object { Write-Output $_ }
    Write-Output "STAGED_END"
    Write-Output "UNSTAGED_BEGIN"
    $unstaged | ForEach-Object { Write-Output $_ }
    Write-Output "UNSTAGED_END"
    Write-Output "UNTRACKED_BEGIN"
    $untracked | ForEach-Object { Write-Output $_ }
    Write-Output "UNTRACKED_END"
}
