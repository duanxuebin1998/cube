param(
    [string]$RepoRoot = "D:\CUBE",
    [switch]$SummaryOnly,
    [string]$OutputPath = "",
    [string]$TaskId = "",
    [string]$Objective = "",
    [string]$Stage = "",
    [string]$NextStep = "",
    [string[]]$AllowedPath = @(),
    [string[]]$ExcludedPath = @(),
    [string[]]$CompletedCheck = @(),
    [string[]]$OpenRisk = @(),
    [string[]]$ConfirmedDecision = @(),
    [string[]]$Assumption = @(),
    [string[]]$OpenQuestion = @(),
    [string[]]$SourceBaseline = @(),
    [string]$LastApprovedDeliverable = "",
    [string[]]$WritePath = @(),
    [string[]]$WatchPath = @(),
    [string[]]$RelatedRepoPath = @()
)

$ErrorActionPreference = "Stop"

function Invoke-CubeGitText {
    param(
        [string[]]$Arguments,
        [string]$WorkingRoot = $RepoRoot
    )

    $previousErrorAction = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $output = & git -C $WorkingRoot @Arguments 2>$null
    $exitCode = $LASTEXITCODE
    $ErrorActionPreference = $previousErrorAction
    if ($exitCode -ne 0) {
        throw "git $($Arguments -join ' ') failed with exit code $exitCode"
    }
    return @($output)
}

function Get-CubeItemCount {
    param([object[]]$Items)
    return @($Items | Where-Object { $_ -ne $null -and $_.ToString().Length -gt 0 }).Count
}

function Get-CubeVersionString {
    param([string]$Path, [string]$Macro)

    $line = & rg -N $Macro $Path 2>$null | Select-Object -First 1
    if ($line -match 'V[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+') {
        return $Matches[0]
    }
    return "UNKNOWN"
}

$RepoRoot = [IO.Path]::GetFullPath($RepoRoot)
if (-not (Test-Path -LiteralPath $RepoRoot -PathType Container)) {
    throw "Repository path does not exist: $RepoRoot"
}

$inside = Invoke-CubeGitText -Arguments @("rev-parse", "--is-inside-work-tree")
if (($inside -join "").Trim() -ne "true") {
    throw "Target path is not a Git worktree: $RepoRoot"
}

$branch = ((Invoke-CubeGitText -Arguments @("branch", "--show-current")) -join "").Trim()
$head = ((Invoke-CubeGitText -Arguments @("rev-parse", "--short=12", "HEAD")) -join "").Trim()
$gitRoot = [IO.Path]::GetFullPath(
    ((Invoke-CubeGitText -Arguments @("rev-parse", "--show-toplevel")) -join "").Trim()
)
$gitCommonDirectoryText = ((Invoke-CubeGitText -Arguments @("rev-parse", "--git-common-dir")) -join "").Trim()
$gitCommonDirectory = if ([IO.Path]::IsPathRooted($gitCommonDirectoryText)) {
    [IO.Path]::GetFullPath($gitCommonDirectoryText)
}
else {
    [IO.Path]::GetFullPath((Join-Path $RepoRoot $gitCommonDirectoryText))
}
$upstream = try {
    ((Invoke-CubeGitText -Arguments @("rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{upstream}")) -join "").Trim()
}
catch {
    ""
}
$aheadCount = 0
$behindCount = 0
if (-not [string]::IsNullOrWhiteSpace($upstream)) {
    $divergence = ((Invoke-CubeGitText -Arguments @("rev-list", "--left-right", "--count", "HEAD...$upstream")) -join " ").Trim()
    if ($divergence -match '^(\d+)\s+(\d+)$') {
        $aheadCount = [int]$Matches[1]
        $behindCount = [int]$Matches[2]
    }
}
$hookPath = try {
    ((Invoke-CubeGitText -Arguments @("config", "--get", "core.hooksPath")) -join "").Trim()
}
catch {
    ""
}
$status = @(Invoke-CubeGitText -Arguments @("status", "--short", "--untracked-files=all"))
$staged = @(Invoke-CubeGitText -Arguments @("diff", "--cached", "--name-only"))
$unstaged = @(Invoke-CubeGitText -Arguments @("diff", "--name-only"))
$untracked = @(Invoke-CubeGitText -Arguments @("ls-files", "--others", "--exclude-standard"))

$cpu2Version = Get-CubeVersionString `
    -Path "$RepoRoot\LTD_MAIN_CPU2\Application\Inc\app_version.h" `
    -Macro "CPU2_APP_VERSION_STRING"
$cpu3Version = Get-CubeVersionString `
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

$localOnlySnapshots = @()
foreach ($target in $localOnlyTargets) {
    $fullPath = Join-Path $RepoRoot $target.RelativePath
    $exists = Test-Path -LiteralPath $fullPath
    $tracked = @(Invoke-CubeGitText -Arguments @("ls-files", "--", $target.RelativePath))
    $localOnlySnapshots += [PSCustomObject]@{
        label = $target.Label
        relativePath = $target.RelativePath
        exists = $exists
        trackedCount = Get-CubeItemCount $tracked
    }
}

$relatedRepoSnapshots = @()
foreach ($relatedPath in @($RelatedRepoPath | Sort-Object -Unique)) {
    $resolvedRelatedPath = [IO.Path]::GetFullPath($relatedPath)
    $relatedExists = Test-Path -LiteralPath $resolvedRelatedPath -PathType Container
    $relatedIsGit = $false
    $relatedGitRoot = ""
    $relatedBranch = ""
    $relatedHead = ""
    $relatedStatus = @()
    $relatedStaged = @()
    $relatedUnstaged = @()
    $relatedUntracked = @()

    if ($relatedExists) {
        try {
            $relatedInside = Invoke-CubeGitText -WorkingRoot $resolvedRelatedPath -Arguments @("rev-parse", "--is-inside-work-tree")
            $relatedIsGit = (($relatedInside -join "").Trim() -eq "true")
            if ($relatedIsGit) {
                $relatedGitRoot = [IO.Path]::GetFullPath(
                    ((Invoke-CubeGitText -WorkingRoot $resolvedRelatedPath -Arguments @("rev-parse", "--show-toplevel")) -join "").Trim()
                )
                $relatedBranch = ((Invoke-CubeGitText -WorkingRoot $relatedGitRoot -Arguments @("branch", "--show-current")) -join "").Trim()
                $relatedHead = ((Invoke-CubeGitText -WorkingRoot $relatedGitRoot -Arguments @("rev-parse", "--short=12", "HEAD")) -join "").Trim()
                $relatedStatus = @(Invoke-CubeGitText -WorkingRoot $relatedGitRoot -Arguments @("status", "--short", "--untracked-files=all"))
                $relatedStaged = @(Invoke-CubeGitText -WorkingRoot $relatedGitRoot -Arguments @("diff", "--cached", "--name-only"))
                $relatedUnstaged = @(Invoke-CubeGitText -WorkingRoot $relatedGitRoot -Arguments @("diff", "--name-only"))
                $relatedUntracked = @(Invoke-CubeGitText -WorkingRoot $relatedGitRoot -Arguments @("ls-files", "--others", "--exclude-standard"))
            }
        }
        catch {
            $relatedIsGit = $false
        }
    }

    $relatedRepoSnapshots += [PSCustomObject]@{
        requestedPath = $resolvedRelatedPath
        gitRoot = $relatedGitRoot
        exists = $relatedExists
        isGitRepository = $relatedIsGit
        branch = $relatedBranch
        head = $relatedHead
        status = @($relatedStatus)
        staged = @($relatedStaged)
        unstaged = @($relatedUnstaged)
        untracked = @($relatedUntracked)
    }
}

$resolvedWritePaths = @()
foreach ($candidatePath in @($WritePath | Sort-Object -Unique)) {
    $resolvedCandidatePath = if ([IO.Path]::IsPathRooted($candidatePath)) {
        [IO.Path]::GetFullPath($candidatePath)
    }
    else {
        [IO.Path]::GetFullPath((Join-Path $RepoRoot $candidatePath))
    }
    if ($resolvedCandidatePath -eq $RepoRoot) {
        throw "Repository root is too broad for a write path"
    }
    $repoPrefix = $RepoRoot.TrimEnd('\') + '\'
    if (-not $resolvedCandidatePath.StartsWith($repoPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Write path must stay inside the repository: $resolvedCandidatePath"
    }
    $resolvedWritePaths += $resolvedCandidatePath
}

$watchPathSnapshots = @()
foreach ($candidatePath in @($WatchPath | Sort-Object -Unique)) {
    $resolvedCandidatePath = if ([IO.Path]::IsPathRooted($candidatePath)) {
        [IO.Path]::GetFullPath($candidatePath)
    }
    else {
        [IO.Path]::GetFullPath((Join-Path $RepoRoot $candidatePath))
    }
    if ($resolvedCandidatePath -eq $RepoRoot) {
        throw "Repository root is too broad for a watch path"
    }
    $repoPrefix = $RepoRoot.TrimEnd('\') + '\'
    if (-not $resolvedCandidatePath.StartsWith($repoPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Watch path must stay inside the repository: $resolvedCandidatePath"
    }
    $exists = Test-Path -LiteralPath $resolvedCandidatePath
    $pathType = if (-not $exists) {
        "missing"
    }
    elseif (Test-Path -LiteralPath $resolvedCandidatePath -PathType Leaf) {
        "file"
    }
    else {
        "directory"
    }
    $sha256 = if ($pathType -eq "file") {
        (Get-FileHash -LiteralPath $resolvedCandidatePath -Algorithm SHA256).Hash
    }
    else {
        ""
    }
    $lastWriteTime = if ($exists) {
        (Get-Item -LiteralPath $resolvedCandidatePath).LastWriteTimeUtc.ToString("o")
    }
    else {
        ""
    }
    $watchPathSnapshots += [PSCustomObject]@{
        path = $resolvedCandidatePath
        exists = $exists
        type = $pathType
        sha256 = $sha256
        lastWriteTimeUtc = $lastWriteTime
    }
}

$skillEntryPath = Join-Path $RepoRoot ".agents\skills\cube-development\SKILL.md"
$skillAgentPath = Join-Path $RepoRoot ".agents\skills\cube-development\agents\openai.yaml"
$skillEntrySha256 = if (Test-Path -LiteralPath $skillEntryPath -PathType Leaf) {
    (Get-FileHash -LiteralPath $skillEntryPath -Algorithm SHA256).Hash
}
else {
    ""
}
$skillAgentSha256 = if (Test-Path -LiteralPath $skillAgentPath -PathType Leaf) {
    (Get-FileHash -LiteralPath $skillAgentPath -Algorithm SHA256).Hash
}
else {
    ""
}

$snapshotData = [ordered]@{
    schemaVersion = 2
    capturedAt = [DateTimeOffset]::Now.ToString("o")
    task = [ordered]@{
        id = $TaskId
        objective = $Objective
        stage = $Stage
        nextStep = $NextStep
        allowedPaths = @($AllowedPath)
        excludedPaths = @($ExcludedPath)
        completedChecks = @($CompletedCheck)
        openRisks = @($OpenRisk)
        confirmedDecisions = @($ConfirmedDecision)
        assumptions = @($Assumption)
        openQuestions = @($OpenQuestion)
        sourceBaselines = @($SourceBaseline)
        lastApprovedDeliverable = $LastApprovedDeliverable
        writePaths = @($resolvedWritePaths)
        watchPaths = @($watchPathSnapshots)
    }
    repository = [ordered]@{
        path = $RepoRoot
        gitRoot = $gitRoot
        gitCommonDirectory = $gitCommonDirectory
        branch = $branch
        head = $head
        upstream = $upstream
        ahead = $aheadCount
        behind = $behindCount
        hooksPath = $hookPath
        status = @($status)
        staged = @($staged)
        unstaged = @($unstaged)
        untracked = @($untracked)
    }
    versions = [ordered]@{
        cpu2 = $cpu2Version
        cpu3 = $cpu3Version
        deviceProtocol = @($protocolVersions)
        cpu2ParameterStorage = @($cpu2ParamVersion)
        cpu3ParameterStorage = @($cpu3ParamVersion)
    }
    skill = [ordered]@{
        entryPath = $skillEntryPath
        entrySha256 = $skillEntrySha256
        agentConfigPath = $skillAgentPath
        agentConfigSha256 = $skillAgentSha256
    }
    localOnly = @($localOnlySnapshots)
    relatedRepositories = @($relatedRepoSnapshots)
}

Write-Output "CUBE_WORKSPACE_SNAPSHOT"
Write-Output "repo=$RepoRoot"
Write-Output "branch=$branch"
Write-Output "head=$head"
Write-Output "upstream=$upstream"
Write-Output "ahead=$aheadCount"
Write-Output "behind=$behindCount"
Write-Output "hooksPath=$hookPath"
Write-Output "statusCount=$(Get-CubeItemCount $status)"
Write-Output "stagedCount=$(Get-CubeItemCount $staged)"
Write-Output "unstagedCount=$(Get-CubeItemCount $unstaged)"
Write-Output "untrackedCount=$(Get-CubeItemCount $untracked)"
Write-Output "cpu2Version=$cpu2Version"
Write-Output "cpu3Version=$cpu3Version"
Write-Output "deviceProtocol=$(($protocolVersions -join '; ').Trim())"
Write-Output "cpu2ParamVersion=$(($cpu2ParamVersion -join '; ').Trim())"
Write-Output "cpu3ParamVersion=$(($cpu3ParamVersion -join '; ').Trim())"

foreach ($target in $localOnlySnapshots) {
    Write-Output "localOnly[$($target.label)].path=$($target.relativePath)"
    Write-Output "localOnly[$($target.label)].exists=$($target.exists)"
    Write-Output "localOnly[$($target.label)].trackedCount=$($target.trackedCount)"
}

foreach ($related in $relatedRepoSnapshots) {
    Write-Output "relatedRepo[$($related.requestedPath)].exists=$($related.exists)"
    Write-Output "relatedRepo[$($related.requestedPath)].isGitRepository=$($related.isGitRepository)"
    Write-Output "relatedRepo[$($related.requestedPath)].gitRoot=$($related.gitRoot)"
    Write-Output "relatedRepo[$($related.requestedPath)].branch=$($related.branch)"
    Write-Output "relatedRepo[$($related.requestedPath)].head=$($related.head)"
    Write-Output "relatedRepo[$($related.requestedPath)].statusCount=$(Get-CubeItemCount $related.status)"
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

if (-not [string]::IsNullOrWhiteSpace($OutputPath)) {
    $resolvedOutputPath = if ([IO.Path]::IsPathRooted($OutputPath)) {
        [IO.Path]::GetFullPath($OutputPath)
    }
    else {
        [IO.Path]::GetFullPath((Join-Path $RepoRoot $OutputPath))
    }
    $tmpRoot = [IO.Path]::GetFullPath((Join-Path $RepoRoot "tmp"))
    $tmpPrefix = $tmpRoot.TrimEnd('\') + '\'
    if (-not $resolvedOutputPath.StartsWith($tmpPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Snapshot output must be inside the repository tmp directory: $resolvedOutputPath"
    }
    if ([IO.Path]::GetExtension($resolvedOutputPath) -ne ".json") {
        throw "Snapshot output must use the .json extension: $resolvedOutputPath"
    }
    $outputDirectory = [IO.Path]::GetDirectoryName($resolvedOutputPath)
    [IO.Directory]::CreateDirectory($outputDirectory) | Out-Null
    $json = $snapshotData | ConvertTo-Json -Depth 8
    [IO.File]::WriteAllText(
        $resolvedOutputPath,
        $json + [Environment]::NewLine,
        [Text.UTF8Encoding]::new($false)
    )
    Write-Output "snapshotOutput=$resolvedOutputPath"
}
