#requires -Version 5.1
[CmdletBinding()]
param(
    [Parameter(Position = 0, ValueFromPipeline = $true, ValueFromPipelineByPropertyName = $true)]
    [Alias('FullName')]
    [string[]] $Path,

    [string] $OutputDirectory,

    [ValidateSet('html', 'latex')]
    [string] $Engine = 'html',

    [string] $ThemePath,

    [string] $BrowserPath,

    [string] $MainFont = 'Arial Unicode MS',

    [string] $CjkFont = 'Microsoft YaHei',

    [string] $MonoFont = 'Consolas',

    [string] $Margin = '20mm',

    [switch] $Toc,

    [switch] $KeepHtml,

    [switch] $Recurse
)

begin {
    $ErrorActionPreference = 'Stop'
    $pendingPaths = New-Object 'System.Collections.Generic.List[string]'

    # Refresh PATH because WinGet installers may update it after this shell started.
    $machinePath = [Environment]::GetEnvironmentVariable('Path', 'Machine')
    $userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
    $env:Path = @($machinePath, $userPath, $env:Path) -join ';'

    function Resolve-RequiredCommand {
        param([Parameter(Mandatory = $true)][string] $Name)

        $command = Get-Command $Name -ErrorAction SilentlyContinue
        if (-not $command) {
            throw "Command not found: $Name. Confirm Pandoc and MiKTeX are installed, then reopen PowerShell."
        }

        return $command.Source
    }

    function Resolve-BrowserCommand {
        if ($BrowserPath) {
            $fullPath = [IO.Path]::GetFullPath((Join-Path (Get-Location) $BrowserPath))
            if ([IO.Path]::IsPathRooted($BrowserPath)) {
                $fullPath = [IO.Path]::GetFullPath($BrowserPath)
            }
            if (-not (Test-Path -LiteralPath $fullPath)) {
                throw "Browser not found: $fullPath"
            }
            return $fullPath
        }

        $commandNames = @('chrome', 'msedge', 'chromium', 'chromium-browser')
        foreach ($name in $commandNames) {
            $command = Get-Command $name -ErrorAction SilentlyContinue
            if ($command) {
                return $command.Source
            }
        }

        $candidatePaths = @(
            "$env:ProgramFiles\Google\Chrome\Application\chrome.exe",
            "${env:ProgramFiles(x86)}\Google\Chrome\Application\chrome.exe",
            "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe",
            "${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe"
        )
        foreach ($candidate in $candidatePaths) {
            if ($candidate -and (Test-Path -LiteralPath $candidate)) {
                return $candidate
            }
        }

        throw 'Browser not found. Install Chrome or Microsoft Edge, or pass -BrowserPath.'
    }

    $pandoc = Resolve-RequiredCommand 'pandoc'
    $xelatex = $null
    $browser = $null

    if ($Engine -eq 'latex') {
        $xelatex = Resolve-RequiredCommand 'xelatex'
    } else {
        $browser = Resolve-BrowserCommand
    }

    if (-not $ThemePath) {
        $ThemePath = Join-Path $PSScriptRoot 'pdf-theme.css'
    }
    if ([IO.Path]::IsPathRooted($ThemePath)) {
        $ThemePath = [IO.Path]::GetFullPath($ThemePath)
    } else {
        $ThemePath = [IO.Path]::GetFullPath((Join-Path (Get-Location) $ThemePath))
    }
    if ($Engine -eq 'html' -and -not (Test-Path -LiteralPath $ThemePath)) {
        throw "Theme CSS not found: $ThemePath"
    }

    if ($OutputDirectory) {
        if ([IO.Path]::IsPathRooted($OutputDirectory)) {
            $OutputDirectory = [IO.Path]::GetFullPath($OutputDirectory)
        } else {
            $OutputDirectory = [IO.Path]::GetFullPath((Join-Path (Get-Location) $OutputDirectory))
        }

        if (-not (Test-Path -LiteralPath $OutputDirectory)) {
            New-Item -ItemType Directory -Path $OutputDirectory | Out-Null
        }
    }

    function Convert-MarkdownToLatexPdf {
        param(
            [Parameter(Mandatory = $true)][IO.FileInfo] $MarkdownFile,
            [Parameter(Mandatory = $true)][string] $PdfPath
        )

        $pandocArgs = @(
            $MarkdownFile.FullName,
            '-o', $PdfPath,
            '--pdf-engine', $xelatex,
            '-V', "mainfont=$MainFont",
            '-V', "CJKmainfont=$CjkFont",
            '-V', "monofont=$MonoFont",
            '-V', "geometry:margin=$Margin"
        )

        & $pandoc @pandocArgs
        if ($LASTEXITCODE -ne 0) {
            throw "Conversion failed: $($MarkdownFile.FullName)"
        }
    }

    function Convert-MarkdownToHtmlPdf {
        param(
            [Parameter(Mandatory = $true)][IO.FileInfo] $MarkdownFile,
            [Parameter(Mandatory = $true)][string] $PdfPath
        )

        $tempRoot = Join-Path ([IO.Path]::GetTempPath()) ("cube-md-pdf-" + [Guid]::NewGuid().ToString('N'))
        New-Item -ItemType Directory -Path $tempRoot | Out-Null

        $htmlPath = if ($KeepHtml) {
            Join-Path ([IO.Path]::GetDirectoryName($PdfPath)) ([IO.Path]::GetFileNameWithoutExtension($PdfPath) + '.html')
        } else {
            Join-Path $tempRoot ($MarkdownFile.BaseName + '.html')
        }
        $profilePath = Join-Path $tempRoot 'chrome-profile'

        $pandocArgs = @(
            $MarkdownFile.FullName,
            '--from', 'gfm',
            '--standalone',
            '--section-divs',
            '--syntax-highlighting', 'tango',
            '--metadata', "pagetitle=$($MarkdownFile.BaseName)",
            '-o', $htmlPath
        )
        if ($Toc) {
            $pandocArgs += @('--toc', '--toc-depth', '3')
        }

        & $pandoc @pandocArgs
        if ($LASTEXITCODE -ne 0) {
            throw "HTML generation failed: $($MarkdownFile.FullName)"
        }

        $utf8 = New-Object Text.UTF8Encoding($true)
        $html = [IO.File]::ReadAllText($htmlPath, [Text.Encoding]::UTF8)
        $css = [IO.File]::ReadAllText($ThemePath, [Text.Encoding]::UTF8)
        $title = [Security.SecurityElement]::Escape($MarkdownFile.BaseName)
        $styleBlock = "<style>`n$css`n</style>`n"
        $metaBlock = "<meta name=`"document-title`" content=`"$title`">`n"
        $html = $html -replace '</head>', "$metaBlock$styleBlock</head>"
        [IO.File]::WriteAllText($htmlPath, $html, $utf8)

        $htmlUri = ([Uri] (Resolve-Path -LiteralPath $htmlPath).Path).AbsoluteUri
        $browserArgs = @(
            '--headless',
            '--disable-gpu',
            '--disable-extensions',
            '--disable-background-networking',
            '--disable-sync',
            '--disable-default-apps',
            '--no-first-run',
            '--log-level=3',
            '--allow-file-access-from-files',
            "--user-data-dir=$profilePath",
            '--run-all-compositor-stages-before-draw',
            '--virtual-time-budget=1000',
            '--no-pdf-header-footer',
            "--print-to-pdf=$PdfPath",
            $htmlUri
        )

        $browserOutput = & $browser @browserArgs 2>&1
        if ($LASTEXITCODE -ne 0) {
            throw "Browser PDF rendering failed: $($MarkdownFile.FullName)"
        }
        $deadline = (Get-Date).AddSeconds(15)
        while ((-not (Test-Path -LiteralPath $PdfPath)) -and (Get-Date) -lt $deadline) {
            Start-Sleep -Milliseconds 200
        }
        if (-not (Test-Path -LiteralPath $PdfPath)) {
            throw "PDF was not generated: $PdfPath"
        }
        while (((Get-Item -LiteralPath $PdfPath).Length -le 0) -and (Get-Date) -lt $deadline) {
            Start-Sleep -Milliseconds 200
        }
        if ((Get-Item -LiteralPath $PdfPath).Length -le 0) {
            throw "Generated PDF is empty: $PdfPath"
        }

        if (-not $KeepHtml -and (Test-Path -LiteralPath $tempRoot)) {
            Remove-Item -LiteralPath $tempRoot -Recurse -Force -ErrorAction SilentlyContinue
        }
    }

    function Convert-MarkdownFile {
        param([Parameter(Mandatory = $true)][IO.FileInfo] $MarkdownFile)

        if ($MarkdownFile.Extension -ne '.md') {
            Write-Warning "Skip non-Markdown file: $($MarkdownFile.FullName)"
            return
        }

        $pdfDirectory = if ($OutputDirectory) { $OutputDirectory } else { $MarkdownFile.DirectoryName }
        $pdfPath = Join-Path $pdfDirectory ($MarkdownFile.BaseName + '.pdf')

        if ($Engine -eq 'latex') {
            Convert-MarkdownToLatexPdf $MarkdownFile $pdfPath
        } else {
            Convert-MarkdownToHtmlPdf $MarkdownFile $pdfPath
        }

        Write-Host "Generated PDF ($Engine): $pdfPath"
    }
}

process {
    if ($Path) {
        foreach ($entry in $Path) {
            [void] $pendingPaths.Add($entry)
        }
    }
}

end {
    if ($pendingPaths.Count -eq 0 -and $Path) {
        foreach ($entry in $Path) {
            [void] $pendingPaths.Add($entry)
        }
    }

    if ($pendingPaths.Count -eq 0 -and [Console]::IsInputRedirected) {
        $stdinText = [Console]::In.ReadToEnd()
        foreach ($entry in ($stdinText -split "`r?`n")) {
            if (-not [string]::IsNullOrWhiteSpace($entry)) {
                [void] $pendingPaths.Add($entry.Trim())
            }
        }
    }

    if ($pendingPaths.Count -eq 0) {
        throw 'No Markdown path specified.'
    }

    foreach ($entry in $pendingPaths) {
        $item = Get-Item -LiteralPath $entry -ErrorAction Stop

        if ($item.PSIsContainer) {
            $files = Get-ChildItem -LiteralPath $item.FullName -Filter '*.md' -File -Recurse:$Recurse
            foreach ($file in $files) {
                Convert-MarkdownFile $file
            }
        } else {
            Convert-MarkdownFile ([IO.FileInfo] $item.FullName)
        }
    }
}
