<# rewrite-author.ps1
   Rewrites Git history to set a new author and email for all commits.
   Safe for private repositories. Creates a backup branch before rewrite.
#>

param(
  [string]$RepoPath = (Get-Location).Path,
  [string]$Branch   = "main",
  [string]$Name     = "Jevgenijs Ricko",
  [string]$Email    = "95184766+biakss@users.noreply.github.com",
  [switch]$Force,
  [switch]$Push
)

function Fail($msg){ Write-Host "[ERR ] $msg" -ForegroundColor Red; exit 1 }
function Info($msg){ Write-Host "[INFO] $msg" -ForegroundColor Cyan }
function Ok($msg){ Write-Host "[ OK ] $msg" -ForegroundColor Green }

# Check Git and repo
& git --version *>$null 2>&1
if ($LASTEXITCODE -ne 0) { Fail "Git not found in PATH." }
if (-not (Test-Path $RepoPath)) { Fail "Repo path not found." }
if (-not (Test-Path (Join-Path $RepoPath ".git"))) { Fail "No .git folder in $RepoPath" }

Push-Location $RepoPath

# Check for uncommitted changes
$dirty = git status --porcelain
if ($dirty -and -not $Force) {
  Fail "Uncommitted changes detected. Commit or run with -Force."
}

# Backup branch
$currentBranch = (git rev-parse --abbrev-ref HEAD).Trim()
$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$backupRef = "backup/auth-rewrite-$stamp"
Info "Creating backup branch: $backupRef"
git branch $backupRef | Out-Null
Ok "Backup branch created. Use 'git reset --hard $backupRef' to restore."

# Rewrite history
Info "Rewriting history with new author: $Name <$Email>"
$envFilter = "GIT_AUTHOR_NAME='$Name'; GIT_AUTHOR_EMAIL='$Email'; GIT_COMMITTER_NAME='$Name'; GIT_COMMITTER_EMAIL='$Email'"
git filter-branch --env-filter "$envFilter" -- --all
if ($LASTEXITCODE -ne 0) { Fail "git filter-branch failed." }
Ok "History rewritten locally."

# Push if requested
if ($Push) {
  Info "Pushing rewritten history to origin ($Branch)..."
  git push origin $Branch --force
  if ($LASTEXITCODE -ne 0) { Fail "git push failed." }
  Ok "Push completed successfully."
} else {
  Info "Push skipped. Run manually: git push origin $Branch --force"
}

Pop-Location

Write-Host ""
Write-Host "Restore command if needed:" -ForegroundColor Yellow
Write-Host "  cd `"$RepoPath`""
Write-Host "  git reset --hard $backupRef"
Write-Host "  git push origin $Branch --force"
