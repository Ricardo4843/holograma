# Lanza el detector de postura para el holograma. Uso:
#   .\webcam\run.ps1                      (webcam)
#   .\webcam\run.ps1 --video fondo.mp4 --loop
# La primera vez crea el entorno de Python (webcam/.venv) e instala MediaPipe.
$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
$py = Join-Path $here ".venv\Scripts\python.exe"
if (-not (Test-Path $py)) {
    Write-Host "Creando el entorno de Python e instalando MediaPipe (solo la primera vez)..."
    python -m venv (Join-Path $here ".venv")
    & $py -m pip install --upgrade pip
    & $py -m pip install -r (Join-Path $here "requirements.txt")
}
& $py (Join-Path $here "pose_sender.py") @args
