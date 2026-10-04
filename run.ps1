# Compila y lanza el visor del holograma. Uso: .\run.ps1
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

# JavaFX (no viene con el JDK): se descarga una vez de Maven Central
$fx = "23.0.2"
New-Item -ItemType Directory -Force lib | Out-Null
foreach ($m in "base", "graphics", "controls") {
    $jar = "lib/javafx-$m-$fx-win.jar"
    if (-not (Test-Path $jar)) {
        Invoke-WebRequest "https://repo1.maven.org/maven2/org/openjfx/javafx-$m/$fx/javafx-$m-$fx-win.jar" -OutFile $jar
    }
}

# JDK: JAVA_HOME si existe; si no, el que trae Eclipse
if ($env:JAVA_HOME) { $bin = "$env:JAVA_HOME\bin" }
else {
    $bin = (Get-ChildItem "$HOME\.p2\pool\plugins" -Directory -Filter "org.eclipse.justj.openjdk.hotspot.jre.full.win32*" |
            Sort-Object Name | Select-Object -Last 1).FullName + "\jre\bin"
}

$sources = Get-ChildItem src -Recurse -Filter *.java | ForEach-Object { $_.FullName }
& "$bin\javac.exe" -encoding UTF-8 --module-path lib -d bin $sources
if ($LASTEXITCODE -ne 0) { exit 1 }

& "$bin\java.exe" --module-path "lib;bin" -m holograma/holograma.gui.HologramApp
