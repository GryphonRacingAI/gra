@echo off
if exist "%~dp0\..\.devcontainer/devcontainer.json" (
    echo "devcontainer.json already exists"
    exit 0
)
copy "%~dp0..\.devcontainer\devcontainer_template.json" "%~dp0..\.devcontainer\devcontainer.json"
echo "devcontainer.json created"

