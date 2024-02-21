if [ -f "$(dirname "$0")/../.devcontainer/devcontainer.json" ]; then
    echo "devcontainer.json already exists"
    exit 0
fi
cp "$(dirname "$0")/../.devcontainer/devcontainer_template.json" "$(dirname "$0")/../.devcontainer/devcontainer.json"
echo "devcontainer.json" created

