# Modified from ChatGPT suggestions
WPILIB_VSIX_URL=$(curl -s "https://api.github.com/repos/wpilibsuite/vscode-wpilib/releases/latest" | jq -r '.assets[] | select(.name | test(".vsix$")) | .browser_download_url')
INSTALL_LOCATION="/tmp/wpilib-extension/latest.vsix"
echo "$WPILIB_VSIX_URL"
curl --create-dirs -L -o "$INSTALL_LOCATION" "$WPILIB_VSIX_URL"
code --install-extension "$INSTALL_LOCATION"

