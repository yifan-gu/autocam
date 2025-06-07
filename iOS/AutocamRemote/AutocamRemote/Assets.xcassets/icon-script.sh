#!/usr/bin/env bash
set -euo pipefail

# 1) Master PNG (must be 1024×1024)
SRC="icon-1024.png"
if [[ ! -f "$SRC" ]]; then
  echo "❌ $SRC not found in this folder." >&2
  exit 1
fi

# 2) Prepare the .appiconset dir
DEST="AppIcon.appiconset"
rm -rf "$DEST"
mkdir -p "$DEST"

# 3) Specify each icon: name:pixel_size
ICON_SPECS=(
  "AppStore-1024@1x:1024"
  "Icon-20@2x:40"
  "Icon-20@3x:60"
  "Icon-29@2x:58"
  "Icon-29@3x:87"
  "Icon-40@2x:80"
  "Icon-40@3x:120"
  "Icon-60@2x:120"
  "Icon-60@3x:180"
  "Icon-76@1x:76"
  "Icon-76@2x:152"
  "Icon-83.5@2x:167"
)

# 4) Loop and generate each PNG
for spec in "${ICON_SPECS[@]}"; do
  name=${spec%%:*}       # before the colon
  size=${spec##*:}       # after the colon
  out="$DEST/${name}.png"
  echo "→ $name → ${size}×${size}"
  sips -Z "$size" "$SRC" --out "$out" >/dev/null
done

# 5) Write Contents.json
cat > "$DEST/Contents.json" <<'EOF'
{
  "images": [
    { "size": "20x20",    "idiom": "iphone",       "filename": "Icon-20@2x.png",    "scale": "2x" },
    { "size": "20x20",    "idiom": "iphone",       "filename": "Icon-20@3x.png",    "scale": "3x" },
    { "size": "29x29",    "idiom": "iphone",       "filename": "Icon-29@2x.png",    "scale": "2x" },
    { "size": "29x29",    "idiom": "iphone",       "filename": "Icon-29@3x.png",    "scale": "3x" },
    { "size": "40x40",    "idiom": "iphone",       "filename": "Icon-40@2x.png",    "scale": "2x" },
    { "size": "40x40",    "idiom": "iphone",       "filename": "Icon-40@3x.png",    "scale": "3x" },
    { "size": "60x60",    "idiom": "iphone",       "filename": "Icon-60@2x.png",    "scale": "2x" },
    { "size": "60x60",    "idiom": "iphone",       "filename": "Icon-60@3x.png",    "scale": "3x" },
    { "size": "20x20",    "idiom": "ipad",         "filename": "Icon-20@2x.png",    "scale": "2x" },
    { "size": "29x29",    "idiom": "ipad",         "filename": "Icon-29@2x.png",    "scale": "2x" },
    { "size": "40x40",    "idiom": "ipad",         "filename": "Icon-40@2x.png",    "scale": "2x" },
    { "size": "76x76",    "idiom": "ipad",         "filename": "Icon-76@1x.png",    "scale": "1x" },
    { "size": "76x76",    "idiom": "ipad",         "filename": "Icon-76@2x.png",    "scale": "2x" },
    { "size": "83.5x83.5","idiom": "ipad",         "filename": "Icon-83.5@2x.png",  "scale": "2x" },
    { "size": "1024x1024","idiom": "ios-marketing","filename": "AppStore-1024@1x.png","scale":"1x"}
  ],
  "info": { "version": 1, "author": "xcode" }
}
EOF

echo "✅ Done. Now drag the "$DEST" folder into your Assets.xcassets."
