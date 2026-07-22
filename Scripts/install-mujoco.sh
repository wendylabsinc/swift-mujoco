#!/usr/bin/env bash
# Install MuJoCo headers + shared library from the installed `mujoco` pip wheel
# into $MUJOCO_PREFIX (default $HOME/.local) and write a pkg-config file.
# The wheel ships identical headers + a versioned shared lib on macOS and Linux,
# so dev and CI/image link the exact same MuJoCo the Python path uses.
set -euo pipefail
PREFIX="${MUJOCO_PREFIX:-$HOME/.local}"
PY="${PYTHON:-python3}"

PKGDIR="$("$PY" -c 'import mujoco, os; print(os.path.dirname(mujoco.__file__))')"
VER="$("$PY" -c 'import mujoco; print(mujoco.__version__)')"
echo "mujoco wheel: $PKGDIR (version $VER) -> $PREFIX"

mkdir -p "$PREFIX/include/mujoco" "$PREFIX/lib/pkgconfig"
cp -R "$PKGDIR/include/mujoco/." "$PREFIX/include/mujoco/"

# Locate the shared library inside the wheel (libmujoco.<ver>.dylib | libmujoco.so.<ver>)
LIB="$(find "$PKGDIR" -maxdepth 1 \( -name 'libmujoco*.dylib' -o -name 'libmujoco*.so*' \) | head -n1)"
[ -n "$LIB" ] || { echo "no libmujoco found in $PKGDIR" >&2; exit 1; }
cp "$LIB" "$PREFIX/lib/"
BASE="$(basename "$LIB")"

case "$(uname -s)" in
  Darwin)
    ln -sf "$BASE" "$PREFIX/lib/libmujoco.dylib"
    install_name_tool -id "$PREFIX/lib/$BASE" "$PREFIX/lib/$BASE" || true
    ;;
  Linux)
    ln -sf "$BASE" "$PREFIX/lib/libmujoco.so"
    ldconfig "$PREFIX/lib" 2>/dev/null || true
    ;;
esac

cat > "$PREFIX/lib/pkgconfig/mujoco.pc" <<EOF
prefix=$PREFIX
libdir=\${prefix}/lib
includedir=\${prefix}/include
Name: mujoco
Description: MuJoCo physics engine
Version: $VER
Libs: -L\${libdir} -lmujoco
Cflags: -I\${includedir}
EOF
echo "wrote $PREFIX/lib/pkgconfig/mujoco.pc"
