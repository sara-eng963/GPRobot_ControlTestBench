set -eu
r="$PWD"
m="/mnt/c/Program Files/MATLAB/R2024b/extern"
out="$r/minimal_velocity/visible_admittance"
u="$r/vendor/GPRobotControl_Simulation-15aa22efc071554fe06f1e42e1007633dee15cc9"
d=$(mktemp -d /tmp/rb2-mingw-XXXXXX)
mkdir -p "$d/pkg/partial" "$d/root"
apt-get -y -d --no-install-recommends -o Debug::NoLocking=1 -o "Dir::Cache::archives=$d/pkg" install gcc-mingw-w64-x86-64-posix > "$out/results/compiler_download.log" 2>&1
for p in "$d"/pkg/*.deb; do dpkg-deb -x "$p" "$d/root"; done
unshare -m --propagation private bash -s -- "$d" "$m" "$r" "$out" "$u" <<'INNER'
set -eux
d=$1
m=$2
r=$3
out=$4
u=$5
mkdir -p "$d/upper" "$d/work" "$d/merged"
mount -t overlay overlay -o "lowerdir=$d/root/usr:/usr,upperdir=$d/upper,workdir=$d/work" "$d/merged"
mount --bind "$d/merged" /usr
cc=x86_64-w64-mingw32-gcc-posix
"$cc" --version
"$cc" -O2 -shared -static-libgcc -DMATLAB_MEX_FILE -I"$m/include" -I"$u" -I"$r/control" -I"$r/integration"     "$out/mapper_gateway.c" "$m/version/c_mexapi_version.c" "$r/control/robot_ver2_kinematics.c"     "$r/integration/robot_ver2_control_adapter.c" "$u/HandGuiding/cartesian_joint_mapper.c" "$u/ControlCore/Math/matrix6.c" "$u/ControlCore/Analysis/singularity.c"     "$m/lib/win64/mingw64/exportsmexfileversion.def" "$m/lib/win64/mingw64/libmex.lib"     "$m/lib/win64/mingw64/libmx.lib" -o "$out/rb2_visible_mapper.mexw64"
x86_64-w64-mingw32-objdump -p "$out/rb2_visible_mapper.mexw64" | grep -E 'DLL Name|mexFunction|mexfilerequiredapi'
INNER