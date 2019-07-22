set -euv +o pipefail

mkdir -p .tools
pushd .tools
  wget "https://mirrors.edge.kernel.org/pub/tools/crosstool/files/bin/x86_64/8.1.0/x86_64-gcc-8.1.0-nolibc-mips-linux.tar.xz"
  tar xaf x86_64-gcc-8.1.0-nolibc-mips-linux.tar.xz

  export PATH="${PWD}/gcc-8.1.0-nolibc/mips-linux/bin:${PATH}"
popd

if [ ! -e .ub-tools/mkimage ]; then
  wget "ftp://ftp.denx.de/pub/u-boot/u-boot-2018.09.tar.bz2"
  tar xaf u-boot-2018.09.tar.bz2
  make -C u-boot-2018.09 sandbox_config
  make -C u-boot-2018.09 tools
  mkdir -p .ub-tools
  cp -v u-boot-2018.09/tools/mkimage .ub-tools/
  rm -rf u-boot-2018.09
fi

export PATH="${PWD}/.ub-tools:${PATH}"

make ARCH=mips CROSS_COMPILE=mips-linux- ${CONFIG}_defconfig
make ARCH=mips CROSS_COMPILE=mips-linux- -j$(nproc --all)

exit 0

br_ver="2018.02.4"

if [ "$(cat .root-images/br_ver 2>/dev/null)" != "${br_ver}" ]; then
  [ ! -d .root-images ] || echo "Wiping outdated RootFS cache"
  rm -rf .root-images
  mkdir -p .root-images
  echo "${br_ver}" >.root-images/br_ver
fi

gen_rootfs()
{
  cfg="$1"
  bin=".root-images/${cfg}.bin"

  if [ -e "${bin}" ]; then
    return 0
  fi

  mkdir -p .buildroot
  pushd .buildroot
    [ -e buildroot-${br_ver}.tar.bz2 ] || wget "https://buildroot.org/downloads/buildroot-${br_ver}.tar.bz2"
    rm -rf buildroot-${br_ver}
    tar xaf buildroot-${br_ver}.tar.bz2
    make -C buildroot-${br_ver} ${cfg}_defconfig
    make -C buildroot-${br_ver} | sed -r -e '/^.{4}>>>[[:space:]]+(.*).{5}$/!d; s//\1/;'
  popd

  mkdir -p .root-images
  cp -v .buildroot/buildroot-${br_ver}/output/images/rootfs.ext2 ${bin}
}

if grep -q 'CONFIG_64BIT=y' .config; then
  bits=64
else
  bits=32
fi

rfs_bits=${bits}
qemu_bits=${bits}

if grep -q 'CONFIG_CPU_MIPSR6=y' .config; then
  arch=6
elif grep -q 'CONFIG_CPU_MIPSR2=y' .config; then
  arch=2
  rfs_bits=32
else
  exit 0
fi

if grep -q 'CONFIG_CPU_LITTLE_ENDIAN=y' .config; then
  endian="el"
else
  endian=""
fi

case "${bits}r${arch}" in
32r2) qemu_cpu="34Kf";;
32r6) qemu_cpu="mips32r6-generic";;
64r2) qemu_cpu="MIPS64R2-generic";;
64r6) qemu_cpu="I6400";;
*) exit 0;;
esac

if grep -q 'CONFIG_FIT_IMAGE_FDT_BOSTON=y' .config; then
  qemu_mach="boston"
  qemu_kernel="arch/mips/boot/vmlinux.gz.itb"
  qemu_wait_exit='expect "reboot: System halted"'

  # QEMU only emulates I6400 / MIPS64r6el for Boston so far
  [ ${arch} -eq 6 ] || exit 0
  [ "${endian}" == "el" ] || exit 0
  qemu_bits=64
  qemu_cpu="I6400"
elif grep -q 'CONFIG_MIPS_MALTA=y' .config; then
  qemu_mach="malta"
  qemu_kernel="vmlinux"
  qemu_wait_exit="expect eof"

  # Malta EVA configs don't work on QEMU for some reason
  if grep -q 'CONFIG_EVA=y' .config; then
    exit 0
  fi

  # KVM guest kernels don't run on 'bare-metal'
  if grep -q 'CONFIG_KVM_GUEST=y' .config; then
    exit 0
  fi
else
  exit 0
fi

rfs_cfg="qemu_mips${rfs_bits}r${arch}${endian}_malta"
gen_rootfs ${rfs_cfg}

qemu="qemu-system-mips${qemu_bits/32/}${endian}"

if [ ! -e .qemu/bin/${qemu} ]; then
  if [ -e .qemu/${qemu} ]; then
    mkdir -p .qemu/bin
    mv -v .qemu/${qemu} .qemu/bin/
  fi
fi

if [ ! -e .qemu/bin/${qemu} ]; then
  mkdir -p .qemu/bin .qemu/pc-bios .qemu-build

  pushd .qemu-build
    wget "https://download.qemu.org/qemu-3.0.0.tar.xz"
    tar xaf qemu-3.0.0.tar.xz --strip-components=1
    ./configure --target-list=${qemu/qemu-system-/}-softmmu --disable-werror --disable-sdl --disable-gtk --disable-vnc --disable-tools
    make -j$(nproc --all)
  popd

  cp -v .qemu-build/${qemu/qemu-system-/}-softmmu/${qemu} .qemu/bin/
  cp -v .qemu-build/pc-bios/efi-pcnet.rom .qemu/pc-bios/
  cp -v .qemu-build/pc-bios/vgabios-cirrus.bin .qemu/pc-bios/
fi

if [ ! -e .qemu/pc-bios/efi-pcnet.rom ]; then
  mkdir -p .qemu/pc-bios .qemu-build

  pushd .qemu-build
    wget "https://download.qemu.org/qemu-3.0.0.tar.xz"
    tar xaf qemu-3.0.0.tar.xz --strip-components=1
  popd

  cp -v .qemu-build/pc-bios/efi-pcnet.rom .qemu/pc-bios/
fi

if [ ! -e .qemu/pc-bios/vgabios-cirrus.bin ]; then
  mkdir -p .qemu/pc-bios .qemu-build

  pushd .qemu-build
    wget "https://download.qemu.org/qemu-3.0.0.tar.xz"
    tar xaf qemu-3.0.0.tar.xz --strip-components=1
  popd

  cp -v .qemu-build/pc-bios/vgabios-cirrus.bin .qemu/pc-bios/
fi

export PATH="${PWD}/.qemu/bin:${PATH}"

mkdir -p .expect
pushd .expect
  apt-get download expect
  dpkg --fsys-tarfile expect*.deb | tar xOf - ./usr/bin/expect >expect
  dpkg --fsys-tarfile expect*.deb | tar xOf - ./usr/lib/libexpect.so.5.45 >libexpect.so.5.45
  chmod +x expect libexpect.so.5.45
  rm expect*.deb

  export PATH="${PWD}:${PATH}"
  export LD_LIBRARY_PATH="${PWD}"
popd

expect_test_fail()
{
  echo " failed!"
  [ ! -e test.log ] || cat test.log
  exit 1
}

expect_test()
{
  desc="$1"
  echo -n "Testing ${desc}..."
  expect <&0 || expect_test_fail
  echo " OK"
}

expect_test "QEMU ${qemu_mach} ${qemu_cpu} no-RFS" <<EOTEST
log_user 0
log_file -noappend -a test.log
spawn ${qemu} -M ${qemu_mach} -cpu ${qemu_cpu} -kernel ${qemu_kernel} -nographic
expect "VFS: Cannot open root device"
EOTEST

expect_test "QEMU ${qemu_mach} ${qemu_cpu} buildroot" <<EOTEST
log_user 0
log_file -noappend -a test.log
spawn ${qemu} -M ${qemu_mach} -cpu ${qemu_cpu} -kernel ${qemu_kernel} -nographic -hda .root-images/${rfs_cfg}.bin -append "root=/dev/sda"
expect "buildroot login: "
send "root\n"
expect "# "
send "uname -a\n"
expect "Linux buildroot"
expect "# "
send "sleep 2; poweroff\n"
${qemu_wait_exit}
EOTEST
