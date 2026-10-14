# ZMK 本地编译环境搭建 (Ubuntu 24.04 VM + 最新 ZMK/Zephyr)

目标: 在 Ubuntu 24.04 虚拟机里从零搭建一个能本地编译 HW-75 固件的
ZMK + Zephyr 4.x 工具链。改完代码跑 `west build` 出 `.uf2`,不用
再等 GitHub Actions。

---

## 0. 为什么换新版

旧 ZMK `zmkx-rel-20231018` 用 Zephyr 3.2.0,它的 USB HID class driver
(`subsys/usb/device/class/hid/core.c`) 把 HID 设备的 descriptor 放到
一个叫 `usb_hid_devlist` 的全局链表里,enumeration 时按 bInterfaceNumber
在链表里查。LampArray 的 race 就在这里 — ZMK 和 usb_comm 的
SYS_INIT 跑得够早能赶在 host 重试前完成,lamparray 的 SYS_INIT 永远赶不上。

新 ZMK (main) 拉 Zephyr 4.1+,USB stack 完全重构过
(`subsys/usb/device_next/`,前缀 `usbd_*`)。新的 `usbd_hid.c` 不再
用全局链表,HID Report descriptor 直接存在 per-instance 的
`ddata->rdesc` 里,`handle_get_descriptor` 直接读指针返回。**race 没了**。

---

## 1. 装 Ubuntu 24.04 VM

任意 VM 软件都行(VirtualBox / VMware Workstation Player / Hyper-V /
QEMU/KVM)。

最小配置:
- 4 核 CPU
- 8 GB 内存
- 60 GB 磁盘(SDK 占用大)
- 启用嵌套虚拟化和 EFI 启动

安装完后:
```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl wget unzip build-essential
```

---

## 2. 装 Zephyr 工具链

### 2.1 系统依赖

Zephyr 4.1 build 需要:
```bash
sudo apt install -y --no-install-recommends \
    git cmake ninja-build gperf \
    ccache dfu-util device-tree-compiler \
    python3-dev python3-pip python3-setuptools python3-tk python3-wheel \
    xz-utils file make gcc g++ libc6-dev libsdl2-dev \
    libmagic1
```

### 2.2 Python 工具

```bash
pip3 install --user --upgrade west
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
which west && west --version    # 应该输出 1.5+
```

### 2.3 Zephyr SDK 0.16+

ZMK 的 board file 要求 ARM 工具链。下载 0.16.5 (目前 ZMK 用的):
```bash
cd ~
wget -q https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_linux-x86_64_minimal.tar.xz
wget -q https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/sha256.sum.txt
sha256sum -c --ignore-missing sha256.sum.txt
tar -xJf zephyr-sdk-0.16.5_linux-x86_64_minimal.tar.xz
rm zephyr-sdk-0.16.5_linux-x86_64_minimal.tar.xz sha256.sum.txt

# 让 setup script 装 ARM GCC + binutils + qemu + 必要的 host tools
bash ~/zephyr-sdk-0.16.5/setup.sh
```

验证:
```bash
arm-zephyr-eabi-gcc --version    # 12.2.0
```

把 SDK 加入 PATH(setup.sh 已经会做):
```bash
cat >> ~/.bashrc << 'EOF'
export ZEPHYR_SDK_INSTALL_DIR=$HOME/zephyr-sdk-0.16.5
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_VERSION=0.16.5
EOF
```

---

## 3. 拉 ZMK workspace

### 3.1 改 west.yml

旧 `config/west.yml` 拉到的是 2023 年的 fork + Zephyr 3.2.0。改用上游:
```yaml
manifest:
  projects:
    - name: zmk
      url: https://github.com/zmkfirmware/zmk.git
      revision: main
      import: app/west.yml
  self:
    path: config
```

`app/west.yml` 是 ZMK 项目里那个(用来再拉 Zephyr)。`revision: main` 永远
拉最新代码,**CI 不稳时会 break**。要稳就 pin 某个 tag:
```bash
# 看最新稳定 tag
git ls-remote --tags https://github.com/zmkfirmware/zmk.git | tail -5
# 比如 revision: v0.4.0  或  revision: 2026-q2  之类的
```

确定版本后写进 west.yml。

### 3.2 west update

```bash
cd ~/hw75
git clone <your-fork-url>      # 你的 ZMK config 仓库
cd <your-config-repo>
west init -l config              # 'config' 是你 manifest 在的目录
west update                      # 拉 zmk + zephyr (耗时长,大约 5-10 分钟)
pip3 install --user -r zephyr/scripts/requirements.txt
```

workspace 布局:
```
~/hw75/<your-repo>/
├── config/              ← 你的 boards / keymap / app
├── zmk/                 ← ZMK 源码 (从 main 拉)
├── zephyr/              ← Zephyr 4.1+ 源码
└── modules/             ← 其他 west module
```

### 3.3 装 lvgl font tool (ZMK 用)

```bash
sudo apt install -y nodejs npm
sudo npm install -g lv_font_conv
```

---

## 4. 编译

CI 的 build 命令:
```yaml
- name: West build
  run: |
    LAMPA_FLAGS=""
    if [ "${{ matrix.lamparray }}" = "true" ]; then
      LAMPA_FLAGS="-DCONFIG_HW75_LAMPARRAY=y"
    fi
    west build -s zmk/app -b ${{ matrix.board }} -- \
      -DZMK_CONFIG=${GITHUB_WORKSPACE}/config \
      -DKEYMAP_FILE=${GITHUB_WORKSPACE}/config/${{ matrix.keymap }} \
      $LAMPA_FLAGS
```

本地对应的:
```bash
cd ~/hw75/<your-repo>

# 编译 keyboard 1.1 + LampArray
west build -s zmk/app -b hw75_keyboard@1.1 -- \
  -DZMK_CONFIG=$(pwd)/config \
  -DKEYMAP_FILE=$(pwd)/config/hw75_keyboard.keymap \
  -DCONFIG_HW75_LAMPARRAY=y

# 不带 LampArray
west build -s zmk/app -b hw75_keyboard@1.1 -- \
  -DZMK_CONFIG=$(pwd)/config \
  -DKEYMAP_FILE=$(pwd)/config/hw75_keyboard.keymap

# dynamic module
west build -s zmk/app -b hw75_dynamic@A -- \
  -DZMK_CONFIG=$(pwd)/config \
  -DKEYMAP_FILE=$(pwd)/config/hw75_dynamic.keymap \
  -DCONFIG_HW75_LAMPARRAY=y
```

产物在 `build/zephyr/`:
```
build/zephyr/zmk.uf2      ← 直接拖到 RP2 bootloader 刷机
build/zephyr/zmk.hex      ← 备用 (用 st-link 烧)
build/zephyr/zmk.bin      ← 原始 binary
build/zephyr/zmk.map      ← 内存布局 (debug 用)
```

刷机:
```bash
# 1. 按住键盘 BOOT 键(或短路 BOOT 引脚),插 USB,会冒出 RPI-RP2 盘
# 2. 拷贝
cp build/zephyr/zmk.uf2 /media/$USER/RPI-RP2/
sync
```

---

## 5. 跟旧版不一样的几个点

### 5.1 USB stack 完全换了

旧 (Zephyr 3.2):
- 头文件 `<zephyr/usb/class/usb_hid.h>`
- API: `usb_hid_register_device(dev, desc, size, &ops); usb_hid_init(dev);`
- 设备定义: `DEFINE_HID_DEVICE(x, _)` + `SYS_INIT(..., APPLICATION, ...)`

新 (Zephyr 4.1):
- 头文件 `<zephyr/usb/class/usbd_hid.h>`
- API: `usbd_hid_init(dev); hid_dev_register(dev, desc, size, &ops);`
- 设备定义: 全部用 devicetree,在 `.dts` / `.overlay` 里写 `zephyr,hid-device` binding

你的 `config/app/lamparray/lamparray_hid.c` 直接编译会挂,因为
用了旧 API。需要按新 API 重写,或用 ZMK 提供的 `zmk,usb-hid` 间接
binding (推荐 — 避免自己写 DT)。

### 5.2 Kconfig 选项名变了

旧: `CONFIG_USB_HID_DEVICE_COUNT` (整数) 改成了 devicetree 里的
`zephyr,hid-device` 节点数。
新: `CONFIG_USBD_HID` 之类,具体看 ZMK 4.x 默认。

`config/app/Kconfig.lamparray` 里的 `USB_HID_DEVICE_COUNT` 配套
`HW75_LAMPARRAY_DEVICE_NAME = "HID_2"` 的逻辑要改成 devicetree 节点
引用。

### 5.3 启动顺序的 race 没了

不用再 hack `usb_hid_devlist` 链表了。新 stack 的 HID class driver 在
`usbd_hid_init` 阶段就把 descriptor 装进 per-instance data,host 第一次
问就能拿到,没有 SYS_INIT 跟 enumeration 的 race window。

### 5.4 west 工作目录

新 `west build` 在 workspace 根目录跑,**不要** 在 `config/` 里面跑。
`config/` 是 manifest 目录,不是 source root。source root 是 `zmk/app`。

---

## 6. 已知会遇到的坑

### 6.1 zephyr SDK 在 WSL/VM 里很慢

第一次 build 大概要 5-15 分钟(交叉编译 + LLVM + newlib)。开 ccache:
```bash
export CCACHE_DIR=$HOME/.ccache
ccache -M 5G     # 限制最大 5 GB
west build --cmake-args=-DCMAKE_C_COMPILER_LAUNCHER=ccache \
            --cmake-args=-DCMAKE_CXX_COMPILER_LAUNCHER=ccache ...
```

`setup.sh` 在装 SDK 时已经自动装好 ccache。

### 6.2 west update 慢

第一次 `west update` 拉 ~2 GB 的源码。如果网络不稳,挂代理。
第二次增量更新就快。

### 6.3 权限

不要用 `sudo` 跑 `west build`。SDK 装在 `$HOME` 就行,west 也不需要 root。
如果不小心 sudo 装了 SDK,`chown -R $USER:$USER ~/zephyr-sdk-0.16.5` 改回来。

### 6.4 多个 west workspace

如果同时维护旧 config (Zephyr 3.2) 和新 config (Zephyr 4.1),
**两个 workspace 完全分开**,不要混。`west update` 会把所有 module
更新到 manifest 指定的版本,跨 workspace 不共享。

---

## 7. 调试 quick reference

```bash
# 看 .config 里某个 Kconfig 实际值
grep CONFIG_USBD_HID build/zephyr/.config

# 看 devicetree 编译后的产物
cat build/zephyr/zephyr.dts | less

# 清理 build(强制重新 cmake)
rm -rf build/
west build ...

# 增量清理(只清 build 不删 west modules)
west build -t clean
```

---

## 8. 跟 GitHub Actions CI 共享

为了避免 CI 和本地行为漂移,本地可以加一个 `Makefile` 复用 CI 的命令:
```makefile
# Makefile
BOARD ?= hw75_keyboard@1.1
KEYMAP ?= hw75_keyboard.keymap
LAMPA ?= 0

LAMPA_FLAGS :=
ifeq ($(LAMPA),1)
	LAMPA_FLAGS = -DCONFIG_HW75_LAMPARRAY=y
endif

.PHONY: build
build:
	west build -s zmk/app -b $(BOARD) -- \
		-DZMK_CONFIG=$(PWD)/config \
		-DKEYMAP_FILE=$(PWD)/config/$(KEYMAP) \
		$(LAMPA_FLAGS)

.PHONY: flash
flash:
	cp build/zephyr/zmk.uf2 /media/$$USER/RPI-RP2/
	sync

.PHONY: clean
clean:
	west build -t clean
```

用:
```bash
make build BOARD=hw75_keyboard@1.1 LAMPA=1
make flash
```

---

## 9. 下一步: 把 HW-75 config 迁到 ZMK main

1. `git remote add zmk-upstream https://github.com/zmkfirmware/zmk-config-template`
   拉一个官方 template 看看新 DTS / Kconfig 长什么样
2. 改 `boards/arm/hw75_keyboard/hw75_keyboard.dts` 用新 binding
3. 改 `Kconfig.defconfig` 去掉 USB_HID_DEVICE_COUNT 这种老 Kconfig
4. 改 `app/lamparray/lamparray_hid.c` 用新 API (usbd_hid_*)
5. 改 `app/usb_comm/usb_comm_hid.c` 同样
6. 第一次 build 后会有一堆 Kconfig 和 binding 错,一个个补

预计 1-2 天,主要是迁移不是写新代码。
