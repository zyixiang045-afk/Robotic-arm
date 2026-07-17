# MuJoCo 3.10.0 部署说明

**在linux系统终端中操作。**

## 1. 下载 MuJoCo native release

在用户目录下载官方 Linux x86_64 压缩包：

```bash
cd ~
curl -L --fail --retry 3 \
  -o mujoco-3.10.0-linux-x86_64.tar.gz \
  https://github.com/google-deepmind/mujoco/releases/download/3.10.0/mujoco-3.10.0-linux-x86_64.tar.gz
```

解压：

```bash
tar -xzf ~/mujoco-3.10.0-linux-x86_64.tar.gz -C ~
```

解压完成后目录为：

```bash
~/mujoco-3.10.0
```

关键文件包括：

```bash
~/mujoco-3.10.0/bin
~/mujoco-3.10.0/include
~/mujoco-3.10.0/lib/libmujoco.so
~/mujoco-3.10.0/model
~/mujoco-3.10.0/sample
```

## 2. 配置 Bash 环境

将下面内容加入 `~/.bashrc`：

```bash
# MuJoCo 3.10.0
export MUJOCO_HOME="$HOME/mujoco-3.10.0"
export PATH="$MUJOCO_HOME/bin:$PATH"
export CPATH="$MUJOCO_HOME/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$MUJOCO_HOME/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$MUJOCO_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PKG_CONFIG_PATH="$HOME/.local/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
```

使配置在当前终端立即生效：

```bash
source ~/.bashrc
```

配置后，MuJoCo 命令行工具可以直接运行：

```bash
basic
simulate
compile
testspeed
record
```

## 3. 配置 pkg-config

创建用户态 pkg-config 文件：

```bash
mkdir -p ~/.local/lib/pkgconfig
cat > ~/.local/lib/pkgconfig/mujoco.pc <<'EOF'
prefix=/home/nightflower/mujoco-3.10.0
exec_prefix=${prefix}
libdir=${exec_prefix}/lib
includedir=${prefix}/include

Name: mujoco
Description: MuJoCo physics engine
Version: 3.10.0
Cflags: -I${includedir}
Libs: -L${libdir} -Wl,-rpath,${libdir} -lmujoco -ldl -pthread
EOF
```

验证：

```bash
pkg-config --modversion mujoco
pkg-config --cflags --libs mujoco
```

## 4. C++ 编译

由于 `CPATH`、`LIBRARY_PATH` 和 `LD_LIBRARY_PATH` 已在 `~/.bashrc` 中配置，普通 C++ 程序不再需要手写 `-I...` 和 `-L...` 路径。

MuJoCo 3.10.0 头文件需要 C++17，因此编译时保留 `-std=c++17`：

```bash
g++ -std=c++17 your_program.cpp -o your_program -lmujoco -ldl -pthread
```

也可以使用 pkg-config：

```bash
g++ -std=c++17 your_program.cpp -o your_program $(pkg-config --cflags --libs mujoco)
```

当前机器已验证无窗口 C++ 程序可以链接并运行 MuJoCo 3.10.0：

```text
MuJoCo 3.10.0: nq=28 nv=27 time=0.005
```

注意：MuJoCo 官方 `sample/basic.cc` 依赖 GLFW 头文件。如果要编译图形窗口示例，还需要安装 GLFW/OpenGL 相关开发包。

## 5. Python 导入

native release 压缩包只提供 C/C++ 头文件、动态库、模型和示例程序，不会自动安装 Python 包。

当前系统 Python 是 3.8。PyPI 上本环境可用的最高 `mujoco` Python 包版本是 `3.2.3`，没有 `mujoco==3.10.0` 可安装。因此当前方案是：

- C++ / 命令行工具使用 native MuJoCo `3.10.0`
- Python `import mujoco` 使用 PyPI 包 `mujoco==3.2.3`

安装 Python 包：

```bash
python3 -m pip install --user --upgrade 'mujoco==3.2.3' 'zipp>=3.1.0'
```

验证：

```bash
python3 - <<'PY'
import mujoco
print(mujoco.__version__)
print(mujoco.__file__)
PY
```

当前机器已验证：

```text
3.2.3
/home/nightflower/.local/lib/python3.8/site-packages/mujoco/__init__.py
```

如果后续切换到更新的 Python 环境，可重新检查 PyPI 是否已有对应版本，再安装匹配的 Python 包。
