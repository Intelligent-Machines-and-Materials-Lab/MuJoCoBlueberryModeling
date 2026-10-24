# MuJoCo + OpenUSD on Ubuntu: working checkpoint

Note: this documentation written by Claude Sonnet 5.5 on 10/19/2026 to summarize how MuJoCo was built with CMake and OpenUSD support. (It kept crashing when I, Hannah, tried building on my own, so I asked for help. This is what got it working on the IMML ROG Zephyrus laptop.)

**Status:** `simulate` built from source with USD support, and a `.usda` file loaded by drag and drop.
**Machine:** Ubuntu 22.04 (jammy), GCC 11.4, CMake 4.4.4 (installed via pip), Python 3.10.12.
**Not done:** Python bindings with USD support (see "Not attempted / open questions").

MuJoCo's OpenUSD support is experimental and changes often, so pin or note the commit you built from:

```bash
cd ~/mujoco && git rev-parse HEAD
```

---

## 1. Prerequisites

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3 python3-dev \
    libgl1-mesa-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    libglew-dev
```

CMake check: `cmake --version` (this machine has 4.4.4; MuJoCo's top-level `CMakeLists.txt` required 3.16).

OpenUSD requres at least cmake 3.26; to upgrade, follow instructions at https://linuxcapable.com/how-to-install-cmake-on-ubuntu-linux/

Notes from `apt update` on this machine:

- `NO_PUBKEY FB0B24895113F120` for the librealsense repo: apt skipped that repo and used old indexes. Harmless for this build.
- "Key is stored in legacy trusted.gpg keyring" warnings (ROS 2, Microsoft): cosmetic.
- Skipped `i386` packages for the Microsoft repo: harmless.

## 2. Get MuJoCo

```bash
git clone https://github.com/google-deepmind/mujoco ~/mujoco
cd ~/mujoco
```

## 3. OpenUSD

MuJoCo's docs build USD with a helper CMake project:

```bash
cd ~/mujoco
cmake -Bcmake/third_party_deps/openusd/build cmake/third_party_deps/openusd
cmake --build cmake/third_party_deps/openusd/build
```

On this machine, the configure step in section 4 reported:

```
pxr_DIR:PATH=/home/imml/mujoco/build/_deps/openusd-build
```

and that folder already contained `pxrConfig.cmake`, `include`, `lib`, and `plugin`, so USD was already built there. On a fresh machine, run the helper build above first. If you use your own USD install, add `-Dpxr_DIR=/path/to/usd_install` to the configure command below.

## 4. Configure MuJoCo with USD

```bash
cd ~/mujoco
cmake -Bbuild -S. -DMUJOCO_WITH_USD=True
```

Verify:

```bash
grep -i -E "pxr|MUJOCO_WITH_USD" build/CMakeCache.txt
```

Expected: `MUJOCO_WITH_USD:BOOL=True` and a real `pxr_DIR` path.

The configure output has many warnings that are harmless: `FetchContent_Populate ... is deprecated` (CMP0169), `Ignored CHECK_PASS without CHECK_START`, the miniz `Compatibility with CMake < 3.10` notice, git `master` branch hints, and failed optional probes (LLD, FileCheck, `-Wshorten-64-to-32`, `-Wthread-safety`). The log ends with `Build files have been written to: /home/imml/mujoco/build`.

## 5. Build the targets that matter

Build only what is needed, not the tests. A GitHub issue (google-deepmind/mujoco #3695) reports `usd_decoder_test` failing to compile with `MUJOCO_WITH_USD=ON`.

```bash
cd ~/mujoco
cmake --build build --target simulate -j$(nproc)
cmake --build build --target usd_decoder_plugin usdMjcf -j$(nproc)
```

**Important:** `simulate` alone is not enough. The USD decoder is a separate plugin (`usd_decoder_plugin`). Without it, dropping a USD file gives:

```
could not find decoder for resource '...usda'
```

Verify the libraries exist:

```bash
ls build/lib | grep -i -E "usd|decoder"
ls build/lib/mujoco-usd-resources
```

Expected: `libusd_decoder_plugin.so`, `libusdMjcf.so`, and a `mujoco-usd-resources` folder containing `mjcPhysics`, `newton`, `usdMjcf`.

## 6. Put the decoder where `simulate` can find it (the fix that worked)

`simulate` did not load the plugin from `build/lib` on its own. Copy it into a `mujoco_plugin` folder next to the executable:

```bash
mkdir -p ~/mujoco/build/bin/mujoco_plugin
cp ~/mujoco/build/lib/libusd_decoder_plugin.so ~/mujoco/build/bin/mujoco_plugin/
```

Optional: use a symlink so rebuilds are picked up automatically:

```bash
ln -sf ~/mujoco/build/lib/libusd_decoder_plugin.so ~/mujoco/build/bin/mujoco_plugin/libusd_decoder_plugin.so
```

## 7. Run it

```bash
~/mujoco/build/bin/simulate
```

Drag a `.usd`, `.usda`, `.usdc`, or `.usdz` file into the window. This loaded `lpy_ufo_00000_skeleton (1).usda` successfully.

## Maintenance notes

- After `git pull` and a rebuild, rebuild `usd_decoder_plugin` too, and recopy it if you used `cp` instead of a symlink.
- If you delete `build/`, redo sections 4 through 6.
- For a clean reconfigure after a failure: `rm -rf ~/mujoco/build`, then repeat from section 4.
- Known upstream issues (open as of Oct 2026): the USD decoder drops some metadata (sites, model names, unlimited joints), and some tools can't load the USD decoder plugin without manual registration (#3389).

## Not attempted / open questions

- **Python bindings with USD support:** pip `mujoco` has no USD loading. Building the bindings from source against this build is documented in general (`make_sdist.sh`, then `MUJOCO_PATH` and `MUJOCO_PLUGIN_PATH` when running `pip install` on the sdist), but I could not confirm that the bindings use the USD decoder plugin. This step was skipped.
- **Authoring USD files:** `pip install mujoco-usd-converter` converts MJCF to USD. Keep it in a separate venv, since it and `usd-core` each vendor their own copy of USD.

## References

- MuJoCo docs, OpenUSD building: https://mujoco.readthedocs.io/en/latest/OpenUSD/building.html
- MuJoCo docs, extensions (decoder plugins): https://mujoco.readthedocs.io/en/stable/programming/extension.html
- Issue #3695 (USD tests and schema resources): https://github.com/google-deepmind/mujoco/issues/3695

## Extensions

- When working with a virtual environment or in VS code, make sure the mujoco being referenced is the one that was built with CMAKE and not the one made with pip!! If built with pip, it will not have the usd decoder that we just went through all the effort of installing. I don't know if this will fix all the problems, but one of the solutions was to put this in the venv bin/activate file: `export LD_LIBRARY_PATH=$HOME/mujoco/build/_deps/openusd-build/lib:$LD_LIBRARY_PATH` (where '$HOME/mujoco' is the location of the cmake-built mujoco installation). 