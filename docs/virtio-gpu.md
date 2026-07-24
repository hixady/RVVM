# VirtIO GPU (VirGL / Venus) in RVVM

*An explainer for the VirtIO transport and VirtIO-GPU device added to RVVM.*

---

## Background

### For the newcomer: what is a virtual GPU, really?

When a guest operating system runs inside an emulator, it still wants to draw
things: a boot logo, a desktop, a browser window. The emulator has to give the
guest *something* that looks like a graphics card. There are broadly three ways
to do this, in increasing order of sophistication:

1. **A dumb framebuffer.** The host hands the guest a slab of memory and says
   "whatever bytes you put here, I'll show on screen." This is what RVVM's
   existing `simple-framebuffer` and `bochs-display` devices do. It works, but
   the guest does *all* the drawing on its CPU. No acceleration.

2. **A paravirtual 2D device.** Instead of a fixed slab, the guest and host
   speak a small protocol: "create a 1280×720 surface", "copy these pixels into
   it", "show it on scanout 0". This decouples the guest's memory layout from
   the display and lets the host manage multiple surfaces. Still CPU rendering,
   but cleaner.

3. **A paravirtual 3D device.** The guest's OpenGL/Vulkan driver serializes GPU
   commands into a byte stream and ships them to the host, which replays them on
   a *real* GPU. This is real acceleration — glmark2, Weston, WebGL, even Vulkan
   all run at near-native speed.

> [!NOTE]
> **VirtIO** is the de-facto standard family of paravirtual device protocols
> (network, block, console, GPU, …). "Paravirtual" means the guest knows it is
> virtualized and cooperates, rather than the host pretending to be real
> hardware bit-for-bit. VirtIO-GPU is the graphics member of that family, and it
> is exactly what QEMU, crosvm, and cloud-hypervisor use for accelerated guest
> graphics.

**VirGL** and **Venus** are the two 3D dialects carried over VirtIO-GPU:

- **VirGL** encodes an OpenGL command stream (Gallium-based). The host decodes
  it with `libvirglrenderer` and runs it on host OpenGL.
- **Venus** encodes a Vulkan command stream. The same `libvirglrenderer`
  (built with Venus support) replays it on host Vulkan.

Crucially, from the device's point of view, **both look identical**: the guest
opens a *context*, creates *resources*, and submits opaque *command buffers*.
The only difference is which *capability set* (capset) the context requests. So
a single, correct 3D command path serves both.

### For the RVVM contributor: what already existed

RVVM has a mature device model that this work builds on rather than reinventing:

- **Region devices** (`<rvvm/rvvm_region.h>`): every MMIO window is a *region*
  with `read`/`write`/`poll`/`reset`/`cleanup` callbacks. The framework
  normalizes access sizes via `min_size`/`max_size`.
- **PCI bus** (`<rvvm/rvvm_pci.h>`): `rvvm_pci_func_init()` attaches a PCI
  function described by a `rvvm_pci_func_desc_t` (vendor/device IDs, BARs,
  IRQ pins/vectors). It provides **DMA** (`rvvm_pci_get_dma`) and **interrupts**
  (`rvvm_pci_set_irq`, routed automatically to INTx / MSI / MSI-X).
- **Framebuffer device** (`<rvvm/rvvm_fb.h>`): a `rvvm_fbdev_t` owns VRAM and a
  *scanout* (a `rvvm_fb_t` describing width/height/stride/format/buffer). The
  GUI window draws whatever the scanout points at. `bochs-display.c` is the
  model citizen here.

What was **missing**: any VirtIO transport at all, and the ability for a PCI
device to publish its **own capabilities** in config space.

> [!IMPORTANT]
> Modern (1.x) VirtIO-over-PCI is *defined* by four vendor-specific PCI
> capabilities that tell the driver where in the BARs to find the *common
> config*, *notification*, *ISR*, and *device config* regions. RVVM's PCI core
> hard-codes its capability chain (PCIe, PM, MSI, MSI-X) and had **no mechanism**
> for a device to add its own. The header declared a `.cap` field on
> `rvvm_pci_func_desc_t` but it was never wired up. Enabling it is the first
> domino.

---

## Intuition

### The transport: a mailbox and a ring buffer

Strip away the jargon and VirtIO is two ideas:

1. **A config mailbox** (the *common configuration*): a handful of registers
   where the driver negotiates features, points the device at queue memory, and
   flips the "I'm ready" switch (`DRIVER_OK`).

2. **Virtqueues**: shared-memory ring buffers. The guest allocates three arrays
   in its own RAM — a *descriptor table*, an *available ring*, and a *used ring*
   — and tells the device their physical addresses.

Picture the guest wanting to send one command:

```
Descriptor table (guest RAM):
  desc[0] = { addr=0x8000, len=24, flags=NEXT,  next=1 }   // command in
  desc[1] = { addr=0x9000, len=24, flags=WRITE, next=0 }   // response out

Available ring:            Used ring (device writes here):
  idx = 1                    idx = 0  -> becomes 1
  ring[0] = 0  (head=desc 0)  ring[0] = { id=0, len=24 }
```

The guest fills `desc[0]` with a command, points `desc[1]` at an empty response
buffer, publishes descriptor `0` in the available ring, bumps `avail.idx`, and
**kicks** the device by writing to the notification region. The device:

1. Sees `avail.idx (1) != last_avail (0)` → a new buffer is ready.
2. Walks the chain, DMA-mapping each descriptor's guest memory to a host pointer.
3. Reads the command from the *readable* buffers, writes a reply into the
   *writable* buffers.
4. Records `{ id=0, len=24 }` in the used ring, bumps `used.idx`, and raises an
   interrupt.

That is the entire engine. Everything else is bookkeeping.

### The GPU: surfaces, backing, and a blit

For 2D, the intuition is a game of "copy the pixels twice":

```
guest pixels ──TRANSFER_TO_HOST_2D──▶ host resource buffer ──RESOURCE_FLUSH──▶ framebuffer VRAM ──▶ window
```

- `RESOURCE_CREATE_2D` allocates a host-side pixel buffer for a surface.
- `RESOURCE_ATTACH_BACKING` tells us *where in guest RAM* that surface's pixels
  live (a scatter list of pages).
- `TRANSFER_TO_HOST_2D` copies guest pixels → host buffer.
- `SET_SCANOUT` says "surface 42 is what the monitor shows."
- `RESOURCE_FLUSH` blits the host buffer → the framebuffer VRAM the RVVM window
  already knows how to display.

A happy accident makes this cheap: the guest's default format
`B8G8R8X8_UNORM` has the exact byte layout `{B,G,R,X}` as RVVM's
`RVVM_RGB_XRGB8888`, so the blit is a straight `memcpy` per row.

### The 3D hand-off

For 3D, the device is a *postal service*, not a renderer. `SUBMIT_3D` hands an
opaque command buffer to `libvirglrenderer`, which does the real GPU work. Our
job is only to faithfully forward contexts, resources, transfers, and fences —
the same plumbing for VirGL and Venus alike.

---

## Code

The change is four new device files, one surgical core change, and build/CLI
wiring.

### 1. PCI core: let devices publish capabilities (`src/core/rvvm_pci.c`)

Modern VirtIO needs vendor capabilities in config space. The built-in chain ends
at MSI-X (`0xB0`–`0xBB`), leaving `0xBC`–`0xFF` (68 bytes) free — *exactly* the
size of the four VirtIO caps. The device supplies a read-only blob; the core
maps it into that window and links it onto the chain:

```c
#define PCI_DEV_CAP_OFFSET 0x000000BCUL // Device capability list offset
#define PCI_DEV_CAP_END    0x00000100UL // End of standard config space

// In pci_func_cfg_read(), before the register switch:
if (func->dev_cap && reg >= PCI_DEV_CAP_OFFSET && reg < PCI_DEV_CAP_END) {
    size_t cap_off = reg - PCI_DEV_CAP_OFFSET;
    if (cap_off < func->dev_cap_size) {
        return read_uint32_le_m(func->dev_cap + cap_off);
    }
    return 0;
}

// The MSI-X capability's "next" pointer now chains into the device window:
case PCI_REG_MSIX:
    if (func->dev_cap) {
        val |= (PCI_DEV_CAP_OFFSET << 8);
    }
    return val | atomic_load_uint32_relax(&func->msix_ctl);
```

`rvvm_pci_func_init()` captures the blob from the (previously unused) `desc->cap`
field. The whole change is additive and read-only — devices that don't set
`.cap` are completely unaffected.

### 2. The transport (`src/devices/virtio.{h,c}`)

A **generic, reusable** modern-VirtIO-over-PCI layer. `virtio_pci_init()`
lays out BAR0 as four page-sized windows and builds the capability blob:

```
BAR0:  0x0000 common cfg | 0x1000 ISR | 0x2000 device cfg | 0x3000 notify
caps:  common @0xBC → notify @0xCC → isr @0xE0 → device @0xF0 → end
```

The split-virtqueue engine is exposed to device code as a tiny API:

```c
virtio_request_t* virtio_queue_pop(virtio_dev_t*, uint16_t queue_id);
size_t virtio_request_read (virtio_request_t*, void* dst, size_t len);   // driver→device
size_t virtio_request_write(virtio_request_t*, const void* src, size_t len); // device→driver
void   virtio_request_complete(virtio_request_t*, uint32_t used_len);
void   virtio_queue_interrupt(virtio_dev_t*, uint16_t queue_id);
```

`virtio_queue_pop()` walks the descriptor chain (including `INDIRECT` tables),
DMA-maps every buffer, and hands back a request whose readable/writable halves
are exposed as simple sequential streams. Feature negotiation, `device_status`,
per-queue MSI-X vectors, and INTx/ISR fallback all live here so device code
never touches a ring index.

> [!WARNING]
> **Lock ordering.** The device's command handler holds its own lock and then
> calls into the transport (`gpu → vdev`). A naïve `device_status` reset would
> invoke `cb->reset()` *while holding* `vdev->lock` (`vdev → gpu`), an AB-BA
> deadlock. The transport therefore performs the internal reset under the lock
> but defers `cb->reset()` until **after** releasing it.

### 3. The device (`src/devices/virtio-gpu.{h,c}`)

Handles the 2D command set natively and forwards 3D to a renderer backend. The
control-queue drain is the heart of it:

```c
static void gpu_notify(virtio_dev_t* vdev, uint16_t queue_id) {
    virtio_gpu_t* gpu = virtio_dev_data(vdev);
    spin_lock(&gpu->lock);
    virtio_request_t* req;
    while ((req = virtio_queue_pop(vdev, queue_id)) != NULL) {
        if (queue_id == VIRTIO_GPU_CTRLQ) gpu_dispatch(gpu, req);
        else                              gpu_process_cursor(gpu, req);
    }
    spin_unlock(&gpu->lock);
    virtio_queue_interrupt(vdev, queue_id);
}
```

`TRANSFER_TO_HOST_2D` copies row-by-row from the guest backing scatter list into
the host resource buffer; `RESOURCE_FLUSH` blits the scanned-out resource into
framebuffer VRAM and calls `rvvm_fbdev_dirty()`/`rvvm_fbdev_update()`. The device
advertises `VIRTIO_GPU_F_VIRGL` **only** when a 3D backend initializes.

### 4. The 3D backend (`src/devices/virtio-gpu-virgl.c`, `USE_VIRGL`)

A thin bridge from the renderer interface to `libvirglrenderer`'s real API
(`virgl_renderer_init`, `_context_create_with_flags`, `_resource_create`,
`_transfer_write_iov`, `_submit_cmd`, `_create_fence`, …). It initializes a
surfaceless EGL renderer and probes capsets for VirGL2 and Venus. The whole file
is guarded by `#ifdef USE_VIRGL`, so the default build has zero new
dependencies.

### 5. Wiring

`rvvm_board.h` gains `rvvm_virtio_gpu_init()`; `main.c` gains a `-virtio_gpu`
flag; `project.mk` and `CMakeLists.txt` gain a `USE_VIRGL` useflag that links
`virglrenderer` via pkg-config.

---

## Verification

What was verified in this environment, and how you can reproduce it:

| Check | Command | Result |
|---|---|---|
| Builds clean, no warnings (gcc & clang) | `make CC=gcc` / `make CC=clang` | ✅ |
| Builds + **links** against real libvirglrenderer | `make USE_VIRGL=1` | ✅ (`ldd` shows `libvirglrenderer.so.1`) |
| Default build has no virgl dependency | `ldd librvvm.so \| grep virgl` | ✅ (absent) |
| Device attaches & machine tears down clean | AddressSanitizer smoke test | ✅ (0 leaks / 0 errors) |
| PCI capability change doesn't regress | full rebuild of `rvvm_pci.c` | ✅ |

The AddressSanitizer smoke test creates a machine, attaches the VirtIO-GPU to
the PCI bus (it lands at bus address `0x8`), and frees the machine — exercising
capability-blob construction, the PCI core change, fbdev ref-counting, and the
full cleanup path with **zero** leaks or use-after-frees.

**Manual QA with a real guest** (needs a host GPU + a RISC-V Linux image with
the `virtio_gpu` DRM driver and Mesa):

1. Build: `make USE_VIRGL=1`
2. Run: `rvvm fw_payload.bin -i rootfs.img -m 2G -smp 4 -virtio_gpu`
3. In the guest, confirm `/dev/dri/card0` appears and `dmesg | grep virtio_gpu`
   shows the device bound. A framebuffer console / Weston should display.
4. For 3D: `glmark2-es2-drm` or `weston --backend=drm` should run accelerated;
   `MESA_LOADER_DRIVER_OVERRIDE=virpipe glxinfo` should report a *virgl* renderer.

> [!NOTE]
> The 2D path is verified by construction and review; the 3D (VirGL/Venus) path
> compiles and links against the real `libvirglrenderer` API but has **not** been
> run end-to-end here, because that requires a host GPU/EGL stack and a
> Mesa-enabled RISC-V guest image that this sandbox lacks. The backend is
> structured (surfaceless EGL, synchronous fences) so a developer with a GPU can
> finish bring-up without touching the transport or 2D code.

---

## Alternatives

### A. Extend the PCI core (chosen) vs. legacy VirtIO transport

| | Extend PCI core for modern VirtIO caps (chosen) | Use legacy (0.9.5) VirtIO transport |
|---|---|---|
| **Pros** | Spec-current; required by VirtIO-GPU (no legacy interface exists); reusable by future VirtIO devices; tiny, additive, read-only core change | No PCI-core change; simpler config space |
| **Cons** | Touches shared PCI core (needs maintainer review) | **VirtIO-GPU has no legacy interface** — it simply would not work; deprecated; awkward I/O-BAR register model |

VirtIO-GPU is a VirtIO-1.0-only device, so the legacy path is a dead end. The
core change is the honest minimum.

### B. Native renderer vs. libvirglrenderer for 3D

| | Bridge to libvirglrenderer (chosen) | Write a native GL/Vulkan translator |
|---|---|---|
| **Pros** | Reuses the same battle-tested library QEMU/crosvm use; VirGL **and** Venus for free; tracks Mesa | No external dependency; full control |
| **Cons** | Runtime dependency on virglrenderer + host EGL | Enormous effort; must re-implement Gallium/Vulkan decoding; effectively re-writing virglrenderer |

---

## Suggested people to talk to

- **LekKit** — author of the PCI core (`src/core/rvvm_pci.c`), the region API,
  and the framebuffer device. The capability-injection change lives in their
  code, and the transport leans on their DMA/IRQ/fbdev contracts. Best person to
  sanity-check the config-space change and the ownership/lifetime model.
- **David Korenchuk (epoll-reactor-2)** — author of the `gpu-xe2` GPU device,
  the most recent and most complex display device in the tree. Deep context on
  DMA-pointer lifetimes, fbdev scanout wiring, and PCI GPU quirks that the
  VirtIO-GPU device shares.

---

## Quiz

<details>
<summary><b>1.</b> Why must the device-provided capability window start at <code>0xBC</code> rather than the documented <code>0xC0</code>?</summary>

- **A.** `0xC0` is reserved by the PCIe spec. — *Incorrect; it is free.*
- **B.** The four VirtIO caps total 68 bytes, and `0x100 − 68 = 0xBC`; starting
  at `0xC0` would spill 4 bytes past `0xFF` into extended config space. — ✅
  **Correct.** MSI-X ends at `0xBB`, so `0xBC` is the earliest free dword and the
  only start that fits all four caps within standard config space.
- **C.** MSI-X requires it. — *Incorrect; MSI-X occupies `0xB0`–`0xBB`.*
- **D.** Alignment. — *Partly relevant (`0xBC` is dword-aligned) but not the
  reason.*
</details>

<details>
<summary><b>2.</b> When the guest kicks the notification region, which thread processes the command, and what protects the resource list?</summary>

The write handler runs on the **vCPU thread** that performed the store. Because
the region framework may invoke handlers concurrently on multiple vCPUs, the
device holds `gpu->lock` across the whole control-queue drain, and the transport
holds `vdev->lock` for ring bookkeeping. There is **no** background render
thread in this design.
</details>

<details>
<summary><b>3.</b> How does the code avoid the AB-BA deadlock between the GPU lock and the transport lock?</summary>

`gpu_notify` takes `gpu->lock` then `vdev->lock` (via `virtio_queue_pop`). A
`device_status = 0` reset arrives holding `vdev->lock` and needs to call
`cb->reset` → `gpu->lock` — the opposite order. The fix: the transport performs
the internal state reset under `vdev->lock`, returns a "reset requested" flag,
and invokes `cb->reset()` **after** unlocking. Both lock nestings are therefore
consistently `gpu → vdev`.
</details>

<details>
<summary><b>4.</b> Why does a 2D <code>RESOURCE_FLUSH</code> of a <code>B8G8R8X8</code> surface reduce to a per-row <code>memcpy</code>?</summary>

`VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM` stores bytes as `{B,G,R,X}`, which is byte-for
-byte identical to RVVM's `RVVM_RGB_XRGB8888`. No channel swizzle or conversion
is needed, so flushing is just copying rows from the host resource buffer into
framebuffer VRAM.
</details>

<details>
<summary><b>5.</b> From the device's perspective, what actually differs between a VirGL (OpenGL) and a Venus (Vulkan) workload?</summary>

Almost nothing. Both open a context, create resources, and submit opaque command
buffers via `SUBMIT_3D`. The distinguishing factor is the **capability set** the
context requests (`VIRTIO_GPU_CAPSET_VIRGL2` vs `VIRTIO_GPU_CAPSET_VENUS`), which
`libvirglrenderer` uses to pick its decoder. That is why one 3D command path
serves both — and why the backend just needs `context_create_with_flags` to pass
the capset through.
</details>
