/*
virtio-gpu.c - VirtIO GPU device (2D + VirGL/Venus 3D acceleration)
Copyright (C) 2026  RVVM Contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/*
 * Implements the VirtIO GPU device on top of the modern VirtIO PCI transport.
 *
 * The 2D command set is handled natively and drives the RVVM framebuffer:
 * guest resources are backed by host pixel buffers, TRANSFER_TO_HOST_2D copies
 * guest pixel data into them, and RESOURCE_FLUSH blits the scanned-out resource
 * into framebuffer VRAM for display.
 *
 * The 3D command set (contexts, 3D resources, SUBMIT_3D, ...) is forwarded to
 * an optional renderer backend (libvirglrenderer) that dispatches OpenGL
 * (virgl) or Vulkan (venus) to the host GPU. Without a backend the device runs
 * 2D-only and does not advertise VIRTIO_GPU_F_VIRGL.
 */

#include <rvvm/rvvm_board.h>
#include <rvvm/rvvm_fb.h>
#include <rvvm/rvvm_pci.h>

#include <util/mem_ops.h>
#include <util/spinlock.h>
#include <util/utils.h>

#include "compiler.h"
#include "virtio.h"
#include "virtio-gpu.h"

PUSH_OPTIMIZATION_SIZE

#define VIRTIO_GPU_CTRLQ 0
#define VIRTIO_GPU_CURSORQ 1

#define VIRTIO_GPU_CTRL_HDR_SIZE 24

/*
 * Parsed control header
 */
typedef struct {
    uint32_t type;
    uint32_t flags;
    uint64_t fence_id;
    uint32_t ctx_id;
    uint8_t  ring_idx;
} gpu_ctrl_hdr_t;

/*
 * Guest backing scatter/gather entry
 */
typedef struct {
    uint64_t addr;
    uint32_t len;
} gpu_backing_t;

/*
 * A GPU resource (2D backing or 3D renderer-owned)
 */
typedef struct {
    uint32_t id; // 0 == free slot
    uint32_t format;
    uint32_t width;
    uint32_t height;
    uint8_t* data;      // Host linear pixel storage (2D resources)
    size_t   data_size;
    bool     is_3d;

    // Guest backing scatter list
    gpu_backing_t* backing;
    uint32_t       backing_cnt;
} gpu_resource_t;

/*
 * A display scanout
 */
typedef struct {
    uint32_t resource_id; // 0 == disabled
    uint32_t x, y, w, h;
} gpu_scanout_t;

struct virtio_gpu {
    virtio_dev_t* vdev;
    rvvm_fbdev_t* fbdev;

    gpu_resource_t* res;
    uint32_t        res_cap;
    uint32_t        res_cnt;

    uint32_t      num_scanouts;
    gpu_scanout_t scanout[VIRTIO_GPU_MAX_SCANOUTS];

    uint32_t disp_width;
    uint32_t disp_height;

    uint32_t events_read;

    const virtio_gpu_renderer_t* renderer;
    bool                         renderer_ok;

    spinlock_t lock;
};

/*
 * Pixel format helpers
 */

static rvvm_rgb_t gpu_format_to_rvvm(uint32_t format)
{
    switch (format) {
        case VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM:
        case VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM:
            return RVVM_RGB_XRGB8888; // Memory layout {B,G,R,X}
        case VIRTIO_GPU_FORMAT_R8G8B8A8_UNORM:
        case VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM:
            return RVVM_RGB_XBGR8888; // Memory layout {R,G,B,X}
        default:
            return RVVM_RGB_XRGB8888;
    }
}

/*
 * Resource management
 */

static gpu_resource_t* gpu_res_find(virtio_gpu_t* gpu, uint32_t id)
{
    if (!id) {
        return NULL;
    }
    for (uint32_t i = 0; i < gpu->res_cnt; ++i) {
        if (gpu->res[i].id == id) {
            return &gpu->res[i];
        }
    }
    return NULL;
}

static gpu_resource_t* gpu_res_alloc(virtio_gpu_t* gpu, uint32_t id)
{
    // Reuse a free slot if any
    for (uint32_t i = 0; i < gpu->res_cnt; ++i) {
        if (gpu->res[i].id == 0) {
            gpu_resource_t* r = &gpu->res[i];
            memset(r, 0, sizeof(*r));
            r->id = id;
            return r;
        }
    }
    if (gpu->res_cnt == gpu->res_cap) {
        uint32_t new_cap = gpu->res_cap ? gpu->res_cap * 2 : 8;
        gpu->res         = safe_realloc(gpu->res, new_cap * sizeof(gpu_resource_t));
        memset(gpu->res + gpu->res_cap, 0, (new_cap - gpu->res_cap) * sizeof(gpu_resource_t));
        gpu->res_cap = new_cap;
    }
    gpu_resource_t* r = &gpu->res[gpu->res_cnt++];
    memset(r, 0, sizeof(*r));
    r->id = id;
    return r;
}

static void gpu_res_free(virtio_gpu_t* gpu, gpu_resource_t* r)
{
    if (r->is_3d && gpu->renderer_ok && gpu->renderer->resource_unref) {
        gpu->renderer->resource_unref(gpu, r->id);
    }
    free(r->data);
    free(r->backing);
    memset(r, 0, sizeof(*r));
}

/*
 * Read `len` bytes from a resource's guest backing starting at byte `off`.
 */
static size_t gpu_backing_read(virtio_gpu_t* gpu, gpu_resource_t* r, uint64_t off, void* dst, size_t len)
{
    uint8_t* out  = dst;
    size_t   done = 0;
    uint64_t cur  = 0; // Running start offset of the current backing entry
    for (uint32_t i = 0; i < r->backing_cnt && done < len; ++i) {
        uint64_t entry_start = cur;
        uint64_t entry_end   = cur + r->backing[i].len;
        cur                  = entry_end;
        if (off >= entry_end) {
            continue; // Entirely before the requested offset
        }
        uint64_t within = (off > entry_start) ? (off - entry_start) : 0;
        size_t   chunk  = EVAL_MIN((size_t)(r->backing[i].len - within), len - done);
        void*    p      = virtio_dma_get(gpu->vdev, r->backing[i].addr + within, chunk);
        if (p) {
            memcpy(out + done, p, chunk);
            virtio_dma_put(gpu->vdev, p);
        } else {
            memset(out + done, 0, chunk);
        }
        done += chunk;
        off += chunk;
    }
    return done;
}

/*
 * Blit a resource's flushed region into framebuffer VRAM and refresh display.
 */
static void gpu_flush_to_display(virtio_gpu_t* gpu, gpu_scanout_t* sc, gpu_resource_t* r)
{
    size_t vram_size = 0;
    uint8_t* vram    = rvvm_fbdev_get_vram(gpu->fbdev, &vram_size);
    if (!vram || !r->data) {
        return;
    }

    const uint32_t bpp = 4;
    // Nothing visible if the scanout origin is outside the resource
    if (sc->x >= r->width || sc->y >= r->height) {
        return;
    }

    uint32_t res_stride = r->width * bpp;
    uint32_t out_stride = sc->w * bpp;                   // Scanout (VRAM) stride
    uint32_t copy_w     = EVAL_MIN(sc->w, r->width - sc->x) * bpp;
    uint32_t out_h      = EVAL_MIN(sc->h, r->height - sc->y);

    // Clamp to available VRAM
    if ((size_t)out_stride * sc->h > vram_size) {
        return;
    }

    for (uint32_t row = 0; row < out_h; ++row) {
        size_t src_off = (size_t)(sc->y + row) * res_stride + (size_t)sc->x * bpp;
        size_t dst_off = (size_t)row * out_stride;
        if (src_off + copy_w <= r->data_size && dst_off + copy_w <= vram_size) {
            memcpy(vram + dst_off, r->data + src_off, copy_w);
        }
    }

    // Mark VRAM dirty; the actual display redraw happens on the event-loop
    // thread via the poll callback (rvvm_fbdev_update).
    rvvm_fbdev_dirty(gpu->fbdev);
}

/*
 * Configure the framebuffer scanout for a resource
 */
static void gpu_set_scanout(virtio_gpu_t* gpu, gpu_scanout_t* sc, gpu_resource_t* r)
{
    rvvm_fb_t fb        = ZERO_INIT;
    size_t    vram_size = 0;
    void*     vram      = rvvm_fbdev_get_vram(gpu->fbdev, &vram_size);

    fb.width  = sc->w;
    fb.height = sc->h;
    fb.stride = sc->w * 4;
    fb.format = gpu_format_to_rvvm(r->format);
    fb.buffer = vram;

    if (rvvm_fb_size(&fb) && rvvm_fb_size(&fb) <= vram_size) {
        rvvm_fbdev_set_scanout(gpu->fbdev, &fb);
    }
}

/*
 * Control header helpers
 */

static bool gpu_read_hdr(virtio_request_t* req, gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[VIRTIO_GPU_CTRL_HDR_SIZE];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        return false;
    }
    hdr->type     = read_uint32_le_m(buf + 0);
    hdr->flags    = read_uint32_le_m(buf + 4);
    hdr->fence_id = read_uint64_le_m(buf + 8);
    hdr->ctx_id   = read_uint32_le_m(buf + 16);
    hdr->ring_idx = buf[20];
    return true;
}

static size_t gpu_build_resp_hdr(uint8_t* buf, uint32_t type, const gpu_ctrl_hdr_t* cmd)
{
    memset(buf, 0, VIRTIO_GPU_CTRL_HDR_SIZE);
    write_uint32_le_m(buf + 0, type);
    uint32_t flags = 0;
    if (cmd->flags & VIRTIO_GPU_FLAG_FENCE) {
        flags |= VIRTIO_GPU_FLAG_FENCE;
        write_uint64_le_m(buf + 8, cmd->fence_id);
        write_uint32_le_m(buf + 16, cmd->ctx_id);
        buf[20] = cmd->ring_idx;
    }
    write_uint32_le_m(buf + 4, flags);
    return VIRTIO_GPU_CTRL_HDR_SIZE;
}

// Write a response that is just a control header, complete the request
static void gpu_respond(virtio_gpu_t* gpu, virtio_request_t* req, uint32_t type, const gpu_ctrl_hdr_t* cmd)
{
    uint8_t buf[VIRTIO_GPU_CTRL_HDR_SIZE];
    gpu_build_resp_hdr(buf, type, cmd);
    size_t written = virtio_request_write(req, buf, sizeof(buf));

    // Settle fence synchronously after the command has been executed
    if ((cmd->flags & VIRTIO_GPU_FLAG_FENCE) && gpu->renderer_ok && gpu->renderer->create_fence) {
        gpu->renderer->create_fence(gpu, cmd->ctx_id, cmd->ring_idx, cmd->fence_id);
    }

    virtio_request_complete(req, (uint32_t)written);
    UNUSED(gpu);
}

/*
 * 2D command handlers
 */

static void gpu_cmd_get_display_info(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t resp[VIRTIO_GPU_CTRL_HDR_SIZE + VIRTIO_GPU_MAX_SCANOUTS * 24];
    memset(resp, 0, sizeof(resp));
    gpu_build_resp_hdr(resp, VIRTIO_GPU_RESP_OK_DISPLAY_INFO, hdr);

    // pmodes[]: virtio_gpu_display_one { rect(x,y,w,h); enabled; flags }
    for (uint32_t i = 0; i < gpu->num_scanouts; ++i) {
        uint8_t* p = resp + VIRTIO_GPU_CTRL_HDR_SIZE + i * 24;
        if (i == 0) {
            write_uint32_le_m(p + 0, 0);                 // x
            write_uint32_le_m(p + 4, 0);                 // y
            write_uint32_le_m(p + 8, gpu->disp_width);   // width
            write_uint32_le_m(p + 12, gpu->disp_height); // height
            write_uint32_le_m(p + 16, 1);                // enabled
            write_uint32_le_m(p + 20, 0);                // flags
        }
    }

    size_t written = virtio_request_write(req, resp, sizeof(resp));
    virtio_request_complete(req, (uint32_t)written);
}

static void gpu_cmd_resource_create_2d(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[16];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t res_id = read_uint32_le_m(buf + 0);
    uint32_t format = read_uint32_le_m(buf + 4);
    uint32_t width  = read_uint32_le_m(buf + 8);
    uint32_t height = read_uint32_le_m(buf + 12);

    if (!res_id || !width || !height || width > 16384 || height > 16384) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    if (gpu_res_find(gpu, res_id)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID, hdr);
        return;
    }

    gpu_resource_t* r = gpu_res_alloc(gpu, res_id);
    r->format         = format;
    r->width          = width;
    r->height         = height;
    r->data_size      = (size_t)width * height * 4;
    r->data           = safe_calloc(r->data_size, 1);
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_resource_unref(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[8];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t        res_id = read_uint32_le_m(buf + 0);
    gpu_resource_t* r      = gpu_res_find(gpu, res_id);
    if (!r) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID, hdr);
        return;
    }
    gpu_res_free(gpu, r);
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_set_scanout(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[24];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t x           = read_uint32_le_m(buf + 0);
    uint32_t y           = read_uint32_le_m(buf + 4);
    uint32_t w           = read_uint32_le_m(buf + 8);
    uint32_t h           = read_uint32_le_m(buf + 12);
    uint32_t scanout_id  = read_uint32_le_m(buf + 16);
    uint32_t resource_id = read_uint32_le_m(buf + 20);

    if (scanout_id >= gpu->num_scanouts) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID, hdr);
        return;
    }

    gpu_scanout_t* sc = &gpu->scanout[scanout_id];
    if (resource_id == 0) {
        // Disable scanout
        sc->resource_id = 0;
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
        return;
    }

    gpu_resource_t* r = gpu_res_find(gpu, resource_id);
    if (!r) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID, hdr);
        return;
    }

    sc->resource_id = resource_id;
    sc->x           = x;
    sc->y           = y;
    sc->w           = w;
    sc->h           = h;
    gpu_set_scanout(gpu, sc, r);
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_resource_flush(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[24];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t resource_id = read_uint32_le_m(buf + 16);

    gpu_resource_t* r = gpu_res_find(gpu, resource_id);
    if (!r) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID, hdr);
        return;
    }

    for (uint32_t i = 0; i < gpu->num_scanouts; ++i) {
        if (gpu->scanout[i].resource_id == resource_id) {
            gpu_flush_to_display(gpu, &gpu->scanout[i], r);
        }
    }
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_transfer_to_host_2d(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[32];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t rx          = read_uint32_le_m(buf + 0);
    uint32_t ry          = read_uint32_le_m(buf + 4);
    uint32_t rw          = read_uint32_le_m(buf + 8);
    uint32_t rh          = read_uint32_le_m(buf + 12);
    uint64_t offset      = read_uint64_le_m(buf + 16);
    uint32_t resource_id = read_uint32_le_m(buf + 24);

    gpu_resource_t* r = gpu_res_find(gpu, resource_id);
    if (!r || !r->data) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID, hdr);
        return;
    }

    uint32_t bpp    = 4;
    uint32_t stride = r->width * bpp;
    // Clamp the destination rectangle to the resource bounds
    if (rx >= r->width || ry >= r->height) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
        return;
    }
    uint32_t clamp_w = EVAL_MIN(rw, r->width - rx);
    uint32_t clamp_h = EVAL_MIN(rh, r->height - ry);
    for (uint32_t row = 0; row < clamp_h; ++row) {
        uint64_t src_off = offset + (uint64_t)row * stride;
        size_t   dst_off = (size_t)(ry + row) * stride + (size_t)rx * bpp;
        size_t   copy_w  = (size_t)clamp_w * bpp;
        if (dst_off + copy_w <= r->data_size) {
            gpu_backing_read(gpu, r, src_off, r->data + dst_off, copy_w);
        }
    }
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_attach_backing(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[8];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t        res_id      = read_uint32_le_m(buf + 0);
    uint32_t        nr_entries  = read_uint32_le_m(buf + 4);
    gpu_resource_t* r           = gpu_res_find(gpu, res_id);
    if (!r || nr_entries == 0 || nr_entries > 16384) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }

    free(r->backing);
    r->backing     = safe_new_arr(gpu_backing_t, nr_entries);
    r->backing_cnt = nr_entries;

    for (uint32_t i = 0; i < nr_entries; ++i) {
        uint8_t ent[16];
        if (virtio_request_read(req, ent, sizeof(ent)) != sizeof(ent)) {
            r->backing_cnt = i;
            break;
        }
        r->backing[i].addr = read_uint64_le_m(ent + 0);
        r->backing[i].len  = read_uint32_le_m(ent + 8);
    }

    // For 3D resources, hand the backing to the renderer as an iovec list
    if (r->is_3d && gpu->renderer_ok && gpu->renderer->resource_attach_backing) {
        virtio_gpu_iovec_t* iov = safe_new_arr(virtio_gpu_iovec_t, r->backing_cnt);
        uint32_t            n   = 0;
        for (uint32_t i = 0; i < r->backing_cnt; ++i) {
            void* p = virtio_dma_get(gpu->vdev, r->backing[i].addr, r->backing[i].len);
            if (p) {
                iov[n].base = p;
                iov[n].len  = r->backing[i].len;
                n++;
            }
        }
        gpu->renderer->resource_attach_backing(gpu, res_id, iov, n);
        free(iov);
    }

    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_detach_backing(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[8];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t        res_id = read_uint32_le_m(buf + 0);
    gpu_resource_t* r      = gpu_res_find(gpu, res_id);
    if (!r) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID, hdr);
        return;
    }
    if (r->is_3d && gpu->renderer_ok && gpu->renderer->resource_detach_backing) {
        gpu->renderer->resource_detach_backing(gpu, res_id);
    }
    free(r->backing);
    r->backing     = NULL;
    r->backing_cnt = 0;
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

/*
 * 3D capset queries
 */

static void gpu_cmd_get_capset_info(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[8];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t index = read_uint32_le_m(buf + 0);

    uint32_t id = 0, max_ver = 0, max_size = 0;
    if (gpu->renderer_ok && gpu->renderer->capset_info) {
        gpu->renderer->capset_info(gpu, index, &id, &max_ver, &max_size);
    }

    uint8_t resp[VIRTIO_GPU_CTRL_HDR_SIZE + 16];
    memset(resp, 0, sizeof(resp));
    gpu_build_resp_hdr(resp, VIRTIO_GPU_RESP_OK_CAPSET_INFO, hdr);
    write_uint32_le_m(resp + VIRTIO_GPU_CTRL_HDR_SIZE + 0, id);
    write_uint32_le_m(resp + VIRTIO_GPU_CTRL_HDR_SIZE + 4, max_ver);
    write_uint32_le_m(resp + VIRTIO_GPU_CTRL_HDR_SIZE + 8, max_size);
    size_t written = virtio_request_write(req, resp, sizeof(resp));
    virtio_request_complete(req, (uint32_t)written);
}

static void gpu_cmd_get_capset(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[8];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t capset_id  = read_uint32_le_m(buf + 0);
    uint32_t capset_ver = read_uint32_le_m(buf + 4);

    // Determine capset size via renderer
    uint32_t max_size = 0, id = 0;
    if (gpu->renderer_ok && gpu->renderer->capset_info) {
        // Look up by matching id across advertised capsets
        uint32_t n = gpu->renderer->num_capsets ? gpu->renderer->num_capsets(gpu) : 0;
        for (uint32_t i = 0; i < n; ++i) {
            uint32_t cid = 0, cver = 0, csize = 0;
            gpu->renderer->capset_info(gpu, i, &cid, &cver, &csize);
            if (cid == capset_id) {
                max_size = csize;
                id       = cid;
                break;
            }
        }
    }

    size_t   resp_size = VIRTIO_GPU_CTRL_HDR_SIZE + max_size;
    uint8_t* resp      = safe_calloc(resp_size, 1);
    gpu_build_resp_hdr(resp, VIRTIO_GPU_RESP_OK_CAPSET, hdr);
    if (id && max_size && gpu->renderer_ok && gpu->renderer->fill_caps) {
        gpu->renderer->fill_caps(gpu, capset_id, capset_ver, resp + VIRTIO_GPU_CTRL_HDR_SIZE, max_size);
    }
    size_t written = virtio_request_write(req, resp, resp_size);
    free(resp);
    virtio_request_complete(req, (uint32_t)written);
}

/*
 * 3D command handlers (forwarded to the renderer backend)
 */

static void gpu_cmd_ctx_create(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[8];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t nlen     = read_uint32_le_m(buf + 0);
    uint32_t ctx_init = read_uint32_le_m(buf + 4);
    char     name[64] = ZERO_INIT;
    uint32_t rd       = EVAL_MIN(nlen, (uint32_t)(sizeof(name) - 1));
    virtio_request_read(req, name, rd);

    if (gpu->renderer_ok && gpu->renderer->ctx_create) {
        gpu->renderer->ctx_create(gpu, hdr->ctx_id, ctx_init, name, rd);
    }
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_ctx_destroy(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    if (gpu->renderer_ok && gpu->renderer->ctx_destroy) {
        gpu->renderer->ctx_destroy(gpu, hdr->ctx_id);
    }
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_ctx_resource(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr, bool attach)
{
    uint8_t buf[8];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t res_id = read_uint32_le_m(buf + 0);
    if (gpu->renderer_ok) {
        if (attach && gpu->renderer->ctx_attach_resource) {
            gpu->renderer->ctx_attach_resource(gpu, hdr->ctx_id, res_id);
        } else if (!attach && gpu->renderer->ctx_detach_resource) {
            gpu->renderer->ctx_detach_resource(gpu, hdr->ctx_id, res_id);
        }
    }
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_resource_create_3d(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t buf[48];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    virtio_gpu_res_create_3d_t args = {
        .res_id     = read_uint32_le_m(buf + 0),
        .target     = read_uint32_le_m(buf + 4),
        .format     = read_uint32_le_m(buf + 8),
        .bind       = read_uint32_le_m(buf + 12),
        .width      = read_uint32_le_m(buf + 16),
        .height     = read_uint32_le_m(buf + 20),
        .depth      = read_uint32_le_m(buf + 24),
        .array_size = read_uint32_le_m(buf + 28),
        .last_level = read_uint32_le_m(buf + 32),
        .nr_samples = read_uint32_le_m(buf + 36),
        .flags      = read_uint32_le_m(buf + 40),
    };

    if (!gpu->renderer_ok || !gpu->renderer->resource_create_3d) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_UNSPEC, hdr);
        return;
    }
    if (gpu_res_find(gpu, args.res_id)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID, hdr);
        return;
    }

    gpu_resource_t* r = gpu_res_alloc(gpu, args.res_id);
    r->is_3d          = true;
    r->format         = args.format;
    r->width          = args.width;
    r->height         = args.height;
    gpu->renderer->resource_create_3d(gpu, &args);
    gpu_respond(gpu, req, VIRTIO_GPU_RESP_OK_NODATA, hdr);
}

static void gpu_cmd_transfer_3d(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr, bool to_host)
{
    uint8_t buf[48];
    if (virtio_request_read(req, buf, sizeof(buf)) != sizeof(buf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    virtio_gpu_transfer_3d_t t = {
        .x            = read_uint32_le_m(buf + 0),
        .y            = read_uint32_le_m(buf + 4),
        .z            = read_uint32_le_m(buf + 8),
        .w            = read_uint32_le_m(buf + 12),
        .h            = read_uint32_le_m(buf + 16),
        .d            = read_uint32_le_m(buf + 20),
        .offset       = read_uint64_le_m(buf + 24),
        .res_id       = read_uint32_le_m(buf + 32),
        .level        = read_uint32_le_m(buf + 36),
        .stride       = read_uint32_le_m(buf + 40),
        .layer_stride = read_uint32_le_m(buf + 44),
    };

    int rc = -1;
    if (gpu->renderer_ok) {
        if (to_host && gpu->renderer->transfer_to_host_3d) {
            rc = gpu->renderer->transfer_to_host_3d(gpu, hdr->ctx_id, &t);
        } else if (!to_host && gpu->renderer->transfer_from_host_3d) {
            rc = gpu->renderer->transfer_from_host_3d(gpu, hdr->ctx_id, &t);
        }
    }
    gpu_respond(gpu, req, rc == 0 ? VIRTIO_GPU_RESP_OK_NODATA : VIRTIO_GPU_RESP_ERR_UNSPEC, hdr);
}

static void gpu_cmd_submit_3d(virtio_gpu_t* gpu, virtio_request_t* req, const gpu_ctrl_hdr_t* hdr)
{
    uint8_t hbuf[8];
    if (virtio_request_read(req, hbuf, sizeof(hbuf)) != sizeof(hbuf)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, hdr);
        return;
    }
    uint32_t size = read_uint32_le_m(hbuf + 0); // size in bytes

    int rc = -1;
    if (size && size <= (32 << 20) && gpu->renderer_ok && gpu->renderer->submit_3d) {
        uint8_t* cmd = safe_calloc(size, 1);
        if (virtio_request_read(req, cmd, size) == size) {
            rc = gpu->renderer->submit_3d(gpu, hdr->ctx_id, cmd, size);
        }
        free(cmd);
    }
    gpu_respond(gpu, req, rc == 0 ? VIRTIO_GPU_RESP_OK_NODATA : VIRTIO_GPU_RESP_ERR_UNSPEC, hdr);
}

/*
 * Control queue dispatch
 */

static void gpu_dispatch(virtio_gpu_t* gpu, virtio_request_t* req)
{
    gpu_ctrl_hdr_t hdr = ZERO_INIT;
    if (!gpu_read_hdr(req, &hdr)) {
        gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_UNSPEC, &hdr);
        return;
    }

    switch (hdr.type) {
        case VIRTIO_GPU_CMD_GET_DISPLAY_INFO:
            gpu_cmd_get_display_info(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_RESOURCE_CREATE_2D:
            gpu_cmd_resource_create_2d(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_RESOURCE_UNREF:
            gpu_cmd_resource_unref(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_SET_SCANOUT:
            gpu_cmd_set_scanout(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_RESOURCE_FLUSH:
            gpu_cmd_resource_flush(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D:
            gpu_cmd_transfer_to_host_2d(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING:
            gpu_cmd_attach_backing(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING:
            gpu_cmd_detach_backing(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_GET_CAPSET_INFO:
            gpu_cmd_get_capset_info(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_GET_CAPSET:
            gpu_cmd_get_capset(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_CTX_CREATE:
            gpu_cmd_ctx_create(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_CTX_DESTROY:
            gpu_cmd_ctx_destroy(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE:
            gpu_cmd_ctx_resource(gpu, req, &hdr, true);
            break;
        case VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE:
            gpu_cmd_ctx_resource(gpu, req, &hdr, false);
            break;
        case VIRTIO_GPU_CMD_RESOURCE_CREATE_3D:
            gpu_cmd_resource_create_3d(gpu, req, &hdr);
            break;
        case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D:
            gpu_cmd_transfer_3d(gpu, req, &hdr, true);
            break;
        case VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D:
            gpu_cmd_transfer_3d(gpu, req, &hdr, false);
            break;
        case VIRTIO_GPU_CMD_SUBMIT_3D:
            gpu_cmd_submit_3d(gpu, req, &hdr);
            break;
        default:
            rvvm_info("virtio-gpu: unhandled command 0x%x", hdr.type);
            gpu_respond(gpu, req, VIRTIO_GPU_RESP_ERR_UNSPEC, &hdr);
            break;
    }
}

/*
 * Cursor queue: acknowledge cursor updates (no hardware cursor yet)
 */
static void gpu_process_cursor(virtio_gpu_t* gpu, virtio_request_t* req)
{
    // Cursor commands carry a virtio_gpu_update_cursor structure and expect
    // the buffer to simply be returned to the used ring.
    UNUSED(gpu);
    virtio_request_complete(req, 0);
}

/*
 * VirtIO transport callbacks
 */

static void gpu_notify(virtio_dev_t* vdev, uint16_t queue_id)
{
    virtio_gpu_t* gpu = virtio_dev_data(vdev);

    spin_lock(&gpu->lock);
    virtio_request_t* req = NULL;
    while ((req = virtio_queue_pop(vdev, queue_id)) != NULL) {
        if (queue_id == VIRTIO_GPU_CTRLQ) {
            gpu_dispatch(gpu, req);
        } else {
            gpu_process_cursor(gpu, req);
        }
    }
    spin_unlock(&gpu->lock);

    virtio_queue_interrupt(vdev, queue_id);
}

// Periodic refresh at host rate: redraw the scanout and poll input
static void gpu_poll(virtio_dev_t* vdev)
{
    virtio_gpu_t* gpu = virtio_dev_data(vdev);
    rvvm_fbdev_update(gpu->fbdev);
}

static void gpu_config_read(virtio_dev_t* vdev, void* data, size_t size, size_t off)
{
    virtio_gpu_t* gpu = virtio_dev_data(vdev);
    uint32_t      val = 0;
    switch (off & ~3u) {
        case 0: // events_read
            val = gpu->events_read;
            break;
        case 4: // events_clear
            val = 0;
            break;
        case 8: // num_scanouts
            val = gpu->num_scanouts;
            break;
        case 12: // num_capsets
            val = (gpu->renderer_ok && gpu->renderer->num_capsets) ? gpu->renderer->num_capsets(gpu) : 0;
            break;
    }
    switch (size) {
        case 1:
            write_uint8(data, val >> ((off & 3) * 8));
            break;
        case 2:
            write_uint16_le(data, val >> ((off & 3) * 8));
            break;
        default:
            write_uint32_le(data, val);
            break;
    }
}

static void gpu_config_write(virtio_dev_t* vdev, const void* data, size_t size, size_t off)
{
    virtio_gpu_t* gpu = virtio_dev_data(vdev);
    if ((off & ~3u) == 4) {
        // events_clear
        uint32_t val = (size >= 4) ? read_uint32_le(data) : read_uint16_le(data);
        gpu->events_read &= ~val;
    }
}

static void gpu_reset(virtio_dev_t* vdev)
{
    virtio_gpu_t* gpu = virtio_dev_data(vdev);
    spin_lock(&gpu->lock);
    for (uint32_t i = 0; i < gpu->res_cnt; ++i) {
        if (gpu->res[i].id) {
            gpu_res_free(gpu, &gpu->res[i]);
        }
    }
    gpu->res_cnt = 0;
    for (uint32_t i = 0; i < gpu->num_scanouts; ++i) {
        gpu->scanout[i].resource_id = 0;
    }
    gpu->events_read = 0;
    spin_unlock(&gpu->lock);
}

static void gpu_cleanup(virtio_dev_t* vdev)
{
    virtio_gpu_t* gpu = virtio_dev_data(vdev);
    if (gpu->renderer_ok && gpu->renderer->cleanup) {
        gpu->renderer->cleanup(gpu);
    }
    for (uint32_t i = 0; i < gpu->res_cnt; ++i) {
        if (gpu->res[i].id) {
            gpu_res_free(gpu, &gpu->res[i]);
        }
    }
    free(gpu->res);
    rvvm_fbdev_dec_ref(gpu->fbdev);
    free(gpu);
}

static const virtio_dev_cb_t gpu_virtio_cb = {
    .config_read  = gpu_config_read,
    .config_write = gpu_config_write,
    .notify       = gpu_notify,
    .poll         = gpu_poll,
    .reset        = gpu_reset,
    .cleanup      = gpu_cleanup,
};

/*
 * Default renderer backend: none (2D only).
 * Overridden by the virgl backend when built with USE_VIRGL.
 */
#ifndef USE_VIRGL
const virtio_gpu_renderer_t* virtio_gpu_get_renderer(void)
{
    return NULL;
}
#endif

/*
 * Public constructor
 */

RVVM_PUBLIC rvvm_pci_func_t* rvvm_virtio_gpu_init(rvvm_machine_t* machine, rvvm_fbdev_t* fbdev, rvvm_pci_addr_t addr)
{
    if (!fbdev) {
        return NULL;
    }

    virtio_gpu_t* gpu = safe_new_obj(virtio_gpu_t);
    spin_init(&gpu->lock);
    rvvm_fbdev_inc_ref(fbdev);
    gpu->fbdev        = fbdev;
    gpu->num_scanouts = 1;

    // Query the display geometry from the framebuffer scanout
    rvvm_fb_t fb = ZERO_INIT;
    rvvm_fbdev_get_scanout(fbdev, &fb);
    gpu->disp_width  = rvvm_fb_width(&fb) ? rvvm_fb_width(&fb) : 1280;
    gpu->disp_height = rvvm_fb_height(&fb) ? rvvm_fb_height(&fb) : 720;

    // Probe the optional 3D renderer backend (virgl / venus)
    gpu->renderer = virtio_gpu_get_renderer();
    if (gpu->renderer && gpu->renderer->init) {
        gpu->renderer_ok = gpu->renderer->init(gpu);
    }

    uint64_t features = 0;
    if (gpu->renderer_ok) {
        features |= (1ULL << VIRTIO_GPU_F_VIRGL);
        features |= (1ULL << VIRTIO_GPU_F_CONTEXT_INIT);
        rvvm_info("virtio-gpu: 3D acceleration enabled (%s)", gpu->renderer->name);
    }

    virtio_dev_desc_t desc = {
        .name        = "virtio-gpu",
        .device_id   = VIRTIO_ID_GPU,
        .class_code  = 0x0380, // Display controller
        .num_queues  = 2,      // controlq + cursorq
        .config_size = 16,     // virtio_gpu_config
        .features    = features,
        .cb          = &gpu_virtio_cb,
        .data        = gpu,
    };

    virtio_dev_t* vdev = virtio_pci_init(machine, &desc, addr);
    if (!vdev) {
        // virtio_pci_init cleanup path already invoked gpu_cleanup
        return NULL;
    }
    gpu->vdev = vdev;
    return virtio_pci_func(vdev);
}

POP_OPTIMIZATION_SIZE
