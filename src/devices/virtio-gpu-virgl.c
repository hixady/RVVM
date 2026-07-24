/*
virtio-gpu-virgl.c - VirtIO GPU 3D renderer backend (VirGL / Venus)
Copyright (C) 2026  RVVM Contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/*
 * Renderer backend that bridges the VirtIO GPU 3D command set onto
 * libvirglrenderer. virglrenderer translates the guest's virgl (OpenGL) or
 * venus (Vulkan) command stream into host GPU work through EGL.
 *
 * Enabled with USE_VIRGL (links against pkg-config `virglrenderer`).
 *
 * NOTE: libvirglrenderer is a process-global singleton, so at most one
 * VirtIO GPU instance may drive it. The backend initializes a surfaceless
 * EGL renderer; a working host GPU/EGL stack is required at runtime.
 */

#ifdef USE_VIRGL

#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <rvvm/rvvm.h>

#include <util/utils.h>

#include "virtio-gpu.h"

#include <fcntl.h>
#include <string.h>
#include <sys/uio.h>
#include <unistd.h>

#include <virgl/virglrenderer.h>

// Complete the box type forward-declared by virglrenderer.h.
// Layout matches virtio_gpu_box / virgl_box: { x, y, z, w, h, d }.
struct virgl_box {
    uint32_t x, y, z;
    uint32_t w, h, d;
};

/*
 * Tracked resource->iovec attachment (virglrenderer stores the pointer,
 * so the host-side iovec array must outlive the attachment).
 */
typedef struct virgl_attach {
    uint32_t             res_id;
    struct iovec*        iov;
    int                  cnt;
    struct virgl_attach* next;
} virgl_attach_t;

typedef struct {
    virtio_gpu_t*   gpu;
    bool            ready;
    virgl_attach_t* attachments;

    // Advertised capsets
    struct {
        uint32_t id;
        uint32_t max_ver;
        uint32_t max_size;
    } capset[4];
    uint32_t capset_cnt;
} virgl_state_t;

static virgl_state_t g_virgl = ZERO_INIT;

/*
 * virglrenderer callbacks
 */

static void virgl_cb_write_fence(void* cookie, uint32_t fence)
{
    // Fences settle synchronously (see create_fence + poll), nothing to do.
    UNUSED(cookie && fence);
}

static int virgl_cb_get_drm_fd(void* cookie)
{
    UNUSED(cookie);
    // Let virglrenderer own the fd; try common render nodes.
    static const char* nodes[] = {"/dev/dri/renderD128", "/dev/dri/renderD129"};
    for (size_t i = 0; i < STATIC_ARRAY_SIZE(nodes); ++i) {
        int fd = open(nodes[i], O_RDWR | O_CLOEXEC);
        if (fd >= 0) {
            return fd;
        }
    }
    return -1;
}

static struct virgl_renderer_callbacks g_virgl_cbs = {
    .version     = 2,
    .write_fence = virgl_cb_write_fence,
    .get_drm_fd  = virgl_cb_get_drm_fd,
};

/*
 * Attachment tracking
 */

static void virgl_attach_store(uint32_t res_id, struct iovec* iov, int cnt)
{
    // Drop any previous attachment for this resource
    virgl_attach_t** pp = &g_virgl.attachments;
    while (*pp) {
        if ((*pp)->res_id == res_id) {
            virgl_attach_t* dead = *pp;
            *pp                  = dead->next;
            free(dead->iov);
            free(dead);
            continue;
        }
        pp = &(*pp)->next;
    }
    virgl_attach_t* node = safe_new_obj(virgl_attach_t);
    node->res_id         = res_id;
    node->iov            = iov;
    node->cnt            = cnt;
    node->next           = g_virgl.attachments;
    g_virgl.attachments  = node;
}

static void virgl_attach_drop(uint32_t res_id)
{
    virgl_attach_t** pp = &g_virgl.attachments;
    while (*pp) {
        if ((*pp)->res_id == res_id) {
            virgl_attach_t* dead = *pp;
            *pp                  = dead->next;
            free(dead->iov);
            free(dead);
            return;
        }
        pp = &(*pp)->next;
    }
}

/*
 * Backend operations
 */

static bool virgl_init(virtio_gpu_t* gpu)
{
    if (g_virgl.ready) {
        rvvm_warn("virtio-gpu: virglrenderer already initialized (only one instance supported)");
        return false;
    }

    int flags = VIRGL_RENDERER_USE_EGL | VIRGL_RENDERER_USE_SURFACELESS;
    int rc    = virgl_renderer_init(gpu, flags, &g_virgl_cbs);
    if (rc != 0) {
        rvvm_warn("virtio-gpu: virgl_renderer_init failed (%d), falling back to 2D", rc);
        return false;
    }

    g_virgl.gpu   = gpu;
    g_virgl.ready = true;

    // Probe available capsets (virgl2 + venus)
    static const uint32_t probe[] = {VIRTIO_GPU_CAPSET_VIRGL2, VIRTIO_GPU_CAPSET_VENUS};
    for (size_t i = 0; i < STATIC_ARRAY_SIZE(probe); ++i) {
        uint32_t max_ver = 0, max_size = 0;
        virgl_renderer_get_cap_set(probe[i], &max_ver, &max_size);
        if (max_ver && max_size && g_virgl.capset_cnt < STATIC_ARRAY_SIZE(g_virgl.capset)) {
            g_virgl.capset[g_virgl.capset_cnt].id       = probe[i];
            g_virgl.capset[g_virgl.capset_cnt].max_ver  = max_ver;
            g_virgl.capset[g_virgl.capset_cnt].max_size = max_size;
            g_virgl.capset_cnt++;
        }
    }
    return true;
}

static void virgl_cleanup(virtio_gpu_t* gpu)
{
    if (!g_virgl.ready) {
        return;
    }
    while (g_virgl.attachments) {
        virgl_attach_t* dead = g_virgl.attachments;
        g_virgl.attachments  = dead->next;
        free(dead->iov);
        free(dead);
    }
    virgl_renderer_cleanup(gpu);
    g_virgl.ready      = false;
    g_virgl.capset_cnt = 0;
}

static uint32_t virgl_num_capsets(virtio_gpu_t* gpu)
{
    UNUSED(gpu);
    return g_virgl.capset_cnt;
}

static void virgl_capset_info(virtio_gpu_t* gpu, uint32_t index, //
                              uint32_t* id, uint32_t* max_version, uint32_t* max_size)
{
    UNUSED(gpu);
    if (index < g_virgl.capset_cnt) {
        *id          = g_virgl.capset[index].id;
        *max_version = g_virgl.capset[index].max_ver;
        *max_size    = g_virgl.capset[index].max_size;
    } else {
        *id          = 0;
        *max_version = 0;
        *max_size    = 0;
    }
}

static void virgl_fill_caps(virtio_gpu_t* gpu, uint32_t id, uint32_t version, void* caps, size_t size)
{
    UNUSED(gpu && size);
    virgl_renderer_fill_caps(id, version, caps);
}

static int virgl_ctx_create(virtio_gpu_t* gpu, uint32_t ctx_id, uint32_t ctx_init, const char* name, uint32_t nlen)
{
    UNUSED(gpu);
    if (ctx_init) {
        return virgl_renderer_context_create_with_flags(ctx_id, ctx_init, nlen, name);
    }
    return virgl_renderer_context_create(ctx_id, nlen, name);
}

static void virgl_ctx_destroy(virtio_gpu_t* gpu, uint32_t ctx_id)
{
    UNUSED(gpu);
    virgl_renderer_context_destroy(ctx_id);
}

static void virgl_ctx_attach(virtio_gpu_t* gpu, uint32_t ctx_id, uint32_t res_id)
{
    UNUSED(gpu);
    virgl_renderer_ctx_attach_resource((int)ctx_id, (int)res_id);
}

static void virgl_ctx_detach(virtio_gpu_t* gpu, uint32_t ctx_id, uint32_t res_id)
{
    UNUSED(gpu);
    virgl_renderer_ctx_detach_resource((int)ctx_id, (int)res_id);
}

static int virgl_resource_create_3d(virtio_gpu_t* gpu, const virtio_gpu_res_create_3d_t* a)
{
    UNUSED(gpu);
    struct virgl_renderer_resource_create_args args = {
        .handle     = a->res_id,
        .target     = a->target,
        .format     = a->format,
        .bind       = a->bind,
        .width      = a->width,
        .height     = a->height,
        .depth      = a->depth,
        .array_size = a->array_size,
        .last_level = a->last_level,
        .nr_samples = a->nr_samples,
        .flags      = a->flags,
    };
    return virgl_renderer_resource_create(&args, NULL, 0);
}

static void virgl_resource_unref(virtio_gpu_t* gpu, uint32_t res_id)
{
    UNUSED(gpu);
    virgl_attach_drop(res_id);
    virgl_renderer_resource_unref(res_id);
}

static void virgl_resource_attach_backing(virtio_gpu_t* gpu, uint32_t res_id, //
                                          const virtio_gpu_iovec_t* iov, uint32_t niov)
{
    UNUSED(gpu);
    struct iovec* viov = safe_new_arr(struct iovec, niov ? niov : 1);
    for (uint32_t i = 0; i < niov; ++i) {
        viov[i].iov_base = iov[i].base;
        viov[i].iov_len  = iov[i].len;
    }
    virgl_renderer_resource_attach_iov((int)res_id, viov, (int)niov);
    virgl_attach_store(res_id, viov, (int)niov);
}

static void virgl_resource_detach_backing(virtio_gpu_t* gpu, uint32_t res_id)
{
    UNUSED(gpu);
    struct iovec* iov = NULL;
    int           cnt = 0;
    virgl_renderer_resource_detach_iov((int)res_id, &iov, &cnt);
    virgl_attach_drop(res_id);
}

static int virgl_transfer_to_host_3d(virtio_gpu_t* gpu, uint32_t ctx_id, const virtio_gpu_transfer_3d_t* t)
{
    UNUSED(gpu);
    struct virgl_box box = {.x = t->x, .y = t->y, .z = t->z, .w = t->w, .h = t->h, .d = t->d};
    return virgl_renderer_transfer_write_iov(t->res_id, ctx_id, (int)t->level, t->stride, //
                                             t->layer_stride, &box, t->offset, NULL, 0);
}

static int virgl_transfer_from_host_3d(virtio_gpu_t* gpu, uint32_t ctx_id, const virtio_gpu_transfer_3d_t* t)
{
    UNUSED(gpu);
    struct virgl_box box = {.x = t->x, .y = t->y, .z = t->z, .w = t->w, .h = t->h, .d = t->d};
    return virgl_renderer_transfer_read_iov(t->res_id, ctx_id, t->level, t->stride, //
                                            t->layer_stride, &box, t->offset, NULL, 0);
}

static int virgl_submit_3d(virtio_gpu_t* gpu, uint32_t ctx_id, void* cmd, size_t size)
{
    UNUSED(gpu);
    return virgl_renderer_submit_cmd(cmd, (int)ctx_id, (int)(size / 4));
}

static int virgl_create_fence(virtio_gpu_t* gpu, uint32_t ctx_id, uint32_t ring_idx, uint64_t fence_id)
{
    UNUSED(gpu && ring_idx);
    int rc = virgl_renderer_create_fence((int)fence_id, ctx_id);
    // Force fence completion (synchronous fencing model)
    virgl_renderer_poll();
    return rc;
}

static void virgl_poll(virtio_gpu_t* gpu)
{
    UNUSED(gpu);
    virgl_renderer_poll();
}

static const virtio_gpu_renderer_t g_virgl_renderer = {
    .name                    = "virglrenderer",
    .init                    = virgl_init,
    .cleanup                 = virgl_cleanup,
    .num_capsets             = virgl_num_capsets,
    .capset_info             = virgl_capset_info,
    .fill_caps               = virgl_fill_caps,
    .ctx_create              = virgl_ctx_create,
    .ctx_destroy             = virgl_ctx_destroy,
    .ctx_attach_resource     = virgl_ctx_attach,
    .ctx_detach_resource     = virgl_ctx_detach,
    .resource_create_3d      = virgl_resource_create_3d,
    .resource_unref          = virgl_resource_unref,
    .resource_attach_backing = virgl_resource_attach_backing,
    .resource_detach_backing = virgl_resource_detach_backing,
    .transfer_to_host_3d     = virgl_transfer_to_host_3d,
    .transfer_from_host_3d   = virgl_transfer_from_host_3d,
    .submit_3d               = virgl_submit_3d,
    .create_fence            = virgl_create_fence,
    .poll                    = virgl_poll,
};

const virtio_gpu_renderer_t* virtio_gpu_get_renderer(void)
{
    return &g_virgl_renderer;
}

#endif /* USE_VIRGL */
