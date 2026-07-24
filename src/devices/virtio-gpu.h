/*
virtio-gpu.h - VirtIO GPU device definitions and renderer backend interface
Copyright (C) 2026  RVVM Contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_VIRTIO_GPU_H
#define RVVM_VIRTIO_GPU_H

#include <rvvm/rvvm.h>

/*
 * VirtIO GPU feature bits (VirtIO spec 5.7.3)
 */
#define VIRTIO_GPU_F_VIRGL         0
#define VIRTIO_GPU_F_EDID          1
#define VIRTIO_GPU_F_RESOURCE_UUID 2
#define VIRTIO_GPU_F_RESOURCE_BLOB 3
#define VIRTIO_GPU_F_CONTEXT_INIT  4

/*
 * VirtIO GPU control commands (VirtIO spec 5.7.6.7)
 */
#define VIRTIO_GPU_CMD_GET_DISPLAY_INFO       0x0100
#define VIRTIO_GPU_CMD_RESOURCE_CREATE_2D     0x0101
#define VIRTIO_GPU_CMD_RESOURCE_UNREF         0x0102
#define VIRTIO_GPU_CMD_SET_SCANOUT            0x0103
#define VIRTIO_GPU_CMD_RESOURCE_FLUSH         0x0104
#define VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D    0x0105
#define VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING 0x0106
#define VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING 0x0107
#define VIRTIO_GPU_CMD_GET_CAPSET_INFO        0x0108
#define VIRTIO_GPU_CMD_GET_CAPSET             0x0109
#define VIRTIO_GPU_CMD_GET_EDID               0x010A
#define VIRTIO_GPU_CMD_RESOURCE_ASSIGN_UUID   0x010B
#define VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB   0x010C
#define VIRTIO_GPU_CMD_SET_SCANOUT_BLOB       0x010D

/* 3D commands (require VIRGL) */
#define VIRTIO_GPU_CMD_CTX_CREATE             0x0200
#define VIRTIO_GPU_CMD_CTX_DESTROY            0x0201
#define VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE    0x0202
#define VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE    0x0203
#define VIRTIO_GPU_CMD_RESOURCE_CREATE_3D     0x0204
#define VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D    0x0205
#define VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D  0x0206
#define VIRTIO_GPU_CMD_SUBMIT_3D              0x0207
#define VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB      0x0208
#define VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB    0x0209

/* Cursor commands */
#define VIRTIO_GPU_CMD_UPDATE_CURSOR          0x0300
#define VIRTIO_GPU_CMD_MOVE_CURSOR            0x0301

/*
 * VirtIO GPU responses
 */
#define VIRTIO_GPU_RESP_OK_NODATA             0x1100
#define VIRTIO_GPU_RESP_OK_DISPLAY_INFO       0x1101
#define VIRTIO_GPU_RESP_OK_CAPSET_INFO        0x1102
#define VIRTIO_GPU_RESP_OK_CAPSET             0x1103
#define VIRTIO_GPU_RESP_OK_EDID               0x1104
#define VIRTIO_GPU_RESP_OK_RESOURCE_UUID      0x1105
#define VIRTIO_GPU_RESP_OK_MAP_INFO           0x1106

#define VIRTIO_GPU_RESP_ERR_UNSPEC            0x1200
#define VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY     0x1201
#define VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID 0x1202
#define VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID 0x1203
#define VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID 0x1204
#define VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER 0x1205

/*
 * Control header flags
 */
#define VIRTIO_GPU_FLAG_FENCE                 (1 << 0)
#define VIRTIO_GPU_FLAG_INFO_RING_IDX         (1 << 1)

/*
 * VirtIO GPU 2D formats
 */
#define VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM      1
#define VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM      2
#define VIRTIO_GPU_FORMAT_A8R8G8B8_UNORM      3
#define VIRTIO_GPU_FORMAT_X8R8G8B8_UNORM      4
#define VIRTIO_GPU_FORMAT_R8G8B8A8_UNORM      67
#define VIRTIO_GPU_FORMAT_X8B8G8R8_UNORM      68
#define VIRTIO_GPU_FORMAT_A8B8G8R8_UNORM      121
#define VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM      134

/*
 * Capsets (VirtIO GPU 3D)
 */
#define VIRTIO_GPU_CAPSET_VIRGL               1
#define VIRTIO_GPU_CAPSET_VIRGL2              2
#define VIRTIO_GPU_CAPSET_VENUS               4

#define VIRTIO_GPU_MAX_SCANOUTS               16

/*
 * Opaque VirtIO GPU device handle
 */
typedef struct virtio_gpu virtio_gpu_t;

/*
 * Host memory scatter/gather entry (mapped guest backing)
 */
typedef struct {
    void*  base;
    size_t len;
} virtio_gpu_iovec_t;

/*
 * 3D resource creation arguments (mirrors virgl_renderer_resource_create_args)
 */
typedef struct {
    uint32_t res_id;
    uint32_t target;
    uint32_t format;
    uint32_t bind;
    uint32_t width;
    uint32_t height;
    uint32_t depth;
    uint32_t array_size;
    uint32_t last_level;
    uint32_t nr_samples;
    uint32_t flags;
} virtio_gpu_res_create_3d_t;

/*
 * 3D transfer arguments (mirrors TRANSFER_*_HOST_3D)
 */
typedef struct {
    uint32_t res_id;
    uint32_t level;
    uint32_t stride;
    uint32_t layer_stride;
    uint64_t offset;
    uint32_t x, y, z;
    uint32_t w, h, d;
} virtio_gpu_transfer_3d_t;

/*
 * Renderer backend interface (3D acceleration: virgl / venus).
 *
 * A backend is optional. When no backend is registered, the device operates
 * in 2D-only mode and does not advertise VIRTIO_GPU_F_VIRGL. The backend maps
 * VirtIO GPU 3D commands onto a host renderer (libvirglrenderer), which in turn
 * dispatches OpenGL (virgl) or Vulkan (venus) to the host GPU.
 *
 * All callbacks are invoked from the control-queue processing context.
 */
typedef struct {
    const char* name;

    /* Initialize the renderer for this GPU instance. Returns success. */
    bool (*init)(virtio_gpu_t* gpu);

    /* Tear down the renderer. */
    void (*cleanup)(virtio_gpu_t* gpu);

    /* Number of capability sets exposed to the guest. */
    uint32_t (*num_capsets)(virtio_gpu_t* gpu);

    /* Describe capset at `index`. */
    void (*capset_info)(virtio_gpu_t* gpu, uint32_t index, //
                        uint32_t* id, uint32_t* max_version, uint32_t* max_size);

    /* Fill `size` bytes of capset data for (id, version) into `caps`. */
    void (*fill_caps)(virtio_gpu_t* gpu, uint32_t id, uint32_t version, void* caps, size_t size);

    /* Rendering context lifecycle. */
    int (*ctx_create)(virtio_gpu_t* gpu, uint32_t ctx_id, uint32_t ctx_init, const char* name, uint32_t nlen);
    void (*ctx_destroy)(virtio_gpu_t* gpu, uint32_t ctx_id);
    void (*ctx_attach_resource)(virtio_gpu_t* gpu, uint32_t ctx_id, uint32_t res_id);
    void (*ctx_detach_resource)(virtio_gpu_t* gpu, uint32_t ctx_id, uint32_t res_id);

    /* 3D resource lifecycle. */
    int (*resource_create_3d)(virtio_gpu_t* gpu, const virtio_gpu_res_create_3d_t* args);
    void (*resource_unref)(virtio_gpu_t* gpu, uint32_t res_id);
    void (*resource_attach_backing)(virtio_gpu_t* gpu, uint32_t res_id, //
                                    const virtio_gpu_iovec_t* iov, uint32_t niov);
    void (*resource_detach_backing)(virtio_gpu_t* gpu, uint32_t res_id);

    /* 3D transfers. */
    int (*transfer_to_host_3d)(virtio_gpu_t* gpu, uint32_t ctx_id, const virtio_gpu_transfer_3d_t* t);
    int (*transfer_from_host_3d)(virtio_gpu_t* gpu, uint32_t ctx_id, const virtio_gpu_transfer_3d_t* t);

    /* Submit a 3D command stream (virgl / venus encoded). */
    int (*submit_3d)(virtio_gpu_t* gpu, uint32_t ctx_id, void* cmd, size_t size);

    /* Create/settle a fence. Returns success; fences settle synchronously. */
    int (*create_fence)(virtio_gpu_t* gpu, uint32_t ctx_id, uint32_t ring_idx, uint64_t fence_id);

    /* Force fence progress. */
    void (*poll)(virtio_gpu_t* gpu);
} virtio_gpu_renderer_t;

/*
 * Obtain the active renderer backend, or NULL when 3D is unavailable.
 * Implemented by the virgl backend (USE_VIRGL) or the stub.
 */
const virtio_gpu_renderer_t* virtio_gpu_get_renderer(void);

#endif
