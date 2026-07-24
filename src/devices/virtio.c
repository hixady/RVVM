/*
virtio.c - VirtIO PCI transport (Modern VirtIO 1.x)
Copyright (C) 2026  RVVM Contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/*
 * Generic modern (1.x) VirtIO transport over PCI, with split virtqueues.
 *
 * The transport publishes the four required VirtIO PCI capabilities
 * (common / notify / ISR / device config) through the device capability
 * window in PCI config space, and lays out their register windows in BAR0.
 *
 * Device implementations (virtio-gpu, etc) plug in via virtio_dev_cb_t and
 * consume descriptor chains through the virtio_queue_pop()/complete() API.
 */

#include "virtio.h"

#include <rvvm/rvvm_region.h>

#include <util/atomics.h>
#include <util/bit_ops.h>
#include <util/mem_ops.h>
#include <util/spinlock.h>
#include <util/utils.h>

#include "compiler.h"

PUSH_OPTIMIZATION_SIZE

/*
 * VirtIO PCI capability types (VirtIO spec 4.1.4)
 */
#define VIRTIO_PCI_CAP_COMMON_CFG 1
#define VIRTIO_PCI_CAP_NOTIFY_CFG 2
#define VIRTIO_PCI_CAP_ISR_CFG    3
#define VIRTIO_PCI_CAP_DEVICE_CFG 4
#define VIRTIO_PCI_CAP_PCI_CFG    5

#define PCI_CAP_ID_VNDR           0x09

/*
 * BAR0 sub-region layout (each region is page sized for alignment)
 */
#define VIRTIO_BAR_COMMON_OFF     0x0000
#define VIRTIO_BAR_ISR_OFF        0x1000
#define VIRTIO_BAR_DEVICE_OFF     0x2000
#define VIRTIO_BAR_NOTIFY_OFF     0x3000
#define VIRTIO_BAR_REGION_LEN     0x1000
#define VIRTIO_BAR_SIZE           0x4000
#define VIRTIO_NOTIFY_MULTIPLIER  4

/*
 * Common configuration structure register offsets (VirtIO spec 4.1.4.3)
 */
#define VIRTIO_COMMON_DFSELECT    0x00 // device_feature_select (RW, 32)
#define VIRTIO_COMMON_DF          0x04 // device_feature        (RO, 32)
#define VIRTIO_COMMON_GFSELECT    0x08 // driver_feature_select (RW, 32)
#define VIRTIO_COMMON_GF          0x0C // driver_feature        (RW, 32)
#define VIRTIO_COMMON_MSIX        0x10 // msix_config           (RW, 16)
#define VIRTIO_COMMON_NUMQ        0x12 // num_queues            (RO, 16)
#define VIRTIO_COMMON_STATUS      0x14 // device_status         (RW, 8)
#define VIRTIO_COMMON_CFGGEN      0x15 // config_generation     (RO, 8)
#define VIRTIO_COMMON_Q_SELECT    0x16 // queue_select          (RW, 16)
#define VIRTIO_COMMON_Q_SIZE      0x18 // queue_size            (RW, 16)
#define VIRTIO_COMMON_Q_MSIX      0x1A // queue_msix_vector     (RW, 16)
#define VIRTIO_COMMON_Q_ENABLE    0x1C // queue_enable          (RW, 16)
#define VIRTIO_COMMON_Q_NOFF      0x1E // queue_notify_off      (RO, 16)
#define VIRTIO_COMMON_Q_DESCLO    0x20 // queue_desc lo         (RW, 32)
#define VIRTIO_COMMON_Q_DESCHI    0x24 // queue_desc hi         (RW, 32)
#define VIRTIO_COMMON_Q_AVAILLO   0x28 // queue_driver lo       (RW, 32)
#define VIRTIO_COMMON_Q_AVAILHI   0x2C // queue_driver hi       (RW, 32)
#define VIRTIO_COMMON_Q_USEDLO    0x30 // queue_device lo       (RW, 32)
#define VIRTIO_COMMON_Q_USEDHI    0x34 // queue_device hi       (RW, 32)

/*
 * Device status bits (VirtIO spec 2.1)
 */
#define VIRTIO_STATUS_ACKNOWLEDGE 0x01
#define VIRTIO_STATUS_DRIVER      0x02
#define VIRTIO_STATUS_DRIVER_OK   0x04
#define VIRTIO_STATUS_FEATURES_OK 0x08
#define VIRTIO_STATUS_NEEDS_RESET 0x40
#define VIRTIO_STATUS_FAILED      0x80

/*
 * ISR status bits
 */
#define VIRTIO_ISR_QUEUE          0x01
#define VIRTIO_ISR_CONFIG         0x02

/*
 * Split virtqueue descriptor flags
 */
#define VRING_DESC_F_NEXT         0x01
#define VRING_DESC_F_WRITE        0x02
#define VRING_DESC_F_INDIRECT     0x04

/*
 * Available/used ring flags
 */
#define VRING_AVAIL_F_NO_INTERRUPT 0x01

#define VIRTIO_MSI_NO_VECTOR      0xFFFF

#define VIRTIO_QUEUE_SIZE_MAX     256
#define VIRTIO_REQ_MAX_IOV        4096
#define VIRTIO_REQ_INLINE_IOV     8

/*
 * Per-queue state
 */
typedef struct {
    uint16_t size;        // Negotiated queue size (num descriptors)
    uint16_t msix_vector; // MSI-X vector for this queue
    uint16_t enable;      // Queue enabled by driver
    uint16_t last_avail;  // Next available ring index to consume
    uint16_t used_idx;    // Shadow of used ring index
    uint64_t desc_addr;   // Descriptor table guest address
    uint64_t avail_addr;  // Available ring guest address
    uint64_t used_addr;   // Used ring guest address
} virtio_queue_t;

struct virtio_dev {
    rvvm_pci_func_t*       func;
    const virtio_dev_cb_t* cb;
    void*                  data;

    uint16_t num_queues;
    uint32_t config_size;
    uint64_t host_features; // Offered feature bits

    // Driver-visible state
    uint32_t device_feature_select;
    uint32_t driver_feature_select;
    uint64_t driver_features;
    uint16_t msix_config;
    uint16_t queue_select;
    uint8_t  status;
    uint8_t  config_generation;
    uint32_t isr_status;
    bool     msix_active;

    virtio_queue_t* queues;

    // PCI config space capability blob (device capability window)
    uint8_t cap_blob[68];

    spinlock_t lock;
};

/*
 * A single mapped descriptor buffer
 */
typedef struct {
    void*    buf;
    uint32_t len;
    bool     write; // Device-writable (VRING_DESC_F_WRITE)
} virtio_iov_t;

struct virtio_request {
    virtio_dev_t* vdev;
    uint16_t      queue_id;
    uint16_t      head;

    virtio_iov_t* iov;
    virtio_iov_t  inline_iov[VIRTIO_REQ_INLINE_IOV];
    uint32_t      iov_cnt;
    uint32_t      iov_cap;

    // Sequential read/write cursors
    uint32_t rd_idx, rd_off;
    uint32_t wr_idx, wr_off;
};

/*
 * DMA helpers
 */

void* virtio_dma_get(virtio_dev_t* vdev, rvvm_addr_t addr, size_t size)
{
    return rvvm_pci_get_dma(vdev->func, addr, size);
}

void virtio_dma_put(virtio_dev_t* vdev, void* ptr)
{
    rvvm_pci_end_dma(vdev->func, ptr);
}

/*
 * Capability blob construction
 */

static size_t virtio_build_cap(uint8_t* blob, size_t pos, uint8_t next, uint8_t type, //
                               uint32_t bar_off, uint32_t bar_len, bool notify)
{
    uint8_t cap_len = notify ? 20 : 16;
    blob[pos + 0]   = PCI_CAP_ID_VNDR;
    blob[pos + 1]   = next;
    blob[pos + 2]   = cap_len;
    blob[pos + 3]   = type;
    blob[pos + 4]   = 0; // bar index (BAR0)
    blob[pos + 5]   = 0; // id
    blob[pos + 6]   = 0; // padding
    blob[pos + 7]   = 0; // padding
    write_uint32_le_m(blob + pos + 8, bar_off);
    write_uint32_le_m(blob + pos + 12, bar_len);
    if (notify) {
        write_uint32_le_m(blob + pos + 16, VIRTIO_NOTIFY_MULTIPLIER);
    }
    return pos + cap_len;
}

static void virtio_build_caps(virtio_dev_t* vdev)
{
    // Capability window starts at PCI config 0xBC:
    //   common @ 0xBC (16) -> next 0xCC
    //   notify @ 0xCC (20) -> next 0xE0
    //   isr    @ 0xE0 (16) -> next 0xF0
    //   device @ 0xF0 (16) -> next 0x00 (end)
    size_t pos = 0;
    pos        = virtio_build_cap(vdev->cap_blob, pos, 0xCC, VIRTIO_PCI_CAP_COMMON_CFG, //
                                  VIRTIO_BAR_COMMON_OFF, VIRTIO_BAR_REGION_LEN, false);
    pos        = virtio_build_cap(vdev->cap_blob, pos, 0xE0, VIRTIO_PCI_CAP_NOTIFY_CFG, //
                                  VIRTIO_BAR_NOTIFY_OFF, VIRTIO_BAR_REGION_LEN, true);
    pos        = virtio_build_cap(vdev->cap_blob, pos, 0xF0, VIRTIO_PCI_CAP_ISR_CFG, //
                                  VIRTIO_BAR_ISR_OFF, VIRTIO_BAR_REGION_LEN, false);
    pos        = virtio_build_cap(vdev->cap_blob, pos, 0x00, VIRTIO_PCI_CAP_DEVICE_CFG, //
                                  VIRTIO_BAR_DEVICE_OFF, VIRTIO_BAR_REGION_LEN, false);
    UNUSED(pos);
}

/*
 * Interrupt handling
 */

// Raise a queue interrupt (must not hold vdev->lock)
static void virtio_raise_queue_irq(virtio_dev_t* vdev, uint16_t vector)
{
    if (vector != VIRTIO_MSI_NO_VECTOR) {
        // MSI-X delivery for this queue
        rvvm_pci_send_irq(vdev->func, vector);
    } else if (!vdev->msix_active) {
        // INTx / ISR delivery
        atomic_or_uint32(&vdev->isr_status, VIRTIO_ISR_QUEUE);
        rvvm_pci_set_irq(vdev->func, 0, true);
    }
}

void virtio_notify_config(virtio_dev_t* vdev)
{
    spin_lock(&vdev->lock);
    vdev->config_generation++;
    uint16_t vector = vdev->msix_config;
    bool     msix   = vdev->msix_active;
    spin_unlock(&vdev->lock);

    if (vector != VIRTIO_MSI_NO_VECTOR) {
        rvvm_pci_send_irq(vdev->func, vector);
    } else if (!msix) {
        atomic_or_uint32(&vdev->isr_status, VIRTIO_ISR_CONFIG);
        rvvm_pci_set_irq(vdev->func, 0, true);
    }
}

void virtio_queue_interrupt(virtio_dev_t* vdev, uint16_t queue_id)
{
    if (queue_id >= vdev->num_queues) {
        return;
    }
    spin_lock(&vdev->lock);
    virtio_queue_t* queue  = &vdev->queues[queue_id];
    uint16_t        vector = queue->msix_vector;
    uint64_t        avail  = queue->avail_addr;
    bool            enable = queue->enable;
    spin_unlock(&vdev->lock);

    if (!enable) {
        return;
    }

    // Honor the driver's interrupt suppression request
    void* avail_ptr = virtio_dma_get(vdev, avail, 4);
    if (avail_ptr) {
        uint16_t flags = read_uint16_le_m(avail_ptr);
        virtio_dma_put(vdev, avail_ptr);
        if (flags & VRING_AVAIL_F_NO_INTERRUPT) {
            return;
        }
    }

    virtio_raise_queue_irq(vdev, vector);
}

/*
 * Device reset
 */

static void virtio_do_reset(virtio_dev_t* vdev)
{
    // Called with vdev->lock held
    vdev->device_feature_select = 0;
    vdev->driver_feature_select = 0;
    vdev->driver_features       = 0;
    vdev->msix_config           = VIRTIO_MSI_NO_VECTOR;
    vdev->queue_select          = 0;
    vdev->status                = 0;
    vdev->isr_status            = 0;
    vdev->msix_active           = false;
    for (uint16_t i = 0; i < vdev->num_queues; ++i) {
        virtio_queue_t* queue = &vdev->queues[i];
        queue->size           = VIRTIO_QUEUE_SIZE_MAX;
        queue->msix_vector    = VIRTIO_MSI_NO_VECTOR;
        queue->enable         = 0;
        queue->last_avail     = 0;
        queue->used_idx       = 0;
        queue->desc_addr      = 0;
        queue->avail_addr     = 0;
        queue->used_addr      = 0;
    }
}

/*
 * Common configuration register access
 */

static uint32_t virtio_common_read_reg(virtio_dev_t* vdev, size_t off)
{
    virtio_queue_t* queue = &vdev->queues[vdev->queue_select];
    switch (off) {
        case VIRTIO_COMMON_DFSELECT:
            return vdev->device_feature_select;
        case VIRTIO_COMMON_DF:
            if (vdev->device_feature_select == 0) {
                return (uint32_t)vdev->host_features;
            } else if (vdev->device_feature_select == 1) {
                return (uint32_t)(vdev->host_features >> 32);
            }
            return 0;
        case VIRTIO_COMMON_GFSELECT:
            return vdev->driver_feature_select;
        case VIRTIO_COMMON_GF:
            if (vdev->driver_feature_select == 0) {
                return (uint32_t)vdev->driver_features;
            } else if (vdev->driver_feature_select == 1) {
                return (uint32_t)(vdev->driver_features >> 32);
            }
            return 0;
        case VIRTIO_COMMON_MSIX:
            return vdev->msix_config;
        case VIRTIO_COMMON_NUMQ:
            return vdev->num_queues;
        case VIRTIO_COMMON_STATUS:
            return vdev->status;
        case VIRTIO_COMMON_CFGGEN:
            return vdev->config_generation;
        case VIRTIO_COMMON_Q_SELECT:
            return vdev->queue_select;
        case VIRTIO_COMMON_Q_SIZE:
            return queue->size;
        case VIRTIO_COMMON_Q_MSIX:
            return queue->msix_vector;
        case VIRTIO_COMMON_Q_ENABLE:
            return queue->enable;
        case VIRTIO_COMMON_Q_NOFF:
            return vdev->queue_select; // notify offset == queue index
        case VIRTIO_COMMON_Q_DESCLO:
            return (uint32_t)queue->desc_addr;
        case VIRTIO_COMMON_Q_DESCHI:
            return (uint32_t)(queue->desc_addr >> 32);
        case VIRTIO_COMMON_Q_AVAILLO:
            return (uint32_t)queue->avail_addr;
        case VIRTIO_COMMON_Q_AVAILHI:
            return (uint32_t)(queue->avail_addr >> 32);
        case VIRTIO_COMMON_Q_USEDLO:
            return (uint32_t)queue->used_addr;
        case VIRTIO_COMMON_Q_USEDHI:
            return (uint32_t)(queue->used_addr >> 32);
    }
    return 0;
}

static bool virtio_common_write_reg(virtio_dev_t* vdev, size_t off, uint32_t val)
{
    // Returns true if the driver requested a device reset, which the caller
    // must service by invoking cb->reset() *without* holding vdev->lock
    // (the device reset callback may take its own locks).
    virtio_queue_t* queue = &vdev->queues[vdev->queue_select];
    switch (off) {
        case VIRTIO_COMMON_DFSELECT:
            vdev->device_feature_select = val;
            break;
        case VIRTIO_COMMON_GFSELECT:
            vdev->driver_feature_select = val;
            break;
        case VIRTIO_COMMON_GF:
            if (vdev->driver_feature_select == 0) {
                vdev->driver_features = (vdev->driver_features & 0xFFFFFFFF00000000ULL) | val;
            } else if (vdev->driver_feature_select == 1) {
                vdev->driver_features = (vdev->driver_features & 0x00000000FFFFFFFFULL) //
                                      | (((uint64_t)val) << 32);
            }
            break;
        case VIRTIO_COMMON_MSIX:
            vdev->msix_config = (uint16_t)val;
            if ((uint16_t)val != VIRTIO_MSI_NO_VECTOR) {
                vdev->msix_active = true;
            }
            break;
        case VIRTIO_COMMON_STATUS:
            if ((val & 0xFF) == 0) {
                // Device reset: clear transport state now, defer cb->reset
                virtio_do_reset(vdev);
                return true;
            } else {
                vdev->status = (uint8_t)val;
            }
            break;
        case VIRTIO_COMMON_Q_SELECT:
            if ((uint16_t)val < vdev->num_queues) {
                vdev->queue_select = (uint16_t)val;
            }
            break;
        case VIRTIO_COMMON_Q_SIZE:
            if (val && val <= VIRTIO_QUEUE_SIZE_MAX && !(val & (val - 1))) {
                queue->size = (uint16_t)val;
            }
            break;
        case VIRTIO_COMMON_Q_MSIX:
            queue->msix_vector = (uint16_t)val;
            if ((uint16_t)val != VIRTIO_MSI_NO_VECTOR) {
                vdev->msix_active = true;
            }
            break;
        case VIRTIO_COMMON_Q_ENABLE:
            queue->enable = (uint16_t)val;
            break;
        case VIRTIO_COMMON_Q_DESCLO:
            queue->desc_addr = bit_replace64(queue->desc_addr, 0, 32, val);
            break;
        case VIRTIO_COMMON_Q_DESCHI:
            queue->desc_addr = bit_replace64(queue->desc_addr, 32, 32, val);
            break;
        case VIRTIO_COMMON_Q_AVAILLO:
            queue->avail_addr = bit_replace64(queue->avail_addr, 0, 32, val);
            break;
        case VIRTIO_COMMON_Q_AVAILHI:
            queue->avail_addr = bit_replace64(queue->avail_addr, 32, 32, val);
            break;
        case VIRTIO_COMMON_Q_USEDLO:
            queue->used_addr = bit_replace64(queue->used_addr, 0, 32, val);
            break;
        case VIRTIO_COMMON_Q_USEDHI:
            queue->used_addr = bit_replace64(queue->used_addr, 32, 32, val);
            break;
    }
    return false;
}

/*
 * BAR0 region access dispatch
 */

static void virtio_bar_read(rvvm_reg_dev_t* dev, void* data, size_t size, size_t off)
{
    virtio_dev_t* vdev = rvvm_region_data(dev);
    uint32_t      val  = 0;

    if (off < VIRTIO_BAR_ISR_OFF) {
        spin_lock(&vdev->lock);
        val = virtio_common_read_reg(vdev, off - VIRTIO_BAR_COMMON_OFF);
        spin_unlock(&vdev->lock);
    } else if (off >= VIRTIO_BAR_ISR_OFF && off < VIRTIO_BAR_DEVICE_OFF) {
        // Reading ISR status clears it and deasserts INTx
        val = atomic_swap_uint32(&vdev->isr_status, 0);
        if (val) {
            rvvm_pci_set_irq(vdev->func, 0, false);
        }
    } else if (off >= VIRTIO_BAR_DEVICE_OFF && off < VIRTIO_BAR_NOTIFY_OFF) {
        size_t cfg_off = off - VIRTIO_BAR_DEVICE_OFF;
        if (vdev->cb->config_read && cfg_off < vdev->config_size) {
            vdev->cb->config_read(vdev, data, size, cfg_off);
            return;
        }
    }

    switch (size) {
        case 1:
            write_uint8(data, val);
            break;
        case 2:
            write_uint16_le(data, val);
            break;
        default:
            write_uint32_le(data, val);
            break;
    }
}

static void virtio_bar_write(rvvm_reg_dev_t* dev, const void* data, size_t size, size_t off)
{
    virtio_dev_t* vdev = rvvm_region_data(dev);
    uint32_t      val  = 0;
    switch (size) {
        case 1:
            val = read_uint8(data);
            break;
        case 2:
            val = read_uint16_le(data);
            break;
        default:
            val = read_uint32_le(data);
            break;
    }

    if (off < VIRTIO_BAR_ISR_OFF) {
        spin_lock(&vdev->lock);
        bool do_reset = virtio_common_write_reg(vdev, off - VIRTIO_BAR_COMMON_OFF, val);
        spin_unlock(&vdev->lock);
        if (do_reset) {
            // Service reset outside the lock to avoid lock-order inversion
            // with the device's own command-processing lock
            if (vdev->cb->reset) {
                vdev->cb->reset(vdev);
            }
            rvvm_pci_set_irq(vdev->func, 0, false);
        }
    } else if (off >= VIRTIO_BAR_DEVICE_OFF && off < VIRTIO_BAR_NOTIFY_OFF) {
        size_t cfg_off = off - VIRTIO_BAR_DEVICE_OFF;
        if (vdev->cb->config_write && cfg_off < vdev->config_size) {
            vdev->cb->config_write(vdev, data, size, cfg_off);
        }
    } else if (off >= VIRTIO_BAR_NOTIFY_OFF && off < VIRTIO_BAR_SIZE) {
        uint16_t queue_id = (off - VIRTIO_BAR_NOTIFY_OFF) / VIRTIO_NOTIFY_MULTIPLIER;
        if (queue_id < vdev->num_queues && vdev->cb->notify) {
            vdev->cb->notify(vdev, queue_id);
        }
    }
}

/*
 * Virtqueue request handling
 */

static void virtio_req_add_iov(virtio_request_t* req, void* buf, uint32_t len, bool write)
{
    if (req->iov_cnt == req->iov_cap) {
        uint32_t      new_cap = req->iov_cap * 2;
        virtio_iov_t* new_iov = safe_new_arr(virtio_iov_t, new_cap);
        memcpy(new_iov, req->iov, req->iov_cnt * sizeof(virtio_iov_t));
        if (req->iov != req->inline_iov) {
            free(req->iov);
        }
        req->iov     = new_iov;
        req->iov_cap = new_cap;
    }
    req->iov[req->iov_cnt].buf   = buf;
    req->iov[req->iov_cnt].len   = len;
    req->iov[req->iov_cnt].write = write;
    req->iov_cnt++;
}

// Walk a descriptor table starting at `idx`, adding buffers to the request.
// Returns false on malformed chains.
static bool virtio_walk_chain(virtio_request_t* req, const uint8_t* desc, uint16_t size, uint16_t idx)
{
    virtio_dev_t* vdev  = req->vdev;
    uint32_t      guard = 0;
    while (guard++ < size) {
        const uint8_t* d     = desc + ((size_t)idx * 16);
        uint64_t       addr  = read_uint64_le_m(d);
        uint32_t       len   = read_uint32_le_m(d + 8);
        uint16_t       flags = read_uint16_le_m(d + 12);
        uint16_t       next  = read_uint16_le_m(d + 14);

        if (flags & VRING_DESC_F_INDIRECT) {
            // Indirect descriptor table
            uint8_t* itable = virtio_dma_get(vdev, addr, len);
            if (itable && len >= 16) {
                uint16_t inum   = len / 16;
                uint16_t iidx   = 0;
                uint32_t iguard = 0;
                while (iguard++ < inum) {
                    const uint8_t* id     = itable + ((size_t)iidx * 16);
                    uint64_t       iaddr  = read_uint64_le_m(id);
                    uint32_t       ilen   = read_uint32_le_m(id + 8);
                    uint16_t       iflags = read_uint16_le_m(id + 12);
                    uint16_t       inext  = read_uint16_le_m(id + 14);
                    void*          ibuf   = virtio_dma_get(vdev, iaddr, ilen);
                    if (ibuf && req->iov_cnt < VIRTIO_REQ_MAX_IOV) {
                        virtio_req_add_iov(req, ibuf, ilen, !!(iflags & VRING_DESC_F_WRITE));
                    }
                    if (!(iflags & VRING_DESC_F_NEXT) || inext >= inum) {
                        break;
                    }
                    iidx = inext;
                }
                virtio_dma_put(vdev, itable);
            } else if (itable) {
                virtio_dma_put(vdev, itable);
            }
        } else {
            void* buf = virtio_dma_get(vdev, addr, len);
            if (buf && req->iov_cnt < VIRTIO_REQ_MAX_IOV) {
                virtio_req_add_iov(req, buf, len, !!(flags & VRING_DESC_F_WRITE));
            }
        }

        if (!(flags & VRING_DESC_F_NEXT) || next >= size) {
            return true;
        }
        idx = next;
    }
    return false;
}

virtio_request_t* virtio_queue_pop(virtio_dev_t* vdev, uint16_t queue_id)
{
    if (queue_id >= vdev->num_queues) {
        return NULL;
    }

    spin_lock(&vdev->lock);
    virtio_queue_t* queue = &vdev->queues[queue_id];
    if (!queue->enable || !(vdev->status & VIRTIO_STATUS_DRIVER_OK)) {
        spin_unlock(&vdev->lock);
        return NULL;
    }
    uint16_t size       = queue->size;
    uint64_t desc_addr  = queue->desc_addr;
    uint64_t avail_addr = queue->avail_addr;
    uint16_t last_avail = queue->last_avail;

    // Read the available ring index
    uint8_t* avail = virtio_dma_get(vdev, avail_addr, 4 + ((size_t)size + 1) * 2);
    if (!avail) {
        spin_unlock(&vdev->lock);
        return NULL;
    }
    uint16_t avail_idx = read_uint16_le_m(avail + 2);
    atomic_fence_ex(ATOMIC_ACQUIRE);
    if (last_avail == avail_idx) {
        // No new buffers
        virtio_dma_put(vdev, avail);
        spin_unlock(&vdev->lock);
        return NULL;
    }
    uint16_t head = read_uint16_le_m(avail + 4 + ((size_t)(last_avail % size) * 2));
    queue->last_avail++;
    virtio_dma_put(vdev, avail);

    if (head >= size) {
        spin_unlock(&vdev->lock);
        rvvm_warn("virtio: bad descriptor head %u (queue size %u)", head, size);
        return NULL;
    }

    virtio_request_t* req = safe_new_obj(virtio_request_t);
    req->vdev             = vdev;
    req->queue_id         = queue_id;
    req->head             = head;
    req->iov              = req->inline_iov;
    req->iov_cap          = VIRTIO_REQ_INLINE_IOV;

    uint8_t* desc = virtio_dma_get(vdev, desc_addr, (size_t)size * 16);
    if (desc) {
        virtio_walk_chain(req, desc, size, head);
        virtio_dma_put(vdev, desc);
    }

    // Position write cursor at the first device-writable buffer
    req->wr_idx = 0;
    while (req->wr_idx < req->iov_cnt && !req->iov[req->wr_idx].write) {
        req->wr_idx++;
    }

    spin_unlock(&vdev->lock);
    return req;
}

size_t virtio_request_read(virtio_request_t* req, void* dst, size_t len)
{
    uint8_t* out    = dst;
    size_t   copied = 0;
    while (copied < len && req->rd_idx < req->iov_cnt) {
        virtio_iov_t* iov = &req->iov[req->rd_idx];
        if (iov->write) {
            break; // Readable region ended
        }
        if (req->rd_off >= iov->len) {
            req->rd_idx++;
            req->rd_off = 0;
            continue;
        }
        size_t avail = iov->len - req->rd_off;
        size_t chunk = EVAL_MIN(avail, len - copied);
        if (iov->buf) {
            memcpy(out + copied, (const uint8_t*)iov->buf + req->rd_off, chunk);
        } else {
            memset(out + copied, 0, chunk);
        }
        copied += chunk;
        req->rd_off += chunk;
    }
    return copied;
}

size_t virtio_request_write(virtio_request_t* req, const void* src, size_t len)
{
    const uint8_t* in     = src;
    size_t         copied = 0;
    while (copied < len && req->wr_idx < req->iov_cnt) {
        virtio_iov_t* iov = &req->iov[req->wr_idx];
        if (!iov->write) {
            req->wr_idx++;
            req->wr_off = 0;
            continue;
        }
        if (req->wr_off >= iov->len) {
            req->wr_idx++;
            req->wr_off = 0;
            continue;
        }
        size_t avail = iov->len - req->wr_off;
        size_t chunk = EVAL_MIN(avail, len - copied);
        if (iov->buf) {
            memcpy((uint8_t*)iov->buf + req->wr_off, in + copied, chunk);
        }
        copied += chunk;
        req->wr_off += chunk;
    }
    return copied;
}

size_t virtio_request_readable(virtio_request_t* req)
{
    size_t total = 0;
    for (uint32_t i = 0; i < req->iov_cnt; ++i) {
        if (!req->iov[i].write) {
            total += req->iov[i].len;
        }
    }
    return total;
}

size_t virtio_request_writable(virtio_request_t* req)
{
    size_t total = 0;
    for (uint32_t i = 0; i < req->iov_cnt; ++i) {
        if (req->iov[i].write) {
            total += req->iov[i].len;
        }
    }
    return total;
}

void virtio_request_complete(virtio_request_t* req, uint32_t used_len)
{
    virtio_dev_t* vdev     = req->vdev;
    uint16_t      queue_id = req->queue_id;

    spin_lock(&vdev->lock);
    virtio_queue_t* queue = &vdev->queues[queue_id];
    uint16_t        size  = queue->size;
    uint8_t*        used  = virtio_dma_get(vdev, queue->used_addr, 4 + ((size_t)size + 1) * 8);
    if (used) {
        uint16_t slot = queue->used_idx % size;
        // used->ring[slot] = { id = head, len = used_len }
        write_uint32_le_m(used + 4 + ((size_t)slot * 8), req->head);
        write_uint32_le_m(used + 4 + ((size_t)slot * 8) + 4, used_len);
        // Publish the new used index after the ring entry is visible
        atomic_fence_ex(ATOMIC_RELEASE);
        queue->used_idx++;
        write_uint16_le_m(used + 2, queue->used_idx);
        virtio_dma_put(vdev, used);
    }
    spin_unlock(&vdev->lock);

    // Release descriptor buffer mappings
    for (uint32_t i = 0; i < req->iov_cnt; ++i) {
        virtio_dma_put(vdev, req->iov[i].buf);
    }
    if (req->iov != req->inline_iov) {
        free(req->iov);
    }
    free(req);
}

/*
 * Device lifecycle
 */

void* virtio_dev_data(virtio_dev_t* vdev)
{
    return vdev ? vdev->data : NULL;
}

rvvm_pci_func_t* virtio_pci_func(virtio_dev_t* vdev)
{
    return vdev ? vdev->func : NULL;
}

uint64_t virtio_features(virtio_dev_t* vdev)
{
    return vdev ? vdev->driver_features : 0;
}

static void virtio_bar_reset(rvvm_reg_dev_t* dev)
{
    virtio_dev_t* vdev = rvvm_region_data(dev);
    spin_lock(&vdev->lock);
    virtio_do_reset(vdev);
    spin_unlock(&vdev->lock);
    if (vdev->cb->reset) {
        vdev->cb->reset(vdev);
    }
}

static void virtio_bar_cleanup(rvvm_reg_dev_t* dev)
{
    virtio_dev_t* vdev = rvvm_region_data(dev);
    if (vdev->cb->cleanup) {
        vdev->cb->cleanup(vdev);
    }
    free(vdev->queues);
    free(vdev);
}

static const rvvm_reg_type_t virtio_bar_type = {
    .name     = "virtio-pci",
    .read     = virtio_bar_read,
    .write    = virtio_bar_write,
    .reset    = virtio_bar_reset,
    .cleanup  = virtio_bar_cleanup,
    .min_size = 1,
    .max_size = 4,
};

virtio_dev_t* virtio_pci_init(rvvm_machine_t* machine, const virtio_dev_desc_t* desc, rvvm_pci_addr_t addr)
{
    virtio_dev_t* vdev = safe_new_obj(virtio_dev_t);
    vdev->cb           = desc->cb;
    vdev->data         = desc->data;
    vdev->num_queues   = desc->num_queues ? desc->num_queues : 1;
    vdev->config_size  = desc->config_size;
    vdev->host_features = desc->features | (1ULL << VIRTIO_F_VERSION_1);
    spin_init(&vdev->lock);

    vdev->queues = safe_new_arr(virtio_queue_t, vdev->num_queues);
    virtio_do_reset(vdev);
    virtio_build_caps(vdev);

    rvvm_reg_desc_t virtio_bar = {
        .size = VIRTIO_BAR_SIZE,
        .data = vdev,
        .type = &virtio_bar_type,
        .attr = RVVM_REG_ATTR_BAR64,
    };
    rvvm_reg_desc_t virtio_cap = {
        .mmap = vdev->cap_blob,
        .size = sizeof(vdev->cap_blob),
    };
    // Modern VirtIO 1.x PCI: transitional device ID range 0x1040 + device type
    rvvm_pci_func_desc_t virtio_pci_desc = {
        .vendor_id  = 0x1AF4, // Red Hat / VirtIO
        .device_id  = 0x1040 + desc->device_id,
        .subsys_ven = 0x1AF4,
        .subsys_dev = desc->device_id,
        .class_code = desc->class_code,
        .revision   = 0x01, // Modern VirtIO
        .irq_pin    = RVVM_PCI_PIN_INTA,
        .irq_vecs   = 8,
        .bar[0]     = &virtio_bar,
        .cap        = &virtio_cap,
    };

    vdev->func = rvvm_pci_func_init(machine, &virtio_pci_desc, addr);
    if (!vdev->func) {
        // rvvm_pci_func_init invoked cleanup, which freed vdev
        return NULL;
    }
    return vdev;
}

POP_OPTIMIZATION_SIZE
