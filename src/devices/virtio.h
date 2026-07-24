/*
virtio.h - VirtIO MMIO/PCI transport (Modern VirtIO 1.x)
Copyright (C) 2026  RVVM Contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef RVVM_VIRTIO_H
#define RVVM_VIRTIO_H

#include <rvvm/rvvm.h>
#include <rvvm/rvvm_pci.h>

/*
 * VirtIO device type IDs (VirtIO spec, section 5)
 */
#define VIRTIO_ID_NET     1
#define VIRTIO_ID_BLOCK   2
#define VIRTIO_ID_CONSOLE 3
#define VIRTIO_ID_RNG     4
#define VIRTIO_ID_GPU     16
#define VIRTIO_ID_INPUT   18

/*
 * Transport (reserved) feature bits
 */
#define VIRTIO_F_RING_INDIRECT_DESC 28
#define VIRTIO_F_RING_EVENT_IDX     29
#define VIRTIO_F_VERSION_1          32
#define VIRTIO_F_ACCESS_PLATFORM    33

/*
 * Opaque VirtIO device handle
 */
typedef struct virtio_dev virtio_dev_t;

/*
 * Opaque VirtIO virtqueue request (descriptor chain) handle
 */
typedef struct virtio_request virtio_request_t;

/*
 * Device-specific callbacks
 */
typedef struct {
    /* Read `size` bytes of device-specific config at `off` into `data` */
    void (*config_read)(virtio_dev_t* vdev, void* data, size_t size, size_t off);

    /* Write `size` bytes of device-specific config at `off` from `data` */
    void (*config_write)(virtio_dev_t* vdev, const void* data, size_t size, size_t off);

    /* A queue was kicked (notified) by the driver, process available buffers */
    void (*notify)(virtio_dev_t* vdev, uint16_t queue_id);

    /* Periodic poll at host refresh rate (display refresh, input polling) */
    void (*poll)(virtio_dev_t* vdev);

    /* Device was reset (status cleared), drop all in-flight state */
    void (*reset)(virtio_dev_t* vdev);

    /* Device is being torn down, release private data */
    void (*cleanup)(virtio_dev_t* vdev);
} virtio_dev_cb_t;

/*
 * VirtIO device description, copied by value by virtio_pci_init()
 */
typedef struct {
    const char*            name;        /* Device name for logging               */
    uint32_t               device_id;   /* VirtIO device type (VIRTIO_ID_*)       */
    uint16_t               class_code;  /* PCI class code                         */
    uint16_t               num_queues;  /* Number of virtqueues                   */
    uint32_t               config_size; /* Device-specific config space size      */
    uint64_t               features;    /* Offered features (VERSION_1 auto-added) */
    const virtio_dev_cb_t* cb;          /* Device callbacks                       */
    void*                  data;        /* Device private data                    */
} virtio_dev_desc_t;

/*
 * Attach a modern VirtIO device to the PCI bus.
 *
 * \param machine Machine handle (Nullable, invokes cleanup)
 * \param desc    VirtIO device description
 * \param addr    PCI bus address (RVVM_PCI_ADDR_ANY for auto)
 * \return        VirtIO device handle (NULL on failure)
 */
virtio_dev_t* virtio_pci_init(rvvm_machine_t* machine, const virtio_dev_desc_t* desc, rvvm_pci_addr_t addr);

/*
 * Get device private data
 */
void* virtio_dev_data(virtio_dev_t* vdev);

/*
 * Get underlying PCI function handle
 */
rvvm_pci_func_t* virtio_pci_func(virtio_dev_t* vdev);

/*
 * Get features negotiated with the driver (valid once DRIVER_OK is set)
 */
uint64_t virtio_features(virtio_dev_t* vdev);

/*
 * Raise a device configuration change notification
 */
void virtio_notify_config(virtio_dev_t* vdev);

/*
 * Obtain a direct mapping into guest physical memory for DMA.
 * Must be released via virtio_dma_put().
 */
void* virtio_dma_get(virtio_dev_t* vdev, rvvm_addr_t addr, size_t size);

/*
 * Release a DMA mapping obtained via virtio_dma_get()
 */
void virtio_dma_put(virtio_dev_t* vdev, void* ptr);

/*
 * Pop the next available descriptor chain from a virtqueue.
 *
 * Returns NULL if the queue is not ready or there are no available buffers.
 * The returned request must eventually be completed via virtio_request_complete().
 */
virtio_request_t* virtio_queue_pop(virtio_dev_t* vdev, uint16_t queue_id);

/*
 * Sequentially read bytes from the device-readable (driver->device) portion
 * of a request. Returns the number of bytes actually read.
 */
size_t virtio_request_read(virtio_request_t* req, void* dst, size_t len);

/*
 * Sequentially write bytes to the device-writable (device->driver) portion
 * of a request. Returns the number of bytes actually written.
 */
size_t virtio_request_write(virtio_request_t* req, const void* src, size_t len);

/*
 * Total number of device-readable bytes in the request
 */
size_t virtio_request_readable(virtio_request_t* req);

/*
 * Total number of device-writable bytes in the request
 */
size_t virtio_request_writable(virtio_request_t* req);

/*
 * Complete a request, publishing `used_len` written bytes to the used ring
 * and releasing all associated DMA mappings.
 */
void virtio_request_complete(virtio_request_t* req, uint32_t used_len);

/*
 * Raise the interrupt for a queue after completing one or more requests,
 * honoring the driver's suppression flags.
 */
void virtio_queue_interrupt(virtio_dev_t* vdev, uint16_t queue_id);

#endif
