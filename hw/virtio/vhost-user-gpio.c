/*
 * Vhost-user GPIO virtio device
 *
 * Copyright (c) 2022 Viresh Kumar <viresh.kumar@linaro.org>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/virtio/virtio-bus.h"
#include "hw/virtio/vhost-user-gpio.h"
#include "standard-headers/linux/virtio_ids.h"
#include "standard-headers/linux/virtio_gpio.h"

#define REALIZE_CONNECTION_RETRIES 3
#define VHOST_NVQS 2

/* Features required from VirtIO */
static const int feature_bits[] = {
    VIRTIO_F_VERSION_1,
    VIRTIO_F_NOTIFY_ON_EMPTY,
    VIRTIO_RING_F_INDIRECT_DESC,
    VIRTIO_RING_F_EVENT_IDX,
    VIRTIO_GPIO_F_IRQ,
    VIRTIO_F_RING_RESET,
    VHOST_INVALID_FEATURE_BIT
};

static void vu_gpio_get_config(VirtIODevice *vdev, uint8_t *config)
{
    VHostUserGPIO *gpio = VHOST_USER_GPIO(vdev);

    memcpy(config, &gpio->config, sizeof(gpio->config));
}

static int vu_gpio_config_notifier(struct vhost_dev *dev)
{
    VHostUserGPIO *gpio = VHOST_USER_GPIO(dev->vdev);

    memcpy(dev->vdev->config, &gpio->config, sizeof(gpio->config));
    virtio_notify_config(dev->vdev);

    return 0;
}

const VhostDevConfigOps gpio_ops = {
    .vhost_dev_config_notifier = vu_gpio_config_notifier,
};

static void vgpio_realize(DeviceState *dev, Error **errp)
{
    VHostUserBase *vub = VHOST_USER_BASE(dev);
    VHostUserBaseClass *vubc = VHOST_USER_BASE_GET_CLASS(dev);

    vhost_virtqueue_mask(&gpio->vhost_dev, vdev, idx, mask);
}

static void do_vhost_user_cleanup(VirtIODevice *vdev, VHostUserGPIO *gpio)
{
    virtio_delete_queue(gpio->command_vq);
    virtio_delete_queue(gpio->interrupt_vq);
    g_free(gpio->vhost_vqs);
    virtio_cleanup(vdev);
    vhost_user_cleanup(&gpio->vhost_user);
}

static int vu_gpio_connect(DeviceState *dev, Error **errp)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VHostUserGPIO *gpio = VHOST_USER_GPIO(vdev);
    struct vhost_dev *vhost_dev = &gpio->vhost_dev;
    int ret;

    if (gpio->connected) {
        return 0;
    }
    gpio->connected = true;

    vhost_dev_set_config_notifier(vhost_dev, &gpio_ops);
    gpio->vhost_user.supports_config = true;

    gpio->vhost_dev.nvqs = VHOST_NVQS;
    gpio->vhost_dev.vqs = gpio->vhost_vqs;

    ret = vhost_dev_init(vhost_dev, &gpio->vhost_user,
                         VHOST_BACKEND_TYPE_USER, 0, errp);
    if (ret < 0) {
        return ret;
    }

    /* restore vhost state */
    if (virtio_device_started(vdev, vdev->status)) {
        vu_gpio_start(vdev);
    }

    return 0;
}

static void vu_gpio_event(void *opaque, QEMUChrEvent event);

static void vu_gpio_disconnect(DeviceState *dev)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VHostUserGPIO *gpio = VHOST_USER_GPIO(vdev);

    if (!gpio->connected) {
        return;
    }
    gpio->connected = false;

    vu_gpio_stop(vdev);
    vhost_dev_cleanup(&gpio->vhost_dev);

    /* Re-instate the event handler for new connections */
    qemu_chr_fe_set_handlers(&gpio->chardev,
                             NULL, NULL, vu_gpio_event,
                             NULL, dev, NULL, true);
}

static void vu_gpio_event(void *opaque, QEMUChrEvent event)
{
    DeviceState *dev = opaque;
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VHostUserGPIO *gpio = VHOST_USER_GPIO(vdev);
    Error *local_err = NULL;

    switch (event) {
    case CHR_EVENT_OPENED:
        if (vu_gpio_connect(dev, &local_err) < 0) {
            qemu_chr_fe_disconnect(&gpio->chardev);
            return;
        }
        break;
    case CHR_EVENT_CLOSED:
        /* defer close until later to avoid circular close */
        vhost_user_async_close(dev, &gpio->chardev, &gpio->vhost_dev,
                               vu_gpio_disconnect);
        break;
    case CHR_EVENT_BREAK:
    case CHR_EVENT_MUX_IN:
    case CHR_EVENT_MUX_OUT:
        /* Ignore */
        break;
    }
}

static int vu_gpio_realize_connect(VHostUserGPIO *gpio, Error **errp)
{
    VirtIODevice *vdev = &gpio->parent_obj;
    DeviceState *dev = &vdev->parent_obj;
    struct vhost_dev *vhost_dev = &gpio->vhost_dev;
    int ret;

    ret = qemu_chr_fe_wait_connected(&gpio->chardev, errp);
    if (ret < 0) {
        return ret;
    }

    /*
     * vu_gpio_connect() may have already connected (via the event
     * callback) in which case it will just report success.
     */
    ret = vu_gpio_connect(dev, errp);
    if (ret < 0) {
        qemu_chr_fe_disconnect(&gpio->chardev);
        return ret;
    }
    g_assert(gpio->connected);

    ret = vhost_dev_get_config(vhost_dev, (uint8_t *)&gpio->config,
                               sizeof(gpio->config), errp);

    if (ret < 0) {
        error_report("vhost-user-gpio: get config failed");

        qemu_chr_fe_disconnect(&gpio->chardev);
        vhost_dev_cleanup(vhost_dev);
        return ret;
    }

    return 0;
}

static void vu_gpio_device_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();

    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VHostUserGPIO *gpio = VHOST_USER_GPIO(dev);
    int retries, ret;

    if (!gpio->chardev.chr) {
        error_setg(errp, "vhost-user-gpio: chardev is mandatory");
        return;
    }

    if (!vhost_user_init(&gpio->vhost_user, &gpio->chardev, errp)) {
        return;
    }

    virtio_init(vdev, VIRTIO_ID_GPIO, sizeof(gpio->config));

    gpio->command_vq = virtio_add_queue(vdev, 256, vu_gpio_handle_output);
    gpio->interrupt_vq = virtio_add_queue(vdev, 256, vu_gpio_handle_output);
    gpio->vhost_vqs = g_new0(struct vhost_virtqueue, VHOST_NVQS);

    gpio->connected = false;

    qemu_chr_fe_set_handlers(&gpio->chardev, NULL, NULL, vu_gpio_event, NULL,
                             dev, NULL, true);

    retries = REALIZE_CONNECTION_RETRIES;
    g_assert(!*errp);
    do {
        if (*errp) {
            error_prepend(errp, "Reconnecting after error: ");
            error_report_err(*errp);
            *errp = NULL;
        }
        ret = vu_gpio_realize_connect(gpio, errp);
    } while (ret < 0 && retries--);

    if (ret < 0) {
        do_vhost_user_cleanup(vdev, gpio);
    }

    return;
}

static void vu_gpio_device_unrealize(DeviceState *dev)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);
    VHostUserGPIO *gpio = VHOST_USER_GPIO(dev);

    vubc->parent_realize(dev, errp);
}

static const VMStateDescription vu_gpio_vmstate = {
    .name = "vhost-user-gpio",
    .unmigratable = 1,
};

static void vu_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    VHostUserBaseClass *vubc = VHOST_USER_BASE_CLASS(klass);

    dc->vmsd = &vu_gpio_vmstate;
    device_class_set_props(dc, vgpio_properties);
    device_class_set_parent_realize(dc, vgpio_realize,
                                    &vubc->parent_realize);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo vu_gpio_info = {
    .name = TYPE_VHOST_USER_GPIO,
    .parent = TYPE_VHOST_USER_BASE,
    .instance_size = sizeof(VHostUserGPIO),
    .class_init = vu_gpio_class_init,
};

static void vu_gpio_register_types(void)
{
    type_register_static(&vu_gpio_info);
}

type_init(vu_gpio_register_types)
