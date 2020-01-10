/*
 * Copyright (c) 2013, Cisco Systems, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "usnic.h"
#include "usnic_ib_abi.h"
#include "usnic_context.h"

static struct {
	unsigned vendor;
	unsigned device;
} device_table[] = {
	{
		.vendor = PCI_VENDOR_ID_CISCO,
		.device = PCI_DEVICE_ID_CISCO_VIC_ENET}
};

/*
 * These are the two most basic functions of this DSO.  They are
 * associated with usnic_device instances.
 */
static struct ibv_device_ops usnic_dev_ops = {
	.alloc_context = usnic_alloc_context,
	.free_context = usnic_free_context
};

/*
 * This function is invoked when the upper-level libibverbs has
 * successfully dlopened() this file and is looking for a driver for a
 * given device.
 */
static struct ibv_device *usnic_driver_init(const char *uverbs_sys_path,
						int abi_version)
{
	char value[8];
	struct usnic_device *dev;
	unsigned vendor, device;
	unsigned i;

	/* Read the device/vendor and device/device files; look in a
	 * table of (device, vendor) tuples to see if we support this
	 *device.
	 */
	if (ibv_read_sysfs_file(uverbs_sys_path, "device/vendor",
		value, sizeof(value)) < 0)
		return NULL;
	sscanf(value, "%i", &vendor);

	if (ibv_read_sysfs_file(uverbs_sys_path, "device/device",
		value, sizeof(value)) < 0)
		return NULL;
	sscanf(value, "%i", &device);

	for (i = 0; i < sizeof(device_table)/sizeof(device_table[0]); ++i) {
		if (vendor == device_table[i].vendor &&
			device == device_table[i].device)
			goto found;
	}

	return NULL;

found:
	if (abi_version != USNIC_UVERBS_ABI_VERSION) {
		fprintf(stderr, PFX "Fatal: ABI version %d of %s is not "
				"supported, should be %d\n",
			abi_version, uverbs_sys_path,
			USNIC_UVERBS_ABI_VERSION);
		return NULL;
	}

	/* It looks like everything is compatible.  Allocate a device
	 * struct (to include a struct ibv_device).
	 */
	dev = calloc(1, sizeof(*dev));
	if (!dev) {
		fprintf(stderr, PFX "Fatal: couldn't allocate device for %s\n",
		uverbs_sys_path);
		return NULL;
	}

	/* Fill in the function pointers for this device, and other
	 * local information that we want to cache with this
	 * device.
	 */
	dev->ibv_dev.ops = usnic_dev_ops;
	dev->vendor_id = vendor;
	dev->vendor_part_id = device;
	dev->transport = USNIC_TRANSPORT_UNKNOWN;

	/* Return a pointer to the struct ibv_device to the upper
	 * level libibverbs.
	 */
	return &dev->ibv_dev;
}

/*
 * This function is invoked when this DSO is dlopen'ed.  It registers
 * this driver with the upper-level libiverbs.
 */
static __attribute__ ((constructor)) void usnic_register_driver(void)
{
	ibv_register_driver("usnic_verbs", usnic_driver_init);
}
