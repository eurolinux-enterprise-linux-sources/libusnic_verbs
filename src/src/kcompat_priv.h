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

#ifndef USNIC_KCOMPAT_H
#define USNIC_KCOMPAT_H

#include "usnic_utils.h"

typedef uint64_t dma_addr_t;
struct pci_dev;
#define pr_err usnic_err
#define pr_warning usnic_err
#define printk usnic_info

static inline const char *pci_name(const struct pci_dev *UNUSED(pdev))
{
	return "usNIC";
}

static inline void *pci_alloc_consistent(struct pci_dev *hwdev, size_t size,
		dma_addr_t *dma_handle)
{
	 void *va;
	 va = usnic_alloc_consistent((struct usnic_pd *) hwdev, size);
	 *dma_handle = (dma_addr_t)va;
	 return va;
}

static inline void pci_free_consistent(struct pci_dev *UNUSED(hwdev),
		size_t UNUSED(size),
		void *vaddr,
		dma_addr_t UNUSED(dma_handle))
{
	usnic_free_consistent(vaddr);
}

#endif /* USNIC_KCOMPAT_H */
