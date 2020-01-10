/*
 * Copyright (c) 2014, Cisco Systems, Inc. All rights reserved.
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

#include <config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <pthread.h>
#include <string.h>
#include <assert.h>

#include "usnic.h"
#include "usnic_ib_abi.h"
#include "usnic_utils.h"

#include "kcompat.h"
#include "linux_types.h"

void *usnic_alloc_consistent(struct usnic_pd *pd, size_t size)
{
	void				*vaddr, *base_addr;
	int				err;
	size_t				size_aligned;
	struct usnic_reg_mr		cmd;
	struct usnic_reg_mr_resp	resp;

	memset(&cmd, 0, sizeof(cmd));

	size_aligned = ALIGN(size + sizeof(struct ibv_mr),
			sysconf(_SC_PAGESIZE));
	base_addr = mmap(NULL, size_aligned, PROT_READ | PROT_WRITE,
				MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (base_addr == NULL || base_addr == MAP_FAILED) {
		usnic_err("Failed to mmap region of size %lu\n", size);
		return NULL;
	}

	err = ibv_dontfork_range(base_addr, size_aligned);
	if (err) {
		munmap(base_addr, size_aligned);
		return NULL;
	}

	vaddr = base_addr + sizeof(struct ibv_mr);
	err = ibv_cmd_reg_mr(&pd->ibv_pd, vaddr,
				size_aligned - sizeof(struct ibv_mr),
				(uint64_t)vaddr, IBV_ACCESS_LOCAL_WRITE,
				(struct ibv_mr *) base_addr,
				&cmd.ibv_cmd, sizeof(cmd),
				&resp.ibv_resp, sizeof(resp));
	if (err) {
		usnic_err("Failed to register mr with error %d\n", err);
		ibv_dofork_range(base_addr, size_aligned);
		munmap(base_addr, size_aligned);
		return NULL;
	}

	((struct ibv_mr *)base_addr)->length = size_aligned;
	usnic_verbose("base: %p sizeofmr: %lu Va: %p Size: 0x%lx\n",
			base_addr, sizeof(struct ibv_mr), vaddr,
			size_aligned - sizeof(struct ibv_mr));
	return vaddr;
}

void usnic_free_consistent(void *vaddr)
{
	void				*base_addr;
	struct ibv_mr			*mr;
	size_t				size;
	int				err;

	base_addr = vaddr - sizeof(struct ibv_mr);
	mr = (struct ibv_mr *) base_addr;

	size = mr->length;
	err = ibv_cmd_dereg_mr(mr);
	if (err)
		usnic_err("Failed to free mr with err %d\n", err);

	usnic_verbose("Base: %p mr: %p va: %p size: 0x%lx\n",
				mr, base_addr, vaddr, size);
	ibv_dofork_range(base_addr, size);
	munmap(base_addr, size);
}

void usnic_gid_to_mac(union ibv_gid *gid, uint8_t *mac)
{
	mac[0] = gid->raw[8]^2;
	mac[1] = gid->raw[9];
	mac[2] = gid->raw[10];
	mac[3] = gid->raw[13];
	mac[4] = gid->raw[14];
	mac[5] = gid->raw[15];

}

int usnic_get_min_safe_cqe(int cqe)
{
	if (cqe + 1 <= (VIC_MAX_WQ_DESC_CNT + VIC_MAX_RQ_DESC_CNT)) {
		return ALIGN(cqe,
			MAX(VIC_WQ_DESC_CNT_ALIGN, VIC_RQ_DESC_CNT_ALIGN))
				+ MAX(MAX(VIC_WQ_DESC_CNT_ALIGN,
					VIC_RQ_DESC_CNT_ALIGN),
					VIC_CQ_DESC_CNT_ALIGN);
	} else {
		return ALIGN(cqe, VIC_CQ_DESC_CNT_ALIGN);

	}
}

int gid_is_valid(union ibv_gid *gid)
{
	unsigned int i;
	for (i = 0; i < sizeof(gid); i++) {
		if (gid->raw[i] != 0)
			return 1;
	}
	return 0;
}

const char *usnic_qp_type_to_string(enum ibv_qp_type qp_type)
{
	switch (qp_type) {
	case IBV_QPT_RC:
		return "RC";
	case IBV_QPT_UC:
		return "UC";
	case IBV_QPT_UD:
		return "UD";
	default:
		return "UNKNOWN";
	}
}
