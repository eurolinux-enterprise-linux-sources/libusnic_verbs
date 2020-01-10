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

#ifndef USNIC_H
#define USNIC_H

#include "config.h"

#include <infiniband/arch.h>
#include <infiniband/driver.h>
#include <stddef.h>
#include <net/if.h>

#include "kcompat.h"
#include "cq_enet_desc.h"
#include "usnic_abi.h"
#include "vnic_wq.h"
#include "vnic_rq.h"
#include "vnic_cq.h"
#include "vnic_intr.h"
#include "vnic_enet.h"
#include "wq_enet_desc.h"
#include "rq_enet_desc.h"

#ifndef UNUSED
#	ifdef __GNUC__
#		define UNUSED(x) (UNUSED_ ## x __attribute__((__unused__)))
#	else
#		define UNUSED(x) (UNUSED_ ## x)
#	endif
#endif

#ifdef HAVE_VALGRIND_MEMCHECK_H
#	include <valgrind/memcheck.h>

#	if !defined(VALGRIND_MAKE_MEM_DEFINED) || \
		!defined(VALGRIND_MAKE_MEM_UNDEFINED)
#		warning "Valgrind support requested, but VALGRIND_MAKE_MEM_(UN)DEFINED not available"
#	endif

#endif /* HAVE_VALGRIND_MEMCHECK_H */

#ifndef VALGRIND_MAKE_MEM_DEFINED
#  define VALGRIND_MAKE_MEM_DEFINED(addr, len)
#endif

#ifndef VALGRIND_MAKE_MEM_UNDEFINED
#  define VALGRIND_MAKE_MEM_UNDEFINED(addr, len)
#endif

#define PFX "usnic: "

#define USNIC_MIN_RX_BUF (64)

#define VIC_WQ_DESC_CNT_ALIGN (32)
#define VIC_MAX_WQ_DESC_CNT (1 << 12)
#define VIC_MAX_WQ_DESC_AVAIL ((VIC_MAX_WQ_DESC_CNT) - (1))
#define VIC_RQ_DESC_CNT_ALIGN (32)
#define VIC_MAX_RQ_DESC_CNT (1 << 12)
#define VIC_MAX_RQ_DESC_AVAIL ((VIC_MAX_RQ_DESC_CNT) - (1))
#define VIC_CQ_DESC_CNT_ALIGN (32)
#define VIC_MAX_CQ_DESC_CNT (1 << 16)
#define VIC_MAX_CQ_DESC_AVAIL ((VIC_MAX_CQ_DESC_CNT) - (1))
#define VIC_MAX_PREFETCH_SIZE (992)
#define USNIC_MAX_INLINE_DATA (VIC_MAX_PREFETCH_SIZE - USNIC_HDR_SZ)

#define USNIC_VIC_WQ_DESC_TO_VERBS_WR_CNT(x) (((x)/(2)) - 1)
#define USNIC_VIC_RQ_DESC_TO_VERBS_WR_CNT(x) ((x) - (1))
#define USNIC_VIC_CQ_DESC_TO_VERBS_CQE_CNT(x) ((x) - (1))
#define USNIC_VERBS_WR_CNT_TO_VIC_WQ_DESC(x) ((2)*(x) + 1)
#define USNIC_VERBS_WR_CNT_TO_VIC_RQ_DESC(x) (x + 1)

#define USNIC_MAX_SEND_WR (USNIC_VIC_WQ_DESC_TO_VERBS_WR_CNT( \
				VIC_MAX_WQ_DESC_CNT))
#define USNIC_MAX_RECV_WR (USNIC_VIC_RQ_DESC_TO_VERBS_WR_CNT( \
				VIC_MAX_RQ_DESC_CNT))
#define USNIC_MAX_CQE (USNIC_VIC_CQ_DESC_TO_VERBS_CQE_CNT( \
				VIC_MAX_CQ_DESC_CNT))
#define USNIC_MAX_SEND_SGE (1)
#define USNIC_MAX_RECV_SGE (1)

#define PCI_VENDOR_ID_CISCO (0x1137)
#define PCI_DEVICE_ID_CISCO_VIC_ENET (0x0043)

#define USNIC_DEFAULT_PORT (1)
#define USNIC_DEFAULT_GID_IDX (0)

#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

struct usnic_device {
	struct ibv_device		ibv_dev;
	uint32_t			vendor_id;
	uint32_t			vendor_part_id;
	char				ifname[IFNAMSIZ];
	int				if_index;
	volatile int			transport;
};

struct usnic_vf {
	struct vnic_dev_bar		bar0;
	struct vnic_dev			*vdev;
	struct usnic_vf			*next;
	/* Will also protect the devcmd region*/
	pthread_mutex_t			vf_lock;
	int				ref_cnt;
	unsigned int			id;
};

struct usnic_context {
	struct ibv_context		ibv_ctx;
	pthread_mutex_t			vf_lst_lock;
	struct usnic_vf			*head;
};

struct usnic_pd {
	struct ibv_pd			ibv_pd;
	uint32_t			pdn;
};

struct usnic_cq {
	struct ibv_cq			 ibv_cq;
	pthread_spinlock_t		 lock;
	struct vnic_cq			 vnic_cq;
	struct usnic_qp			*qp;
	uint32_t			 cqn;
	uint32_t			 max_shared_qps;
	uint32_t			 num_shared_qps;
};

struct usnic_qp {
	struct ibv_qp			 ibv_qp;
	pthread_spinlock_t		wq_lock;
	struct vnic_wq			 wq;
	pthread_spinlock_t		rq_lock;
	struct vnic_rq			 rq;
	struct usnic_vf			*vf;
	struct usnic_cq			*wq_cq;
	struct usnic_cq			*rq_cq;
	void				*shdrs;
	void				*shdrs_unaligned;
	int				sq_sig_all;
	int				qp_socket;
};

#define usnic_container_of(member, type) \
	((struct usnic_##type *) \
	((char *) ib##member - offsetof(struct usnic_##type, ibv_##member)))

static inline struct usnic_device *to_udev(struct ibv_device *ibdev)
{
	return usnic_container_of(dev, device);
}

static inline struct usnic_context *to_uctx(struct ibv_context *ibctx)
{
	return usnic_container_of(ctx, context);
}

static inline struct usnic_pd *to_upd(struct ibv_pd *ibpd)
{
	return usnic_container_of(pd, pd);
}

static inline struct usnic_cq *to_ucq(struct ibv_cq *ibcq)
{
	return usnic_container_of(cq, cq);
}

static inline struct usnic_qp *to_uqp(struct ibv_qp *ibqp)
{
	return usnic_container_of(qp, qp);
}

static inline void *usnic_get_shdr_buf(struct usnic_qp *qp, int index)
{
	return qp->shdrs + VIC_MAX_PREFETCH_SIZE*index;
}
#endif /* !USNIC_H */
