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

#include "config.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <netinet/in.h>
#include <assert.h>
#include <sys/mman.h>

#include "usnic.h"
#include "usnic_ib_abi.h"
#include "usnic_utils.h"
#include "usnic_transport_priv.h"

static void usnic_wq_desc_to_work_comp(struct cq_enet_wq_desc *desc,
					struct ibv_wc *wc, struct vnic_wq *wq,
					struct vnic_wq_buf *buf,
					uint8_t compressed_send)
{
	u8 type, color;
	u16 q_number, completed_index;
	cq_enet_wq_desc_dec(desc, &type, &color, &q_number, &completed_index);

	wc->wr_id = buf->wr_id;
	wc->byte_len = buf->len;
	if (likely(compressed_send))
		wc->byte_len -= USNIC_HDR_SZ;
	wc->status = IBV_WC_SUCCESS;
	wc->opcode = IBV_WC_SEND;
	wc->qp_num = wq->qp_num;
	wc->slid = 0;
}

static unsigned int usnic_cqe_to_rq_descs(struct vnic_rq *rq,
						struct cq_desc *cqe,
						u16 completed_index,
						unsigned int comp_list_len,
						unsigned int *next_avail_idx,
						struct ibv_wc *wc)
{
	struct vnic_rq_buf *buf;
	unsigned int i;
	int consumed = 0;

	i = *next_avail_idx;
	while (i < comp_list_len) {
		buf = rq->to_clean;
		consumed = (buf->index == completed_index);
		usnic_rq_desc_to_work_comp((struct cq_enet_rq_desc *)cqe,
						wc + i, rq, buf);

		i++;
		rq->ring.desc_avail++;
		rq->to_clean = buf->next;

		if (consumed)
			break;
	}
	*next_avail_idx = i;
	return consumed;
}

static unsigned int usnic_cqe_to_wq_descs(struct vnic_wq *wq,
						struct cq_desc *cqe,
						u16 completed_index,
						unsigned int comp_list_len,
						unsigned int *next_avail_idx,
						struct ibv_wc *wc)
{
	struct vnic_wq_buf *buf;
	unsigned int i;
	int consumed = 0;

	i = *next_avail_idx;
	while (i < comp_list_len) {
		buf = wq->to_clean;
		while (!buf->cq_entry) {
			wq->ring.desc_avail += buf->desc_skip_cnt;
			wq->to_clean = buf->next;
			buf = buf->next;
		}
		consumed = (buf->index == completed_index);
		usnic_wq_desc_to_work_comp((struct cq_enet_wq_desc *) cqe,
						wc + i, wq, buf,
						buf->compressed_send);

		i++;
		wq->ring.desc_avail += buf->desc_skip_cnt;
		wq->to_clean = buf->next;

		if (consumed)
			break;
	}
	*next_avail_idx = i;
	return consumed;
}

static unsigned int usnic_cqe_to_completions(struct vnic_wq *wq,
						struct vnic_rq *rq,
						struct cq_desc *cqe,
						u16 completed_index,
						u8 type,
						unsigned int comp_list_len,
						unsigned int *next_avail_idx,
						struct ibv_wc *wc)
{
	unsigned int consumed = 0;
	switch (type) {
	case CQ_DESC_TYPE_CLASSIFIER:
		consumed = usnic_cqe_to_rq_descs(rq, cqe,
				completed_index, comp_list_len, next_avail_idx,
				wc);
		break;
	case CQ_DESC_TYPE_WQ_ENET:
		consumed = usnic_cqe_to_wq_descs(wq, cqe,
				completed_index, comp_list_len, next_avail_idx,
				wc);
		break;
	default:
		assert(0);
		break;
	}
	return consumed;
}

static unsigned int _usnic_poll_cq(struct vnic_cq *cq, struct vnic_wq *wq,
				struct vnic_rq *rq, unsigned int comp_list_len,
				struct ibv_wc *wc, unsigned int *out_credits)
{
	struct cq_desc *cq_desc;
	unsigned int cqes_done = 0;
	unsigned int i;
	unsigned int consumed;
	u16 q_number, completed_index;
	u8 type, color;

	for (i = 0; i < comp_list_len; ) {
		cq_desc = (struct cq_desc *)((u8 *)cq->ring.descs +
				cq->ring.desc_size * cq->to_clean);
		cq_color_dec(cq_desc, &color);

		if (color == cq->last_color)
			break;

		/* Microoptimization: make copy of cq desc on the stack and use
		 * that for our processing.  This avoids cache thrashing if our
		 * reads race against adjacent writes when the NIC writes the
		 * next descriptor as we are processing this one.  Good for
		 * about 30 nsec on average in my tests.  -AJF
		 */
		struct cq_desc cqe_copy;
		/* color bit we read above is written last; preserve that */
		rmb();
		cqe_copy = *cq_desc;
		cq_desc_dec(&cqe_copy, &type, &color, &q_number,
				&completed_index);
		consumed = usnic_cqe_to_completions(wq, rq, &cqe_copy,
							completed_index,
			type, comp_list_len, &i, wc);

		if (!consumed)
			break;

		cqes_done++;
		cq->to_clean++;
		if (cq->to_clean == cq->ring.desc_count) {
			cq->to_clean = 0;
			cq->last_color = cq->last_color ? 0 : 1;
		}
	}
	*out_credits = cqes_done;
	return i;
}

int usnic_poll_cq(struct ibv_cq *ibcq, int ne, struct ibv_wc *wc)
{
	struct usnic_cq *cq = to_ucq(ibcq);
	struct vnic_cq *vnic_cq = &cq->vnic_cq;
	int n;
	unsigned int credits_done = 0;

	pthread_spin_lock(&cq->lock);
	n = _usnic_poll_cq(vnic_cq, &cq->qp->wq, &cq->qp->rq, ne, wc,
			&credits_done);
	pthread_spin_unlock(&cq->lock);
	return n;
}
