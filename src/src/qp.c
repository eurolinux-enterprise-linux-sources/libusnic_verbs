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

static int usnic_qp_dump(char *buf, int buf_sz, struct usnic_qp *qp)
{
	int offset = 0;
	offset += snprintf(buf, buf_sz,
			"QP: %d TYPE:%s VF:%d\n"
			"RES\t|RING_BASE\t|DSC_CNT|VERBS_WR_CNT\t|CQ\n"
			"WQ[%d]\t|%lu\t|%d\t|%d\t\t|%d\t\n"
			"RQ[%d]\t|%lu\t|%d\t|%d\t\t|%d\t\n"
			"CQ[%d]\t|%lu\t|%d\t|%d\t\t|N/A\t\n",
			qp->ibv_qp.qp_num,
			usnic_qp_type_to_string(qp->ibv_qp.qp_type),
			qp->vf->id,
			0, qp->wq.ring.base_addr, qp->wq.ring.desc_count,
			USNIC_VIC_WQ_DESC_TO_VERBS_WR_CNT(
				qp->wq.ring.desc_count),
			qp->wq_cq->ibv_cq.handle,
			0, qp->rq.ring.base_addr, qp->rq.ring.desc_count,
			USNIC_VIC_RQ_DESC_TO_VERBS_WR_CNT(
				qp->rq.ring.desc_count),
			0, qp->wq_cq->ibv_cq.handle,
			qp->wq_cq->vnic_cq.ring.base_addr,
			qp->wq_cq->vnic_cq.ring.desc_count,
			USNIC_VIC_CQ_DESC_TO_VERBS_CQE_CNT(
				qp->wq_cq->vnic_cq.ring.desc_count));

	if (qp->wq_cq != qp->rq_cq) {
		offset += snprintf(buf + offset, buf_sz - offset,
				"CQ[%d]\t|%lu\t|%d\t|%d\t\t|N/A\t\n",
				qp->rq_cq->ibv_cq.handle,
				qp->rq_cq->vnic_cq.ring.base_addr,
				qp->rq_cq->vnic_cq.ring.desc_count,
				USNIC_VIC_CQ_DESC_TO_VERBS_CQE_CNT(
					qp->rq_cq->vnic_cq.ring.desc_count));
	}
	offset += snprintf(buf + offset, buf_sz - offset, "\n");
	return offset;
}

static void usnic_qp_log(struct usnic_qp *qp)
{
	char buf[512];

	if (USNIC_LOG_LVL_INFO > USNIC_LOG_LVL)
		return;
	usnic_qp_dump(buf, sizeof(buf), qp);
	usnic_info(buf);
}

static void usnic_qp_resp_log(struct usnic_create_qp_resp *resp)
{
	char buf[512];
	unsigned int i, offset;

	if (USNIC_LOG_LVL_INFO > USNIC_LOG_LVL)
		return;

	offset = snprintf(buf, sizeof(buf),
			"vfid: %u bus_addr: 0x%lx bar_len: 0x%x qp_num: 0x%x wq_cnt: %d rq_cnt:%d cq_cnt:%d ",
			resp->usnic_resp.vfid, resp->usnic_resp.bar_bus_addr,
			resp->usnic_resp.bar_len, resp->usnic_resp.qp_grp_id,
			resp->usnic_resp.wq_cnt, resp->usnic_resp.rq_cnt,
			resp->usnic_resp.cq_cnt);
	for (i = 0; i < MIN(resp->usnic_resp.wq_cnt, USNIC_QP_GRP_MAX_WQS);
			i++) {
		offset += snprintf(buf + offset, sizeof(buf) - offset,
				"wq[%d]: %d ", i, resp->usnic_resp.wq_idx[i]);
	}
	for (i = 0; i < MIN(resp->usnic_resp.rq_cnt, USNIC_QP_GRP_MAX_RQS);
			i++) {
		offset += snprintf(buf + offset, sizeof(buf) - offset,
				"rq[%d]: %d ", i, resp->usnic_resp.rq_idx[i]);
	}
	for (i = 0; i < MIN(resp->usnic_resp.cq_cnt, USNIC_QP_GRP_MAX_CQS);
			i++) {
		offset += snprintf(buf + offset, sizeof(buf) - offset,
				"cq[%d]: %d ", i, resp->usnic_resp.cq_idx[i]);
	}

	offset += snprintf(buf + offset, sizeof(buf) - offset, "\n");
	usnic_info(buf);
}

static int usnic_qp_cq_bind_check(struct usnic_qp *qp)
{
	struct usnic_cq *wq_cq, *rq_cq;

	wq_cq = qp->wq_cq;
	rq_cq = qp->rq_cq;

	if (wq_cq != rq_cq) {
		if ((USNIC_VIC_CQ_DESC_TO_VERBS_CQE_CNT(
				wq_cq->vnic_cq.ring.desc_count) <
			USNIC_VIC_WQ_DESC_TO_VERBS_WR_CNT(
				qp->wq.ring.desc_count))
			||
			(USNIC_VIC_CQ_DESC_TO_VERBS_CQE_CNT(
				rq_cq->vnic_cq.ring.desc_count) <
			 USNIC_VIC_RQ_DESC_TO_VERBS_WR_CNT(
				 qp->rq.ring.desc_count))) {
				usnic_err("(CQE (%d) < WQE (%d)) OR (CQE (%d) < RQE (%d))",
					wq_cq->vnic_cq.ring.desc_avail,
					qp->wq.ring.desc_avail,
					rq_cq->vnic_cq.ring.desc_avail,
					qp->rq.ring.desc_avail);
				return -1;
		}
	} else {
		if (USNIC_VIC_CQ_DESC_TO_VERBS_CQE_CNT(
					wq_cq->vnic_cq.ring.desc_count) <
			(USNIC_VIC_WQ_DESC_TO_VERBS_WR_CNT(
					qp->wq.ring.desc_count) +
			USNIC_VIC_RQ_DESC_TO_VERBS_WR_CNT(
					qp->rq.ring.desc_count))) {
				usnic_err("CQE (%d) < (RQE (%d) + WQE (%d))",
				wq_cq->vnic_cq.ring.desc_avail,
				qp->rq.ring.desc_avail,
				qp->wq.ring.desc_avail);
				return -1;
		}
	}
	return 0;
}

static int usnic_init_qp_shdrs(struct usnic_qp *qp)
{
	int err;
	int i;
	union ibv_gid my_gid;
	int hdr_cnt;

	err = ibv_query_gid(qp->ibv_qp.context, USNIC_DEFAULT_PORT,
				USNIC_DEFAULT_GID_IDX, &my_gid);
	if (err) {
		usnic_err("Unable to get gid with err %d\n",
				err);
		return err;
	}

	hdr_cnt = qp->wq.ring.desc_count;
	for (i = 0; i < hdr_cnt; i++)
		usnic_write_hdr_at_ring_init(usnic_get_shdr_buf(qp, i), qp,
						&my_gid);
	return 0;
}

void usnic_wq_init(struct usnic_qp *qp)
{
	qp->wq.qp_num = qp->ibv_qp.qp_num;
}

void usnic_rq_init(struct usnic_qp *qp)
{
	qp->rq.qp_num = qp->ibv_qp.qp_num;
}

static int usnic_alloc_qp_and_cq_resources(struct usnic_qp *qp,
						int wq_idx, int rq_idx,
						int cq_wq_idx, int cq_rq_idx,
						int wq_desc_cnt,
						int rq_desc_cnt)
{
	int err;
	struct usnic_vf *vf;
	uint64_t shdrs_size_align;

	vf = qp->vf;

	err = vnic_wq_alloc(vf->vdev, &qp->wq, wq_idx,
		wq_desc_cnt,
		sizeof(struct wq_enet_desc));
	if (err)
		return err;

	unsigned int wq_err_intr_enable = 0;
	unsigned int wq_err_intr_offset = 0;
	vnic_wq_init(&qp->wq, cq_wq_idx, wq_err_intr_enable,
			wq_err_intr_offset);

	usnic_wq_init(qp);

	err = vnic_rq_alloc(vf->vdev, &qp->rq, rq_idx, rq_desc_cnt,
					sizeof(struct rq_enet_desc));
	if (err)
		goto err_rq_alloc;

	unsigned int rq_err_intr_enable = 0;
	unsigned int rq_err_intr_offset = 0;
	vnic_rq_init(&qp->rq, cq_rq_idx, rq_err_intr_enable,
			rq_err_intr_offset);

	usnic_rq_init(qp);

	shdrs_size_align = 32;
	qp->shdrs_unaligned = usnic_alloc_consistent(to_upd(qp->ibv_qp.pd),
				VIC_MAX_PREFETCH_SIZE*wq_desc_cnt+
				shdrs_size_align);

	if (!qp->shdrs_unaligned) {
		err = (-1);
		goto err_shdrs;
	}

	qp->shdrs = (void *)ALIGN((uint64_t)qp->shdrs_unaligned,
					shdrs_size_align);

	err = usnic_init_qp_shdrs(qp);
	if (err) {
		usnic_err("Failed to init qp hdrs\n");
		goto err_shdrs_init;
	}

	err = vnic_cq_alloc(vf->vdev, &qp->wq_cq->vnic_cq, cq_wq_idx,
				qp->wq_cq->ibv_cq.cqe,
				sizeof(struct cq_enet_wq_desc));

	if (err) {
		usnic_err("Failed to allocate CQ\n");
		goto err_shdrs_init;
	}

	unsigned int cq_flow_control_enable = 0, cq_color_enable = 1;
	unsigned int cq_head = 0, cq_tail = 0, cq_tail_color = 1;
	unsigned int cq_intr_enable = 0, cq_entry_enable = 1;
	unsigned int cq_msg_enable = 0, cq_intr_offset = 0;
	u64 cq_msg_addr = 0;

	vnic_cq_init(&qp->wq_cq->vnic_cq, cq_flow_control_enable,
			cq_color_enable, cq_head, cq_tail, cq_tail_color,
			cq_intr_enable, cq_entry_enable, cq_msg_enable,
			cq_intr_offset, cq_msg_addr);

	if (cq_rq_idx != cq_wq_idx) {
		err = vnic_cq_alloc(vf->vdev, &qp->rq_cq->vnic_cq, cq_rq_idx,
					qp->rq_cq->ibv_cq.cqe,
					sizeof(struct cq_enet_rq_desc));

		if (err) {
			usnic_err("Failed to allocate RQ CQ\n");
			goto err_cq_two_alloc;
		}

		vnic_cq_init(&qp->rq_cq->vnic_cq, cq_flow_control_enable,
				cq_color_enable, cq_head, cq_tail,
				cq_tail_color, cq_intr_enable, cq_entry_enable,
				cq_msg_enable, cq_intr_offset, cq_msg_addr);
	}

	return 0;

err_cq_two_alloc:
	vnic_cq_free(&qp->wq_cq->vnic_cq);
err_shdrs_init:
	usnic_free_consistent(qp->shdrs_unaligned);
err_shdrs:
	vnic_rq_free(&qp->rq);
err_rq_alloc:
	vnic_wq_free(&qp->wq);

	return err;
}

static int usnic_alloc_qp_and_cq_rings(struct usnic_qp *qp,
					struct ibv_qp_init_attr *attr,
					struct usnic_create_qp_resp *resp)
{
	struct usnic_ib_create_qp_resp *urp;
	int err;
	int vic_wq_desc_cnt;
	int vic_rq_desc_cnt;

	urp = &resp->usnic_resp;
	vic_wq_desc_cnt = ALIGN(USNIC_VERBS_WR_CNT_TO_VIC_WQ_DESC(
					attr->cap.max_send_wr),
					VIC_WQ_DESC_CNT_ALIGN);
	vic_rq_desc_cnt = ALIGN(USNIC_VERBS_WR_CNT_TO_VIC_RQ_DESC(
					attr->cap.max_recv_wr),
					VIC_RQ_DESC_CNT_ALIGN);
	err = usnic_alloc_qp_and_cq_resources(qp, urp->wq_idx[0],
						urp->rq_idx[0],
						urp->cq_idx[0],
						urp->cq_idx[(urp->cq_cnt == 1)
								? 0 : 1],
						vic_wq_desc_cnt,
						vic_rq_desc_cnt);
	return err;
}

static void usnic_free_qp_and_cq_rings(struct usnic_qp *qp)
{
	if (qp->wq_cq == qp->rq_cq) {
		vnic_cq_free(&qp->wq_cq->vnic_cq);
	} else {
		vnic_cq_free(&qp->wq_cq->vnic_cq);
		vnic_cq_free(&qp->rq_cq->vnic_cq);
	}

	vnic_wq_free(&qp->wq);
	vnic_rq_free(&qp->rq);

	usnic_free_consistent(qp->shdrs_unaligned);
}

static struct usnic_vf*
usnic_find_vf_in_ctx(struct usnic_context *ctx, unsigned int vfid)
{
	struct usnic_vf *curr_vf;

	curr_vf = ctx->head;
	while (curr_vf) {
		if (curr_vf->id == vfid)
			return curr_vf;
		curr_vf = curr_vf->next;
	}
	return NULL;
}

static void usnic_add_vf_to_ctx(struct usnic_context *ctx,
					struct usnic_vf *vf)
{
	struct usnic_vf *curr_vf;

	if (!ctx->head) {
		ctx->head = vf;
		return;
	}

	curr_vf = ctx->head;
	while (curr_vf->next)
		curr_vf = curr_vf->next;

	curr_vf->next = vf;
}

static void usnic_remove_vf_from_ctx(struct usnic_context *ctx,
					struct usnic_vf *vf)
{
	struct usnic_vf *prev_vf, *curr_vf;

	if (vf == ctx->head) {
		ctx->head = vf->next;
		vf->next = NULL;
		return;
	}

	prev_vf = ctx->head;
	curr_vf = (ctx->head) ? ctx->head->next : NULL;
	while (curr_vf) {
		if (curr_vf == vf) {
			prev_vf->next = vf->next;
			vf->next = NULL;
			return;
		}
		prev_vf = curr_vf;
		curr_vf = curr_vf->next;
	}

	assert(0);
}

static int usnic_qp_attr_parse(struct ibv_qp_init_attr *attr)
{
	if (attr->cap.max_send_sge	> USNIC_MAX_SEND_SGE ||
		attr->cap.max_recv_sge	> USNIC_MAX_RECV_SGE ||
		attr->cap.max_inline_data	> USNIC_MAX_INLINE_DATA ||
		attr->srq) {
		return EINVAL;
	}

	switch (attr->qp_type) {
	case (IBV_QPT_UD):
		if (attr->cap.max_send_wr > USNIC_MAX_SEND_WR ||
			attr->cap.max_recv_wr > USNIC_MAX_RECV_WR) {
			return EINVAL;
		}
		break;
	default:
		return EINVAL;
	}

	return 0;
}

static int usnic_validate_qp_resp(struct usnic_create_qp_resp *resp,
				struct ibv_qp_init_attr *attr)
{
	struct usnic_ib_create_qp_resp *urp;
	usnic_qp_resp_log(resp);

	urp = &resp->usnic_resp;
	assert(urp->wq_cnt > 0 && urp->rq_cnt > 0);
	assert(((attr->recv_cq != attr->send_cq) || (urp->cq_cnt == 1)) &&
			((attr->recv_cq == attr->send_cq) ||
			(urp->cq_cnt == 2)));
	return 0;
}

static int usnic_discover_vf(struct usnic_create_qp_resp *resp,
				struct usnic_context *uctx,
				struct usnic_pd *upd,
				struct usnic_vf **vf_to_return)
{
	struct usnic_ib_create_qp_resp *urp;
	struct usnic_vf *vf;

	urp = &resp->usnic_resp;

	pthread_mutex_lock(&uctx->vf_lst_lock);
	vf = usnic_find_vf_in_ctx(uctx, urp->vfid);
	if (!vf) {
		vf = calloc(1, sizeof(*vf));
		if (!vf) {
			usnic_err("Failed to create vf\n");
			pthread_mutex_unlock(&uctx->vf_lst_lock);
			return ENOMEM;
		}
		vf->id = urp->vfid;
		vf->ref_cnt = 1;
		vf->bar0.bus_addr = urp->bar_bus_addr;
		vf->bar0.len = urp->bar_len;
		vf->bar0.vaddr = mmap64(NULL, vf->bar0.len,
					PROT_READ + PROT_WRITE, MAP_SHARED,
					uctx->ibv_ctx.cmd_fd,
					vf->id*sysconf(_SC_PAGE_SIZE));
		if (vf->bar0.vaddr == MAP_FAILED) {
			usnic_err("Failed to mmap bar\n");
			free(vf);
			pthread_mutex_unlock(&uctx->vf_lst_lock);
			return EFAULT;
		}
		usnic_info("Mapped VF %d bar0 at va:0x%p\n",
				vf->id, vf->bar0.vaddr);
		/*
		 * calling vnic_dev_alloc_disover here as libusnic_verb
		 * does not make use of devcmd now.
		 */
		vf->vdev = vnic_dev_alloc_discover(NULL, NULL, (void *)upd,
				&vf->bar0, 1);
		if (vf->vdev == NULL) {
			usnic_err("Failed to create vdev\n");
			munmap(vf->bar0.vaddr, vf->bar0.len);
			free(vf->vdev);
			pthread_mutex_unlock(&uctx->vf_lst_lock);
			return ENOENT;
		}
		usnic_add_vf_to_ctx(uctx, vf);
	} else {
		assert(vf->id == urp->vfid);
		assert(vf->bar0.bus_addr == urp->bar_bus_addr);
		assert(vf->bar0.len == urp->bar_len);
		vf->ref_cnt++;
	}
	usnic_info("VF: %u ref_cnt: %u\n", vf->id, vf->ref_cnt);
	*vf_to_return = vf;
	pthread_mutex_unlock(&uctx->vf_lst_lock);
	return 0;
}

static void usnic_undiscover_vf(struct usnic_qp *qp)
{
	struct usnic_context *uctx = to_uctx(qp->ibv_qp.context);
	struct usnic_vf *vf = qp->vf;

	pthread_mutex_lock(&uctx->vf_lst_lock);
	if (--vf->ref_cnt == 0) {
		vnic_dev_unregister(vf->vdev);
		munmap(vf->bar0.vaddr, vf->bar0.len);
		usnic_remove_vf_from_ctx(uctx, vf);
		free(vf);
	}
	pthread_mutex_unlock(&uctx->vf_lst_lock);
}

static void usnic_init_qp(struct usnic_qp *qp, struct ibv_pd *pd,
				struct ibv_qp_init_attr *attr,
				struct usnic_vf *vf)
{
	qp->vf = vf;
	qp->ibv_qp.pd = pd;
	qp->ibv_qp.qp_context = to_uctx(pd->context);
	qp->ibv_qp.qp_type = attr->qp_type;
	qp->wq_cq = to_ucq(attr->send_cq);
	qp->rq_cq = to_ucq(attr->recv_cq);
	to_ucq(attr->recv_cq)->qp = qp;
	to_ucq(attr->send_cq)->qp = qp;
	pthread_spin_init(&qp->wq_lock, PTHREAD_PROCESS_PRIVATE);
	pthread_spin_init(&qp->rq_lock, PTHREAD_PROCESS_PRIVATE);
	qp->sq_sig_all = attr->sq_sig_all;
}

static void usnic_qp_attr_set(struct usnic_qp *qp,
				struct ibv_qp_init_attr *attr)
{
	switch (qp->ibv_qp.qp_type) {
	case IBV_QPT_UD:
		attr->cap.max_send_wr =
			USNIC_VIC_WQ_DESC_TO_VERBS_WR_CNT(
					qp->wq.ring.desc_count);
		attr->cap.max_recv_wr =
			USNIC_VIC_RQ_DESC_TO_VERBS_WR_CNT(
					qp->rq.ring.desc_count);
		break;
	default:
		assert(0);

	}

	attr->cap.max_send_sge = USNIC_MAX_SEND_SGE;
	attr->cap.max_recv_sge = USNIC_MAX_RECV_SGE;
	attr->cap.max_inline_data = USNIC_MAX_INLINE_DATA;
}

struct ibv_qp *usnic_create_qp_allocated(struct ibv_pd *pd, struct ibv_qp_init_attr *attr, struct usnic_qp *qp)
{
	struct usnic_create_qp		cmd;
	struct usnic_create_qp_resp	resp;
	struct usnic_vf			*vf;
	struct usnic_context		*uctx;
	int				err;

	if (pd->context->ops.post_send == NULL) {
		usnic_err("Application must enable UDP transport type\n");
		goto out_qp_free;
	}

	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));
	uctx = to_uctx(pd->context);

	if (usnic_qp_attr_parse(attr))
		goto out_qp_free;

	err = usnic_prep_qp_create_cmd(uctx, qp, &cmd);
	if (err)
		goto out_qp_free;

	err = ibv_cmd_create_qp(pd, &qp->ibv_qp, attr, &cmd.ibv_cmd,
			sizeof(cmd), &resp.ibv_resp, sizeof(resp));
	if (err) {
		usnic_err("Failed to create qp with err %d\n", err);
		goto out_undo_qp_create;
	}

	if (usnic_validate_qp_resp(&resp, attr))
		goto out_cmd_qp_destroy;

	if (usnic_discover_vf(&resp, uctx, to_upd(pd), &vf))
		goto out_cmd_qp_destroy;

	usnic_init_qp(qp, pd, attr, vf);

	if (usnic_alloc_qp_and_cq_rings(qp, attr, &resp))
		goto out_undiscover_vf;

	err = usnic_qp_cq_bind_check(qp);
	if (err) {
		usnic_err("Failed to create qp with err %d - CQ depth mis-match\n",
				err);
		goto out_free_qp_rings;
	}

	usnic_qp_attr_set(qp, attr);
	usnic_qp_log(qp);

	return &qp->ibv_qp;

out_free_qp_rings:
	usnic_free_qp_and_cq_rings(qp);
out_undiscover_vf:
	usnic_undiscover_vf(qp);
out_cmd_qp_destroy:
	ibv_cmd_destroy_qp(&qp->ibv_qp);
out_undo_qp_create:
	usnic_undo_qp_create_prep(qp);
out_qp_free:
	free(qp);

	return NULL;
}

struct ibv_qp *usnic_create_qp(struct ibv_pd *pd, struct ibv_qp_init_attr *attr)
{
	struct usnic_qp			*qp;

	qp = calloc(1, sizeof(*qp));
	if (!qp)
		return NULL;

	return usnic_create_qp_allocated(pd, attr, qp);
}

int usnic_destroy_qp(struct ibv_qp *ibqp)
{
	struct usnic_qp *qp = to_uqp(ibqp);
	int err;

	vnic_wq_disable(&qp->wq);
	vnic_rq_disable(&qp->rq);

	err = ibv_cmd_destroy_qp(ibqp);
	if (err)
		return err;

	usnic_free_qp_and_cq_rings(qp);
	usnic_undiscover_vf(qp);
	usnic_undo_qp_create_prep(qp);
	free(qp);
	return 0;
}

int usnic_modify_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
			int attr_mask)
{
	struct ibv_modify_qp	cmd;
	struct usnic_qp		*uqp;
	int			err;

	uqp = to_uqp(qp);

	if (attr->qp_state == IBV_QPS_SQD) {
		usnic_err("qp_state: %d not supported", attr->qp_state);
		err = EINVAL;
		goto err_out;
	} else if (attr->qp_state == IBV_QPS_INIT) {
		usnic_info("Disabling queues\n");
		vnic_wq_disable(&uqp->wq);
		vnic_rq_disable(&uqp->rq);
	}

	attr_mask = IBV_QP_STATE;
	err = ibv_cmd_modify_qp(qp, attr, attr_mask, &cmd, sizeof(cmd));
	if (err) {
		usnic_err("Failed to modify qp with err %d\n", err);
		goto err_out;
	}

	if (attr->qp_state == IBV_QPS_RTR) {
		vnic_rq_enable(&uqp->rq);
		usnic_info("Enabling RQ for qp %d\n", qp->qp_num);
	} else if (attr->qp_state == IBV_QPS_RTS) {
		vnic_wq_enable(&uqp->wq);
		usnic_info("Enabling WQ for qp %d\n", qp->qp_num);
	} else if (attr->qp_state == IBV_QPS_SQD) {
		vnic_wq_disable(&uqp->wq);
		vnic_rq_disable(&uqp->rq);
		usnic_info("Disabling WQ and RQ for %d\n", qp->qp_num);
	}

	return err;
err_out:
	return err;
}

