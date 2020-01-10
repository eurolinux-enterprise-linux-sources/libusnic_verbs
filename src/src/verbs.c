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
#include <fcntl.h>
#include <netinet/in.h>
#include <assert.h>
#include <sys/mman.h>

#include "usnic.h"
#include "usnic_ib_abi.h"
#include "usnic_utils.h"
#include "usnic_transport_priv.h"
#include "usnic_verbs_ext.h"

/*
 * Find firmware version
 */
static int get_firmware_string(struct ibv_context *context, char *fw,
				size_t max_length)
{
	char name[128];
	int fd;
	int n;

	snprintf(name, sizeof(name), "%s/fw_ver", context->device->ibdev_path);

	fd = open(name, O_RDONLY);
	if (fd == -1) {
		perror(name);
		return errno;
	}

	n = read(fd, fw, max_length);
	close(fd);
	if (n < 0) {
		perror("reading fw_ver");
		return errno;
	}
	/* allow for trailing newline */
	if (n > 0 && fw[n - 1] == '\n')
		fw[n - 1] = '\0';
	else
		fw[n] = '\0';

	return 0;
}

int usnic_query_device(struct ibv_context *context,
				struct ibv_device_attr *attr)
{
	struct ibv_query_device cmd;
	uint64_t raw_fw_ver;
	int err;

	memset(attr, 0, sizeof(*attr));

	err = ibv_cmd_query_device(context, attr, &raw_fw_ver, &cmd,
			sizeof(cmd));
	if (err) {
		usnic_err("Failed to query device %s with error %d\n",
				context->device->name, err);
		return err;
	}

	attr->max_qp_wr = MIN(USNIC_MAX_SEND_WR, USNIC_MAX_RECV_WR);
	attr->max_sge = MIN(USNIC_MAX_SEND_SGE, USNIC_MAX_RECV_SGE);
	attr->max_sge_rd = 0;
	attr->max_cqe = USNIC_MAX_CQE;

	err = get_firmware_string(context, attr->fw_ver, sizeof(attr->fw_ver));
	if (err)
		usnic_err("Failed to get firmware version for %s with error %d",
				context->device->name, err);

	return err;
}

int usnic_query_port(struct ibv_context *context, uint8_t port,
			struct ibv_port_attr *attr)
{
	struct ibv_query_port cmd;
	int err;

#if !USNIC_8915
	if (USNIC_EXT_QUERY_PORT_NUM == port) {
		usnic_ext_query_port(attr);
		return 0;
	}
#endif /* !USNIC_8915 */

	err = ibv_cmd_query_port(context, port, attr, &cmd, sizeof(cmd));
	if (err) {
		usnic_err("Failed to query port %u on device %s with err %d\n",
				port, context->device->name, err);
		return err;
	}

	attr->max_msg_sz -= USNIC_HDR_SZ;
	return 0;
}

int usnic_query_qp(struct ibv_qp *ibqp, struct ibv_qp_attr *attr,
			int attr_mask,
			struct ibv_qp_init_attr *init_attr)
{
	struct ibv_query_qp cmd;
	struct ibv_qp_cap cap;
	int err;
	struct usnic_qp *uqp = to_uqp(ibqp);

	memset(attr, 0, sizeof(*attr));
	memset(init_attr, 0, sizeof(*init_attr));

	err = ibv_cmd_query_qp(ibqp, attr, attr_mask, init_attr, &cmd,
					sizeof(cmd));
	if (err) {
		usnic_err("Failed to query qp %u with err %d\n", ibqp->qp_num,
				err);
		return err;
	}

	cap.max_send_wr =
		USNIC_VIC_WQ_DESC_TO_VERBS_WR_CNT(uqp->wq.ring.desc_count);
	cap.max_recv_wr =
		USNIC_VIC_RQ_DESC_TO_VERBS_WR_CNT(uqp->rq.ring.desc_count);
	cap.max_send_sge = USNIC_MAX_SEND_SGE;
	cap.max_recv_sge = USNIC_MAX_RECV_SGE;
	cap.max_inline_data = USNIC_MAX_INLINE_DATA;

	memcpy(&init_attr->cap, &cap, sizeof(cap));
	memcpy(&attr->cap, &cap, sizeof(cap));

	return 0;
}

struct ibv_pd *usnic_alloc_pd(struct ibv_context *context)
{
	struct usnic_alloc_pd		cmd;
	struct usnic_alloc_pd_resp	resp;
	struct usnic_pd			*pd;
	int				err;

	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));

	pd = calloc(1, sizeof(*pd));
	if (!pd) {
		usnic_err("Failed to malloc pd for device %s\n",
				context->device->name);
		return NULL;
	}

	err = ibv_cmd_alloc_pd(context, &pd->ibv_pd, &cmd.ibv_cmd, sizeof(cmd),
				&resp.ibv_resp, sizeof(resp));
	if (err) {
		usnic_err("Failed to alloc pd for device %s with err %d\n",
				context->device->name,
				err);
		free(pd);
		return NULL;
	}

	return &pd->ibv_pd;
}

int usnic_free_pd(struct ibv_pd *pd)
{
	int err;

	err = ibv_cmd_dealloc_pd(pd);
	if (err) {
		usnic_err("Failed to dealloc pd %u with err %d\n",
				pd->handle, err);
		return err;
	}

	free(to_upd(pd));
	return 0;
}

struct ibv_mr *usnic_reg_mr(struct ibv_pd *pd, void *addr, size_t length,
				int access)
{
	struct ibv_mr			*mr;
	struct usnic_reg_mr		 cmd;
	struct usnic_reg_mr_resp	 resp;
	int				 err;

	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));

	mr = calloc(1, sizeof(*mr));
	if (!mr) {
		usnic_err("Failed to allocate memory for mr starting at 0x%p for pd %u\n",
				addr, pd->handle);
		return NULL;
	}

	err = ibv_cmd_reg_mr(pd, addr, length, (uintptr_t) addr,
				access, mr, &cmd.ibv_cmd, sizeof(cmd),
				&resp.ibv_resp, sizeof(resp));
	if (err) {
		usnic_err("Failed to reg mr starting at 0x%p for pd %u with err %d\n",
				addr, pd->handle, err);
		free(mr);
		return NULL;
	}

	return mr;
}

int usnic_dereg_mr(struct ibv_mr *mr)
{
	int err;

	err = ibv_cmd_dereg_mr(mr);
	if (err) {
		usnic_err("Failed to dereg mr %u with err %d\n",
				mr->handle, err);
		return err;
	}

	free(mr);
	return 0;
}

struct ibv_cq *usnic_create_cq(struct ibv_context *context, int cqe,
				struct ibv_comp_channel *channel,
				int comp_vector)
{
	struct usnic_create_cq		cmd;
	struct usnic_create_cq_resp	resp;
	struct usnic_cq			*cq;
	int				err;
	int				cqe_actual;

	if (context->ops.poll_cq == NULL) {
		usnic_err("Application must enable UDP transport type\n");
		return NULL;
	}

	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));

	if (cqe < 0 || cqe > USNIC_MAX_CQE) {
		usnic_err("Failed to alloc cq for device %s - Too many cqe requested %d\n",
				context->device->name, cqe);
		return NULL;
	}

	cqe_actual = usnic_get_min_safe_cqe(cqe + 1);
	if (cqe_actual > VIC_MAX_CQ_DESC_CNT) {
		usnic_err("Failed to alloc cq for device %s - Too many actual cqes %d for asked cqes %d\n",
				context->device->name, cqe_actual, cqe);
		return NULL;
	}

	cq = calloc(1, sizeof(*cq));
	if (!cq) {
		usnic_err("Failed to alloc cq for device %s - Out of memory\n",
				context->device->name);
		return NULL;
	}

	pthread_spin_init(&cq->lock, PTHREAD_PROCESS_PRIVATE);
	err = ibv_cmd_create_cq(context, cqe, channel, comp_vector, &cq->ibv_cq,
				&cmd.ibv_cmd, sizeof(cmd),
				&resp.ibv_resp, sizeof(resp));

	if (err) {
		usnic_err("Failed to create CQ for device %s with error %d\n",
				context->device->name, err);
		goto err_create_cq;
	}

	cq->ibv_cq.cqe = cqe_actual - 1;
	usnic_info("Created CQ %u CQE Asked: %u CQE Given: %u\n",
			cq->ibv_cq.handle,
			cqe,
			cq->ibv_cq.cqe);

	return &cq->ibv_cq;

err_create_cq:
	free(cq);
	return NULL;
}

int usnic_destroy_cq(struct ibv_cq *cq)
{
	int err;

	err = ibv_cmd_destroy_cq(cq);
	if (err) {
		usnic_err("Failed to destroy CQ %u", cq->handle);
		return err;
	}
	usnic_info("Destroying CQ %u", cq->handle);
	free(to_ucq(cq));
	return 0;
}

static inline int usnic_wq_ring_overflow(struct usnic_qp *uqp, unsigned nreq)
{
	struct usnic_cq *ucq = to_ucq(uqp->ibv_qp.send_cq);
	unsigned desc_avail;

	/*
	 * First try to read desc_avail w/o acquiring the lock
	 * since poll_cq only increments desc_avail and
	 * it is only place besides post_send that desc_avail
	 * is modified - UM
	 */
	if (vnic_wq_desc_avail(&uqp->wq) >= nreq)
		return 0;

	pthread_spin_lock(&ucq->lock);
	desc_avail = vnic_wq_desc_avail(&uqp->wq);
	pthread_spin_unlock(&ucq->lock);
	return desc_avail < nreq;
}

static inline int usnic_rq_ring_overflow(struct usnic_qp *uqp, unsigned nreq)
{
	unsigned desc_avail;
	struct usnic_cq *ucq = to_ucq(uqp->ibv_qp.recv_cq);

	/* See comments in usnic_wq_ring_overflow */
	if (vnic_rq_desc_avail(&uqp->rq) >= nreq)
		return 0;

	pthread_spin_lock(&ucq->lock);
	desc_avail = vnic_rq_desc_avail(&uqp->rq);
	pthread_spin_unlock(&ucq->lock);
	return desc_avail < nreq;
}

static inline void
usnic_append_payload_to_hdr_offset(void *hdr, void *payload,
					int len, uint16_t offset)
{
	memcpy((char *)hdr + offset, payload, len);
}

static inline
int usnic_check_wq_wr(struct usnic_qp *uqp, struct ibv_send_wr *wr,
			struct ibv_send_wr **bad_wr)
{
	if (unlikely(usnic_wq_ring_overflow(uqp, 2))) {
		usnic_err("Insufficent descs avail\n");
		*bad_wr = wr;
		return ENOMEM;
	}
	if (unlikely(wr->num_sge > 1 && !(wr->send_flags & IBV_SEND_INLINE))) {
		usnic_err("Too many sqe\n");
		*bad_wr = wr;
		return EINVAL;
	}
	return 0;
}

static inline
int usnic_prepare_inline_frame(struct ibv_send_wr *wr,
				struct ibv_send_wr **bad_wr, void *hdr,
				uint16_t *p_payload_size,
				uint16_t *p_hdr_frag_size)
{
	int i;
	uint64_t payload_addr;
	uint16_t segment_size;
	uint16_t payload_size = 0;
	uint16_t hdr_frag_size = *p_hdr_frag_size;

	for (i = 0; i < wr->num_sge; i++) {
		payload_addr = wr->sg_list[i].addr;
		segment_size = wr->sg_list[i].length;
		if (unlikely(segment_size + hdr_frag_size >
				VIC_MAX_PREFETCH_SIZE)) {
			*bad_wr = wr;
			return E2BIG;
		}
		usnic_append_payload_to_hdr_offset(hdr,
					(void *)payload_addr,
					segment_size,
					hdr_frag_size);
		payload_size += segment_size;
		hdr_frag_size += segment_size;
	}

	*p_payload_size = payload_size;
	*p_hdr_frag_size = hdr_frag_size;
	return 0;
}

static inline
void usnic_wq_post(struct vnic_wq *wq, struct usnic_cq *ucq,
			struct ibv_send_wr *wr, struct wq_enet_desc *desc,
			uint64_t addr, uint16_t size, uint8_t sop, uint8_t eop,
			uint8_t cq_entry, uint8_t compressed_send,
			uint8_t desc_skip_cnt)
{
	uint16_t mss = usnic_get_wq_desc_mss();

	wq_enet_desc_enc(desc, (u64)addr, size, mss, 0,
				WQ_ENET_OFFLOAD_MODE_CSUM, eop, cq_entry,
				0, 0, 0, 0);

	/*
	 * This lock is held for longer than needed.  Only critical
	 * section that this lock needs to protect is modification
	 * to desc_avail. It could be optimized.
	 * Same goes for vnic_rq_post. -UM
	 */
	pthread_spin_lock(&ucq->lock);
	vnic_wq_post(wq, wr, (dma_addr_t)addr, size, sop, eop,
		desc_skip_cnt, cq_entry, compressed_send, wr->wr_id);
	pthread_spin_unlock(&ucq->lock);
}

int usnic_post_send(struct ibv_qp *qp, struct ibv_send_wr *wr,
			struct ibv_send_wr **bad_wr)
{
	struct vnic_wq *wq;
	struct usnic_qp *uqp;
	struct usnic_cq *ucq;
	int err;

	uqp = to_uqp(qp);
	ucq = to_ucq(qp->send_cq);
	pthread_spin_lock(&uqp->wq_lock);
	wq = (&to_uqp(qp)->wq);

	for (; wr; wr = wr->next) {
		err = usnic_check_wq_wr(uqp, wr, bad_wr);
		if (unlikely(err))
			goto err_out;

		void *hdr;
		u_int8_t eop = 0;
		u_int8_t compressed_send = 0;
		u_int8_t cq_entry = (uqp->sq_sig_all ||
					(wr->send_flags & IBV_SEND_SIGNALED)) ?
					1 : 0;
		uint64_t payload_addr = likely((wr->num_sge)) ?
					(unsigned long)wr->sg_list[0].addr : 0;
		uint16_t payload_size = likely((wr->num_sge)) ?
						wr->sg_list[0].length : 0;
		uint16_t hdr_frag_size = USNIC_HDR_SZ;

		/* Post Pkt Hdr Fragment*/
		hdr = usnic_get_shdr_buf(uqp, wq->to_use->index);
		if (unlikely(!wr->num_sge || payload_size == 0)) {
			eop = 1;
			compressed_send = 1;
		} else if (likely((wr->send_flags & IBV_SEND_INLINE))) {
			err = usnic_prepare_inline_frame(wr, bad_wr, hdr,
						&payload_size, &hdr_frag_size);
			if (unlikely(err))
				goto err_out;
			eop = 1;
			compressed_send = 1;
		}

		/* fill in the header */
		hdr = usnic_write_hdr_at_send(hdr, wr, payload_size);

		usnic_wq_post(wq, ucq, wr, vnic_wq_next_desc(wq), (uint64_t)hdr,
				hdr_frag_size, 1, eop, eop & cq_entry,
				compressed_send, eop + 1);

		if (likely(eop))
			continue;

		/* Post Payload Frag */
		usnic_wq_post(wq, ucq, wr, vnic_wq_next_desc(wq), payload_addr,
				payload_size, 0, 1, cq_entry, 0, 1);

	}
	pthread_spin_unlock(&uqp->wq_lock);
	return 0;

err_out:
	pthread_spin_unlock(&uqp->wq_lock);
	return err;
}

int usnic_post_recv(struct ibv_qp *qp, struct ibv_recv_wr *wr,
			struct ibv_recv_wr **bad_wr)
{
	struct usnic_qp *uqp;
	struct usnic_cq *ucq;
	struct vnic_rq *rq;
	struct rq_enet_desc *desc;
	int err;

	uqp = to_uqp(qp);
	ucq = to_ucq(qp->recv_cq);

	pthread_spin_lock(&uqp->rq_lock);
	rq = (&to_uqp(qp)->rq);
	for (; wr; wr = wr->next) {
		if (unlikely(wr->num_sge > 1)) {
			err = EINVAL;
			*bad_wr = wr;
			goto err_out;
		}
		if (unlikely(0 == wr->num_sge)) {
			err = EINVAL;
			*bad_wr = wr;
			goto err_out;
		}
		if (unlikely(usnic_rq_ring_overflow(uqp, 1))) {
			err = ENOMEM;
			*bad_wr = wr;
			goto err_out;
		}
		if (unlikely(wr->num_sge == 1 &&
			wr->sg_list[0].length < USNIC_MIN_RX_BUF)) {
			err = EINVAL;
			*bad_wr = wr;
			goto err_out;
		}
		u_int64_t bus_addr = (unsigned long)wr->sg_list[0].addr;
		u_int16_t size = wr->sg_list[0].length;
		u_int8_t type = RQ_ENET_TYPE_ONLY_SOP;

		desc = vnic_rq_next_desc(rq);
		rq_enet_desc_enc(desc, bus_addr, type, size);
		pthread_spin_lock(&ucq->lock);
		vnic_rq_post(rq, wr, 0, bus_addr, size, wr->wr_id);
		pthread_spin_unlock(&ucq->lock);
	}
	pthread_spin_unlock(&uqp->rq_lock);
	return 0;

err_out:
	pthread_spin_unlock(&uqp->rq_lock);
	return err;
}

