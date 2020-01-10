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
#include <sys/ioctl.h>
#include <sys/eventfd.h>

#include "usnic_verbs.h"
#include "usnic.h"
#include "usnic_ib_abi.h"
#include "usnic_utils.h"
#include "usnic_transport_priv.h"

#include "kcompat.h"
#include "linux_types.h"
#include "wq_enet_desc.h"
#include "rq_enet_desc.h"
#include "cq_enet_desc.h"

#include "vnic_resource.h"
#include "vnic_devcmd.h"
#include "vnic_dev.h"
#include "vnic_stats.h"
#include "vnic_nic.h"

struct ibv_context_ops usnic_ctx_ops = {
	.query_device = usnic_query_device,
	.query_port = usnic_query_port,
	.query_qp = usnic_query_qp,
	.alloc_pd = usnic_alloc_pd,
	.dealloc_pd = usnic_free_pd,
	.reg_mr = usnic_reg_mr,
	.dereg_mr = usnic_dereg_mr,
	.create_cq = usnic_create_cq,
	.destroy_cq = usnic_destroy_cq,
	.create_qp = usnic_create_qp,
	.modify_qp = usnic_modify_qp,
	.destroy_qp = usnic_destroy_qp,
	.create_ah = usnic_create_ah,
	.destroy_ah = usnic_destroy_ah,
	.post_send = usnic_post_send,
	.post_recv = usnic_post_recv,
	.poll_cq = usnic_poll_cq,
};

struct ibv_context_ops usnic_ctx_stats_ops = {
	.query_device = usnic_query_device,
	.query_port = usnic_query_port,
	.query_qp = usnic_query_qp,
	.alloc_pd = usnic_alloc_pd,
	.dealloc_pd = usnic_free_pd,
	.reg_mr = usnic_reg_mr,
	.dereg_mr = usnic_dereg_mr,
	.create_cq = usnic_create_cq,
	.destroy_cq = usnic_destroy_cq,
	.create_qp = usnic_create_qp_stats,
	.modify_qp = usnic_modify_qp,
	.destroy_qp = usnic_destroy_qp_stats,
	.create_ah = usnic_create_ah,
	.destroy_ah = usnic_destroy_ah,
	.post_send = usnic_post_send_stats,
	.post_recv = usnic_post_recv,
	.poll_cq = usnic_poll_cq_stats,
};

struct ibv_context *usnic_alloc_context(struct ibv_device *ibdev,
					int cmd_fd)
{
	struct usnic_context		*uctx;
	struct usnic_get_context	cmd;
	struct usnic_get_context_resp	resp;
	int err;

	uctx = calloc(1, sizeof(*uctx));
	if (!uctx)
		return NULL;

	err = pthread_mutex_init(&uctx->vf_lst_lock, NULL);
	if (err) {
		usnic_err(
			"Failed to init usnic context mutext for PF %s with err %d\n",
			ibdev->name, err);
		goto err_out_free_context;
	}

	memset(&cmd, 0, sizeof(cmd));
	memset(&resp, 0, sizeof(resp));

	uctx->ibv_ctx.device = ibdev;
	uctx->ibv_ctx.cmd_fd = cmd_fd;
	if (getenv("USNIC_QP_STATS") == NULL) {
		usnic_assign_ctx_op(uctx, &usnic_ctx_ops);
	} else {
		usnic_assign_ctx_op(uctx, &usnic_ctx_stats_ops);
	}

	if (ibv_cmd_get_context(&uctx->ibv_ctx, &cmd.ibv_cmd, sizeof(cmd),
		&resp.ibv_resp, sizeof(resp))) {
		usnic_err("Couldn't get context from %s\n",
				ibdev->name);
		goto err_out_free_context;
	}

	err = pthread_mutex_init(&uctx->ibv_ctx.mutex, NULL);
	if (err) {
		usnic_err("Failed to init context mutex for PF %s with err %d\n"
				, ibdev->name, err);
		goto err_out_free_context;
	}
	uctx->ibv_ctx.abi_compat = NULL;

	usnic_info("Allocated context for PF %s with fd %d\n",
			ibdev->name, cmd_fd);
	return &uctx->ibv_ctx;

err_out_free_context:
	free(uctx);

	return NULL;
}

void usnic_free_context(struct ibv_context *ibctx)
{
	struct usnic_context *uctx = to_uctx(ibctx);

	usnic_info("Freeing context for PF %s with fd %d", ibctx->device->name,
		ibctx->cmd_fd);
	free(uctx);
}

