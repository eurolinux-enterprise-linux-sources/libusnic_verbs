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

#ifndef USNIC_VERBS_H
#define USNIC_VERBS_H

#include "config.h"

#include <stddef.h>

#include <infiniband/arch.h>
#include <infiniband/driver.h>

int usnic_query_device(struct ibv_context *context,
			struct ibv_device_attr *attr);
int usnic_query_port(struct ibv_context *context, uint8_t port,
			struct ibv_port_attr *attr);
struct ibv_pd *usnic_alloc_pd(struct ibv_context *context);
int usnic_free_pd(struct ibv_pd *pd);
struct ibv_mr *usnic_reg_mr(struct ibv_pd *pd, void *addr,
				size_t length, int access);
int usnic_dereg_mr(struct ibv_mr *mr);

struct ibv_cq *usnic_create_cq(struct ibv_context *context, int cqe,
				struct ibv_comp_channel *channel,
				int comp_vector);
int usnic_destroy_cq(struct ibv_cq *cq);
int usnic_poll_cq(struct ibv_cq *cq, int ne, struct ibv_wc *wc);
struct ibv_qp *usnic_create_qp(struct ibv_pd *pd,
					struct ibv_qp_init_attr *attr);
int usnic_query_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
			int attr_mask,
			struct ibv_qp_init_attr *init_attr);
int usnic_modify_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
			int attr_mask);
int usnic_destroy_qp(struct ibv_qp *qp);
int usnic_post_send(struct ibv_qp *ibqp, struct ibv_send_wr *wr,
			struct ibv_send_wr **bad_wr);
int usnic_post_recv(struct ibv_qp *ibqp, struct ibv_recv_wr *wr,
			struct ibv_recv_wr **bad_wr);
struct ibv_ah *usnic_create_ah(struct ibv_pd *pd, struct ibv_ah_attr *attr);
int usnic_destroy_ah(struct ibv_ah *ah);

struct usnic_qp;
struct ibv_qp *usnic_create_qp_allocated(struct ibv_pd *pd, struct ibv_qp_init_attr *attr, struct usnic_qp *ap);


struct ibv_qp *usnic_create_qp_stats(struct ibv_pd *pd,
					struct ibv_qp_init_attr *attr);
int usnic_destroy_qp_stats(struct ibv_qp *qp);
int usnic_poll_cq_stats(struct ibv_cq *cq, int ne, struct ibv_wc *wc);
int usnic_post_send_stats(struct ibv_qp *ibqp, struct ibv_send_wr *wr,
			struct ibv_send_wr **bad_wr);
#endif /* USNIC_VERBS_H */
