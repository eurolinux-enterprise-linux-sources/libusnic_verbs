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
#include "usnic_verbs.h"
#include "verbs_stats.h"

int usnic_post_send_stats(struct ibv_qp *qp, struct ibv_send_wr *wr,
			struct ibv_send_wr **bad_wr)
{
	struct usnic_qp_stats *sqp;
	struct ibv_send_wr *cwr;
	unsigned pkts;
       	unsigned bytes;
	int ret;
	int i;

	pkts = 0;
	bytes = 0;

	for (cwr = wr; cwr != NULL; cwr = cwr->next) {
		++pkts;
		bytes += (USNIC_HDR_SZ - sizeof(struct ether_header));
		for (i = 0; i < cwr->num_sge; ++i) {
			bytes += cwr->sg_list[i].length;
		}
	}

	ret = usnic_post_send(qp, wr, bad_wr);

	/* subtract out the failed sends */
	if (ret != 0) {
		for (cwr = *bad_wr; cwr != NULL; cwr = cwr->next) {
			--pkts;
			bytes -= (USNIC_HDR_SZ - sizeof(struct ether_header));
			for (i = 0; i < cwr->num_sge; ++i) {
				bytes -= cwr->sg_list[i].length;
			}
		}
	}

	sqp = to_sqp(qp);
	pthread_spin_lock(&sqp->stats_lock);
	sqp->tx_bytes += bytes;
	sqp->tx_pkts += pkts;
	pthread_spin_unlock(&sqp->stats_lock);

	return ret;
}

int usnic_poll_cq_stats(struct ibv_cq *ibcq, int ne, struct ibv_wc *wc)
{
	struct usnic_cq *cq = to_ucq(ibcq);
	struct usnic_qp_stats *sqp;
	unsigned pkts;
	unsigned bytes;
	int ret;
	int i;

	ret = usnic_poll_cq(ibcq, ne, wc);

	pkts = 0;
	bytes = 0;
	for (i = 0; i < ret; ++i) {
		if (wc[i].opcode & IBV_WC_RECV) {
			++pkts;
			bytes += wc[i].byte_len - sizeof(struct ether_header);
		}
	}

	sqp = (struct usnic_qp_stats *)cq->qp;
	pthread_spin_lock(&sqp->stats_lock);
	sqp->rx_bytes += bytes;
	sqp->rx_pkts += pkts;
	pthread_spin_unlock(&sqp->stats_lock);

	return ret;
}

struct ibv_qp *usnic_create_qp_stats(struct ibv_pd *pd, struct ibv_qp_init_attr *attr)
{
	struct usnic_qp_stats *sqp;
	struct usnic_qp *uqp;

	sqp = calloc(1, sizeof(*sqp));
	if (!sqp)
		return NULL;

	pthread_spin_init(&sqp->stats_lock, PTHREAD_PROCESS_PRIVATE);

	uqp = (struct usnic_qp *)sqp;

	return usnic_create_qp_allocated(pd, attr, uqp);
}

int usnic_destroy_qp_stats(struct ibv_qp *ibqp)
{
	struct usnic_qp *uqp = to_uqp(ibqp);
	struct usnic_qp_stats *sqp;
	char host[32];
	int rc;
	char *p;

	sqp = (struct usnic_qp_stats *)uqp;

	/* prefix with hostname and device */
	rc = gethostname(host, sizeof(host));
	if (rc != 0)
		strcpy(host, "<unknown>");
	p = strchr(host, '.');
	if (p != NULL)
		*p = '\0';
	printf("%s %s QP:%5d rxpkts:%12lu rxipbytes:%12lu  "
			"txpkts:%12lu txipbytes:%12lu\n",
			host,
			ibqp->context->device->name,
			ibqp->qp_num,
			sqp->rx_pkts,
			sqp->rx_bytes,
			sqp->tx_pkts,
			sqp->tx_bytes);

	/* now do the real destroy */
	return usnic_destroy_qp(ibqp);
}
