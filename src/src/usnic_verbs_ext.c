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

#include <stdio.h>

#include "usnic.h"
#include "usnic_transport_udp.h"
#include "usnic_verbs.h"
#include "usnic_verbs_ext.h"

static int usnic_ext_enable_udp(struct ibv_context *ibv_ctx)
{
	struct usnic_device *udev = to_udev(ibv_ctx->device);
	struct usnic_context *uctx = to_uctx(ibv_ctx);

	if (usnic_mark_dev_transport_udp(udev)) {
		if (getenv("USNIC_QP_STATS") == NULL) {
			uctx->ibv_ctx.ops.post_send = usnic_post_send;
			uctx->ibv_ctx.ops.poll_cq = usnic_poll_cq;
		} else {
			uctx->ibv_ctx.ops.post_send = usnic_post_send_stats;
			uctx->ibv_ctx.ops.poll_cq = usnic_poll_cq_stats;
		}
		uctx->ibv_ctx.ops.post_recv = usnic_post_recv;
	}
	return 0;
}

static size_t usnic_ext_get_ud_header_len(struct ibv_context *UNUSED(ibv_ctx),
						uint8_t UNUSED(port_num))
{
	return USNIC_HDR_SZ;
}

static void *usnic_ext_entry(const char *name)
{
	if (strcmp(name, "enable_udp") == 0)
		return (void *)usnic_ext_enable_udp;
	else if (strcmp(name, "get_ud_header_len") == 0)
		return (void *)usnic_ext_get_ud_header_len;

	return NULL;
}

void usnic_ext_query_port(struct ibv_port_attr *attr)
{
	struct usnic_query_port_table *qpt =
		(struct usnic_query_port_table *) attr;
	qpt->version = USNIC_EXT_ABI_VERSION;
	qpt->magic = USNIC_EXT_ABI_MAGIC;
	qpt->entry_fn = usnic_ext_entry;
}
