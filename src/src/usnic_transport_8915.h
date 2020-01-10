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

#ifndef USNIC_TRANSPORT_8915_H
#define USNIC_TRANSPORT_8915_H

#include <netinet/in.h>
#include <infiniband/arch.h>

#include "usnic.h"
#include "usnic_common_pkt_hdr.h"

#define USNIC_FLAGS_IMM_DATA_BIT	1

union usnic_hdr {
	uint8_t				raw[40];
	struct {
		uint8_t			dmac[6];
		uint8_t			smac[6];
		uint16_t		ethertype;
		uint8_t			proto_version;
		uint32_t		dqpn;
		uint32_t		sqpn;
		uint16_t		payload_len;
		uint8_t			flags;
		uint32_t		imm_data;
	} __attribute__((__packed__)) l2;
} __attribute__((__packed__));

#define USNIC_HDR_SZ  (sizeof(union usnic_hdr))

struct usnic_ah {
	struct ibv_ah			ibv_ah;
	struct ibv_pd			*pd;
	union ibv_gid			dgid;
	uint8_t				dmac[6];
};

union usnic_gid {
	uint8_t				raw[16];
	struct {
		uint64_t		subnet_prefix;
		uint64_t		interface_id;
	} __attribute__((__packed__)) global;
} __attribute__((__packed__));

struct grh_hdr {
	uint32_t			version:4;
	uint32_t			qos:8;
	uint32_t			flow_label:20;
	uint16_t			payload_len;
	uint8_t				next_hdr;
	uint8_t				hop_limit;
	union usnic_gid			src_gid;
	union usnic_gid			dst_gid;
} __attribute__((__packed__));

static inline
void usnic_assign_ctx_op(struct usnic_context *uctx,
				struct ibv_context_ops *usnic_ctx_ops)
{
	memcpy(&uctx->ibv_ctx.ops, usnic_ctx_ops, sizeof(*usnic_ctx_ops));
}

static inline struct usnic_ah *to_uah(struct ibv_ah *ibah)
{
	return usnic_container_of(ah, ah);
}

static inline void
usnic_print_frame(const char *label, const char *ptr,
			unsigned int len, unsigned long counter)
{
	union usnic_hdr *hdr = (union usnic_hdr *) ptr;
	uint8_t *dst_mac = hdr->l2.dmac;
	uint8_t *src_mac = hdr->l2.smac;

	fprintf(stdout, "%s: %lu"
		"|%02x:%02x:%02x:%02x:%02x:%02x"
		"|%02x:%02x:%02x:%02x:%02x:%02x"
		"|0x%x|0x%02x|0x%08x|0x%08x"
		"|0x%x|0x%08x|...SIZE(%u)\n",
		label, counter,
		dst_mac[0], dst_mac[1], dst_mac[2], dst_mac[3],
		dst_mac[4], dst_mac[5], src_mac[0], src_mac[1],
		src_mac[2], src_mac[3], src_mac[4], src_mac[5],
		ntohs(hdr->l2.ethertype),
		hdr->l2.proto_version,
		ntohl(hdr->l2.dqpn),
		ntohl(hdr->l2.sqpn),
		hdr->l2.flags,
		ntohl(hdr->l2.imm_data),
		len);
}

static inline uint16_t usnic_get_wq_desc_mss(void)
{
	uint16_t ip_csum = 0;
	uint16_t tcpudp_csm = 0;
	uint16_t iptcpudp_rewrite = 0;

	return ip_csum + tcpudp_csm + iptcpudp_rewrite;
}

static inline void
usnic_write_hdr_at_ring_init(void *hdr, struct usnic_qp *qp, union ibv_gid *gid)
{
	uint8_t mac[6];
	union usnic_hdr *l2_hdr = (union usnic_hdr *)hdr;

	usnic_gid_to_mac(gid, mac);

	memcpy(l2_hdr->l2.smac, mac, sizeof(mac));
	l2_hdr->l2.ethertype = htons(USNIC_ROCE_ETHERTYPE);
	l2_hdr->l2.proto_version =
		(USNIC_ROCE_GRH_VER << USNIC_ROCE_GRH_VER_SHIFT) |
		USNIC_PROTO_VER;
	l2_hdr->l2.sqpn = htonl(qp->ibv_qp.qp_num);
}

static inline void *
usnic_write_hdr_at_send(void *header, struct ibv_send_wr *wr, int payload_len)
{
	union usnic_hdr *hdr = (union usnic_hdr *)header;
	struct usnic_ah	*uah = to_uah(wr->wr.ud.ah);

	memcpy(hdr->l2.dmac, &uah->dmac[0], ETH_ALEN);
	hdr->l2.dqpn = htonl(wr->wr.ud.remote_qpn);
	if (wr->opcode == IBV_WR_SEND_WITH_IMM) {
		hdr->l2.flags = USNIC_FLAGS_IMM_DATA_BIT;
		hdr->l2.imm_data = htonl(wr->imm_data);
	} else {
		hdr->l2.flags = 0;
	}
	hdr->l2.payload_len = htons(payload_len);
#if USNIC_LOG_LVL >= USNIC_LOG_LVL_VERBOSE
	static unsigned long counter;
	usnic_print_frame("Tx", (const char *)hdr,
				sizeof(union usnic_hdr), counter++);
#endif
	return hdr;
}

void print_grh_hdr(const char *label, const char *ptr, unsigned int len);

#endif /* USNIC_TRANSPORT_8915_H */
