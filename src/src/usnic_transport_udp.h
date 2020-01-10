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

#ifndef USNIC_TRANSPORT_UDP_H
#define USNIC_TRANSPORT_UDP_H

#include <netinet/ip.h>
#include <netinet/udp.h>
#include <net/ethernet.h>

#include "usnic.h"

#ifndef IP_DF
# define IP_DF 0x4000
#endif

union usnic_hdr {
	uint8_t				raw[42];
	struct {
		struct ether_header	eth;
		struct iphdr		ip;
		struct udphdr		udp;
	} __attribute__((__packed__)) hdr;
} __attribute__((__packed__));

#define USNIC_HDR_SZ  (sizeof(union usnic_hdr))

struct usnic_ah {
	struct ibv_ah	ibv_ah;
	struct ibv_pd	*pd;
	union ibv_gid	dgid;
	uint32_t	dip;
	uint8_t		dmac[6];
};

static inline bool usnic_is_dev_transport_udp(struct usnic_device *udev)
{
	return __sync_bool_compare_and_swap(&udev->transport,
		USNIC_TRANSPORT_IPV4_UDP, USNIC_TRANSPORT_IPV4_UDP);
}

static inline bool usnic_mark_dev_transport_udp(struct usnic_device *udev)
{
	return __sync_bool_compare_and_swap(&udev->transport,
		USNIC_TRANSPORT_UNKNOWN, USNIC_TRANSPORT_IPV4_UDP);
}

static inline
void usnic_assign_ctx_op(struct usnic_context *uctx,
				struct ibv_context_ops *usnic_ctx_ops)
{
	memcpy(&uctx->ibv_ctx.ops, usnic_ctx_ops, sizeof(*usnic_ctx_ops));
	/*
	 * The library can only supports either UDP transport format
	 * or usnic_8915 because it is easier to fail ibv_ud_pingpong gracefully
	 * by setting some callbacks in context stucture as NULL and
	 * let ibv_create_qp fail.
	 */
        struct usnic_device *udev = to_udev(uctx->ibv_ctx.device);

	/*
	 *if a context is already allocated for the same device and UDP
	 * transport is enabled, then do not set function pointers as NULL
	 */
        if (!usnic_is_dev_transport_udp(udev)) {
		uctx->ibv_ctx.ops.post_send = NULL;
		uctx->ibv_ctx.ops.post_recv = NULL;
		uctx->ibv_ctx.ops.poll_cq = NULL;
	}
}

static inline struct usnic_ah *to_uah(struct ibv_ah *ibah)
{
	return usnic_container_of(ah, ah);
}

static inline uint16_t usnic_get_wq_desc_mss(void)
{
	uint16_t ip_csum = 1;
	uint16_t tcpudp_csm = 2;
	uint16_t iptcpudp_rewrite = 4; /* hardware overwrite ip/udp checksum */

	return ip_csum + tcpudp_csm + iptcpudp_rewrite;
}

static inline void
usnic_write_hdr_at_ring_init(void *hdr, struct usnic_qp *qp, union ibv_gid *gid)
{
	union usnic_hdr *udp_hdr = (union usnic_hdr *)hdr;
	uint8_t mac[6];

	usnic_gid_to_mac(gid, mac);

	udp_hdr->hdr.eth.ether_type = htons(0x0800);
	memcpy(udp_hdr->hdr.eth.ether_shost, mac, sizeof(mac));
	udp_hdr->hdr.ip.ihl = 5;	    /* no options */
	udp_hdr->hdr.ip.version = 4;
	udp_hdr->hdr.ip.tos = 0;
	/* we don't reassemble frags, so don't allow them to be created */
	udp_hdr->hdr.ip.frag_off = htons(IP_DF);
	udp_hdr->hdr.ip.ttl = 64;
	udp_hdr->hdr.ip.protocol = IPPROTO_UDP;
	usnic_udp_gid_to_ipaddr(gid, &udp_hdr->hdr.ip.saddr);
	udp_hdr->hdr.udp.source = htons(qp->ibv_qp.qp_num);
}

static inline void*
usnic_write_hdr_at_send(void *header, struct ibv_send_wr *wr, int payload_len)
{
	union usnic_hdr *hdr = (union usnic_hdr *)header;
	struct usnic_ah *uah = to_uah(wr->wr.ud.ah);

	memcpy(hdr->hdr.eth.ether_dhost, &uah->dmac[0], 6);
	hdr->hdr.ip.daddr = uah->dip;
	hdr->hdr.ip.tot_len = htons(sizeof(union usnic_hdr) -
		       sizeof(struct ether_header) + payload_len);
	hdr->hdr.udp.dest = htons(wr->wr.ud.remote_qpn);
	hdr->hdr.udp.len = htons(sizeof(struct udphdr) + payload_len);
	return hdr;
}

#endif /* USNIC_TRANSPORT_UDP_H */
