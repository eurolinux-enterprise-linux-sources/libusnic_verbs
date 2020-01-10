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
#include <errno.h>
#include <time.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <ifaddrs.h>

#include "usnic.h"
#include "usnic_ib_abi.h"
#include "usnic_utils.h"
#include "usnic_verbs.h"
#include "usnic_ip_utils.h"
#include "usnic_transport_priv.h"

static int usnic_get_udev_ifinfo(struct usnic_context *uctx, uint32_t if_ipaddr)
{
	struct usnic_device *udev;
	struct ifaddrs *ifaddr, *ifa;
	int if_index;
	int err = EADDRNOTAVAIL;

	udev = to_udev(uctx->ibv_ctx.device);

	/*
	 * usnic PF interface index and name should not be changed during
	 * application life, it's safe to cache the information
	 */
	if (udev->if_index && udev->ifname[0] != '\0')
		return 0;

	if (getifaddrs(&ifaddr) == -1) {
		usnic_perr("getifaddr failed");
		return -1;
	}

	for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
		if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
			if (((struct sockaddr_in *)ifa->ifa_addr)
				->sin_addr.s_addr == if_ipaddr) {
				if_index = if_nametoindex(ifa->ifa_name);
				if (!if_index) {
					usnic_perr(
						"if_nametoindex failed, if name: %s",
						ifa->ifa_name);
					err = -1;
					break;
				}
				udev->if_index = if_index;
				strncpy(udev->ifname, ifa->ifa_name,
						sizeof(udev->ifname) - 1);
				err = 0;
				break;
			}
		}
	}

	freeifaddrs(ifaddr);
	return err;
}

static int usnic_get_local_ip(struct ibv_context *context, uint32_t *ip_addr)
{
	int err;
	union ibv_gid my_gid;

	err = ibv_query_gid(context, USNIC_DEFAULT_PORT,
				USNIC_DEFAULT_GID_IDX, &my_gid);
	if (err) {
		usnic_err("Failed to query gid for ib device: %s\n",
				context->device->name);
		return -1;
	}
	usnic_udp_gid_to_ipaddr(&my_gid, ip_addr);

	return 0;
}

static int usnic_alloc_qp_udp_socket(struct usnic_context *uctx,
					struct usnic_qp *qp)
{
	struct sockaddr_in sin;
	uint32_t local_ip;
	int sockfd;
	int err;

	err = usnic_get_local_ip(&uctx->ibv_ctx, &local_ip);
	if (err)
		return -1;

	err = usnic_get_udev_ifinfo(uctx, local_ip);
	if (err) {
		if (err == EADDRNOTAVAIL) {
			char buf[INET_ADDRSTRLEN];
			inet_ntop(AF_INET, &local_ip, buf, sizeof(buf));
			usnic_err("Failed to find if index and name for %s\n",
					buf);
		}
		return -1;
	}

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd == -1) {
		usnic_perr("socket() failed");
		return -1;
	}

	memset(&sin, 0, sizeof(sin));
	sin.sin_family = AF_INET;
	sin.sin_port = 0;
	sin.sin_addr.s_addr = local_ip;
	err = bind(sockfd, (struct sockaddr *)&sin, sizeof(sin));
	if (err == -1) {
		char buf[INET_ADDRSTRLEN];
		inet_ntop(AF_INET, &local_ip, buf, sizeof(buf));
		usnic_perr("bind() failed, local ip: %s", buf);
		goto out_close_sock;
	}
	qp->qp_socket = sockfd;
	return 0;

out_close_sock:
	close(sockfd);
	return err;
}

int usnic_prep_qp_create_cmd(struct usnic_context *uctx, struct usnic_qp *qp,
				struct usnic_create_qp *cmd)
{
	int err;

	err = usnic_alloc_qp_udp_socket(uctx, qp);
	if (err)
		return err;
	cmd->usnic_cmd.spec.trans_type = USNIC_TRANSPORT_IPV4_UDP;
	cmd->usnic_cmd.spec.udp.sock_fd = qp->qp_socket;

	return 0;
}

void usnic_undo_qp_create_prep(struct usnic_qp *qp)
{
	close(qp->qp_socket);
}

void usnic_rq_desc_to_work_comp(struct cq_enet_rq_desc *desc,
				struct ibv_wc *wc,
				struct vnic_rq *UNUSED(rq),
				struct vnic_rq_buf *buf)
{
	union usnic_hdr *udp_hdr;
	u8 type, color, ingress_port, fcoe, vlan_stripped;
	u8 packet_error, fcs_ok, sop, eop, rss_type, csum_not_calc;
	u8 fcoe_sof, fcoe_fc_crc_ok, fcoe_enc_error, fcoe_eof;
	u8 tcp_udp_csum_ok, udp, tcp, ipv4_csum_ok, ipv4, ipv6;
	u8 ipv4_fragment;
	u16 bytes_written, vlan_tci, completed_index, q_number;
	u16 checksum;
	u32 rss_hash;

	VALGRIND_MAKE_MEM_DEFINED(desc, sizeof(*desc));
	cq_enet_rq_desc_dec(desc, &type, &color, &q_number, &completed_index,
		&ingress_port, &fcoe, &eop, &sop, &rss_type,
		&csum_not_calc, &rss_hash, &bytes_written, &packet_error,
		&vlan_stripped, &vlan_tci, &checksum, &fcoe_sof,
		&fcoe_fc_crc_ok, &fcoe_enc_error, &fcoe_eof,
		&tcp_udp_csum_ok, &udp, &tcp, &ipv4_csum_ok,
		&ipv6, &ipv4, &ipv4_fragment, &fcs_ok);

	memset(wc, 0, sizeof(*wc));
	wc->wr_id = buf->wr_id;
	wc->opcode = IBV_WC_RECV;
	wc->vendor_err = packet_error;

	if (unlikely(packet_error)) {
		if (fcs_ok)
			wc->status = IBV_WC_LOC_PROT_ERR;
		else if (0 == bytes_written)
			wc->status = IBV_WC_LOC_LEN_ERR;
		else
			wc->status = IBV_WC_LOC_EEC_OP_ERR;
		wc->byte_len = bytes_written;
		return;
	}

#if WANT_DEBUG_MSGS
	if (csum_not_calc) {
		usnic_err("ipv4 header checksum not calculated\n");
		assert(0);
	}

	if (!ipv4_csum_ok)
		usnic_err("wrong ipv4 header checksum!\n");

	if (ipv4_fragment) {
		usnic_err("received an ipv4 fragment\n");
		assert(0);
	}

	if (!tcp_udp_csum_ok)
		usnic_err("wrong udp checksum!\n");
#endif
	/*
	 * From sereno TOO 7.4.18.1, if checksum_not_calculated bit is set
	 * All checksum bits are set to zero. So here only checks checksum
	 * bits directly.
	 * Also if the packet is an IPv4/6 fragment, tcp_udp_checksum_ok is
	 * set to zero. So ipv4_fragment is not checked here.
	 */
	if (unlikely(!ipv4_csum_ok || !tcp_udp_csum_ok)) {
		wc->status = IBV_WC_LOC_EEC_OP_ERR;
		wc->byte_len = bytes_written;
		return;
	}

	wc->status = IBV_WC_SUCCESS;
	VALGRIND_MAKE_MEM_DEFINED(buf->dma_addr, bytes_written);

	udp_hdr = (union usnic_hdr *) buf->dma_addr;

	wc->qp_num = ntohs(udp_hdr->hdr.udp.dest);
	wc->src_qp = ntohs(udp_hdr->hdr.udp.source);
	wc->byte_len = ntohs(udp_hdr->hdr.ip.tot_len) +
				sizeof(struct ether_header);

	return;
}

struct ibv_ah *usnic_create_ah(struct ibv_pd *pd, struct ibv_ah_attr *attr)
{
	struct usnic_device		*udev;
	struct usnic_ah			*ah;
	uint32_t			src_ip_addr;
	char				addr_buf[INET_ADDRSTRLEN];
	int				err;

	if (pd->context->ops.post_send == NULL) {
		usnic_err("Application must enable UDP transport type\n");
		errno = EINVAL;
		return NULL;
	}

	if (!attr->is_global) {
		usnic_err("Must specify the destination gid to create ah\n");
		errno = EINVAL;
		return NULL;
	}

	udev = to_udev(pd->context->device);
	err = usnic_get_local_ip(pd->context, &src_ip_addr);
	if (err) {
		errno = EADDRNOTAVAIL;
		return NULL;
	}

	ah = calloc(1, sizeof(*ah));
	if (!ah) {
		usnic_perr("Failed to allocate memory for usnic_ah\n");
		errno = ENOMEM;
		return NULL;
	}

	ah->pd = pd;
	memcpy(&ah->dgid, &attr->grh.dgid, sizeof(ah->dgid));
	usnic_udp_gid_to_ipaddr(&attr->grh.dgid, &ah->dip);
	inet_ntop(AF_INET, &ah->dip, addr_buf, sizeof(addr_buf));

	err = usnic_resolve_dst(udev->if_index, src_ip_addr, ah->dip, ah->dmac);
	if (err) {
		char src_buf[INET_ADDRSTRLEN];

		/*
		 * propagate errno back to application
		 * EHOSTUNREACH: route lookup fails
		 * EAGAIN: ARP resolution for nexthop ongoing
		 * ENXIO: ARP resolution fails
		 * other: other fatal errors
		 */
		inet_ntop(AF_INET, &src_ip_addr, src_buf, sizeof(src_buf));
		usnic_strerror(err,
				"failed to resolve dst %s on if %d device %s src ip: %s",
				addr_buf, udev->if_index,
				udev->ifname, src_buf);
		free(ah);
		errno = err;
		return NULL;
	}
	usnic_info("Dest: %s next hop MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
				addr_buf, ah->dmac[0], ah->dmac[1],
				ah->dmac[2], ah->dmac[3], ah->dmac[4],
				ah->dmac[5]);

	return &ah->ibv_ah;
}

int usnic_destroy_ah(struct ibv_ah *ah)
{
	free(to_uah(ah));

	return 0;
}
