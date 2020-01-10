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
#include <netinet/in.h>

#include "usnic.h"
#include "usnic_ib_abi.h"
#include "usnic_utils.h"
#include "usnic_verbs.h"
#include "usnic_transport_priv.h"

static void
usnic_8915_to_grh(struct vnic_rq_buf *buf, struct ibv_wc *wc)
{
	union usnic_hdr *hdr;

	hdr = (union usnic_hdr *) buf->dma_addr;
	wc->qp_num = ntohl(hdr->l2.dqpn);
	wc->src_qp = ntohl(hdr->l2.sqpn);
	wc->byte_len = ntohs(hdr->l2.payload_len) + sizeof(struct grh_hdr);
	if (hdr->l2.flags & USNIC_FLAGS_IMM_DATA_BIT) {
		wc->wc_flags |= IBV_WC_WITH_IMM;
		wc->imm_data = ntohl(hdr->l2.imm_data);
	}

	struct grh_hdr *grh = (struct grh_hdr *) buf->dma_addr;
	wc->wc_flags |= IBV_WC_GRH;
	/* Manipuating the hdr in place
	 * Need to write the gid interface_id only first, as
	 * oppose writing interface_id and subnet mask,
	 * to prevent the write buf from smashing the read buf
	 */
	usnic_write_gid_if_id_from_mac((char *)&hdr->l2.smac[0],
					(char *)&grh->src_gid.raw[0]);
	usnic_write_gid_if_id_from_mac((char *)&hdr->l2.dmac[0],
					(char *)&grh->dst_gid.raw[0]);
	grh->src_gid.global.subnet_prefix = 0x000000000080fe;
	grh->dst_gid.global.subnet_prefix = 0x000000000080fe;
	grh->version = 6;
	grh->qos = 0;
	grh->flow_label = 0;
	grh->payload_len = htons(wc->byte_len - sizeof(struct grh_hdr));
	grh->next_hdr = 0;
	grh->hop_limit = 0;
}

static int usnic_dump_gid(char *buf, int buf_sz, union usnic_gid gid)
{
	int offset;

	offset = snprintf(buf, buf_sz,
			"|%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x"
			":%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x|",
			gid.raw[0], gid.raw[1], gid.raw[2],
			gid.raw[3], gid.raw[4], gid.raw[5],
			gid.raw[6], gid.raw[7], gid.raw[8],
			gid.raw[9], gid.raw[10], gid.raw[11],
			gid.raw[12], gid.raw[13], gid.raw[14],
			gid.raw[15]);
	return offset;
}

void print_grh_hdr(const char *label, const char *ptr, unsigned int len)
{
	char buf[128];
	int offset;
	struct grh_hdr *hdr = (struct grh_hdr *) ptr;

	offset = snprintf(buf, sizeof(buf),
		"%s: |%02x|%02x|%02x|%04x|%02x|%02x",
		label,
		hdr->version,
		hdr->qos,
		hdr->flow_label,
		hdr->payload_len,
		hdr->next_hdr,
		hdr->hop_limit);
	offset += usnic_dump_gid(buf, sizeof(buf) - offset, hdr->src_gid);
	offset += usnic_dump_gid(buf, sizeof(buf) - offset, hdr->dst_gid);
	offset += snprintf(buf, sizeof(buf) - offset, "...SIZE(%u)\n", len);

	fprintf(stdout, buf);
}

int usnic_prep_qp_create_cmd(struct usnic_context *UNUSED(uctx),
				struct usnic_qp *UNUSED(qp),
				struct usnic_create_qp *cmd)
{
	cmd->usnic_cmd.spec.trans_type = USNIC_TRANSPORT_ROCE_CUSTOM;
	return 0;
}

void usnic_undo_qp_create_prep(struct usnic_qp *UNUSED(qp))
{
	/*
	 * Function body is empty because this is not neccessary for this
	 * transport, but is for udp transport.
	 */
}

void usnic_rq_desc_to_work_comp(struct cq_enet_rq_desc *desc,
				struct ibv_wc *wc,
#if WANT_DEBUG_MSGS
				struct vnic_rq *rq,
#else
				struct vnic_rq *UNUSED(rq),
#endif /* WANT_DEBUG_MSGS */
				struct vnic_rq_buf *buf)
{
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

	if (packet_error) {
		if (fcs_ok)
			wc->status = IBV_WC_LOC_PROT_ERR;
		else if (0 == bytes_written)
			wc->status = IBV_WC_LOC_LEN_ERR;
		else
			wc->status = IBV_WC_LOC_EEC_OP_ERR;
		wc->byte_len = bytes_written;
		return;
	}

	wc->status = IBV_WC_SUCCESS;
	VALGRIND_MAKE_MEM_DEFINED(buf->dma_addr, bytes_written);

	usnic_8915_to_grh(buf, wc);
#if WANT_DEBUG_MSGS
	if (wc->qp_num != rq->qp_num) {
		usnic_err("wc qp_num: %u qp_num:%u\n", wc->qp_num, rq->qp_num);
		assert(0);
	}
	assert(eop);
#endif
}

struct ibv_ah *usnic_create_ah(struct ibv_pd *pd, struct ibv_ah_attr *attr)
{
	struct usnic_ah	*ah;

	if (!attr->is_global) {
		usnic_err("Must specify the destination gid to create ah\n");
		return NULL;
	}

	if (!gid_is_valid(&attr->grh.dgid))
		return NULL;

	ah = calloc(1, sizeof(*ah));
	if (!ah)
		return NULL;

	ah->pd = pd;
	memcpy(&ah->dgid, &attr->grh.dgid, sizeof(ah->dgid));
	usnic_gid_to_mac(&ah->dgid, &ah->dmac[0]);

	return &ah->ibv_ah;
}

int usnic_destroy_ah(struct ibv_ah *ah)
{
	free(to_uah(ah));

	return 0;
}
