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

#ifndef USNIC_CMN_UTIL_H
#define USNIC_CMN_UTIL_H
#if !defined(__KERNEL__)
#include <linux/types.h>
#endif

static inline void
usnic_mac_to_gid(const char *const mac, char *raw_gid)
{
	raw_gid[0] = 0xfe;
	raw_gid[1] = 0x80;
	memset(&raw_gid[2], 0, 6);
	raw_gid[8] = mac[0]^2;
	raw_gid[9] = mac[1];
	raw_gid[10] = mac[2];
	raw_gid[11] = 0xff;
	raw_gid[12] = 0xfe;
	raw_gid[13] = mac[3];
	raw_gid[14] = mac[4];
	raw_gid[15] = mac[5];
}

static inline void
usnic_mac_ip_to_gid(const char *const mac, const __be32 inaddr, char *raw_gid)
{
	raw_gid[0] = 0xfe;
	raw_gid[1] = 0x80;
	memset(&raw_gid[2], 0, 2);
	memcpy(&raw_gid[4], &inaddr, 4);
	raw_gid[8] = mac[0]^2;
	raw_gid[9] = mac[1];
	raw_gid[10] = mac[2];
	raw_gid[11] = 0xff;
	raw_gid[12] = 0xfe;
	raw_gid[13] = mac[3];
	raw_gid[14] = mac[4];
	raw_gid[15] = mac[5];
}

static inline void
usnic_write_gid_if_id_from_mac(char *mac, char *raw_gid)
{
	raw_gid[8] = mac[0]^2;
	raw_gid[9] = mac[1];
	raw_gid[10] = mac[2];
	raw_gid[11] = 0xff;
	raw_gid[12] = 0xfe;
	raw_gid[13] = mac[3];
	raw_gid[14] = mac[4];
	raw_gid[15] = mac[5];
}

#if defined(__LIBUSNIC__)
static inline void
usnic_udp_gid_to_ipaddr(union ibv_gid *gidp, uint32_t *ipaddr)
{
	memcpy(ipaddr, &gidp->raw[4], sizeof(*ipaddr));
}
#endif /* __LIBUSNIC__ */

#endif /* USNIC_COMMON_UTIL_H */
