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

#ifndef USNIC_VERBS_UTIL_H
#define USNIC_VERBS_UTIL_H

#include <infiniband/verbs.h>

#define USNIC_DEFAULT_PORT 1
#define USNIC_DEFAULT_GID_IDX 0

static inline void
usnic_udp_gid_to_ipaddr(union ibv_gid *gidp, uint32_t *ipaddr)
{
	memcpy(ipaddr, &gidp->raw[4], sizeof(*ipaddr));
}

int usnic_get_local_ip(struct ibv_context *context, uint32_t *ip_addr);
int usnic_ipaddr_to_ifinfo(uint32_t if_ipaddr, char *ifname, size_t ifname_len, int *if_index);
int usnic_get_if_mtu(char *ifname, int *mtu);

#endif /* USNIC_VERBS_UTIL_H */
