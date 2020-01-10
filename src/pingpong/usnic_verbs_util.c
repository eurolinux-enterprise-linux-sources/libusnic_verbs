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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "usnic_verbs_util.h"

int usnic_get_local_ip(struct ibv_context *context, uint32_t *ip_addr)
{
	int err;
	union ibv_gid my_gid;

	/* port_num is always 1 and gid index is always 0 for usNIC */
	err = ibv_query_gid(context, 1, 0, &my_gid);
	if (err) {
		fprintf(stderr, "Failed to query gid for ib device: %s\n",
				context->device->name);
		return EFAULT;
	}
	usnic_udp_gid_to_ipaddr(&my_gid, ip_addr);

	return 0;
}

int usnic_ipaddr_to_ifinfo(uint32_t if_ipaddr, char *ifname, size_t ifname_len, int *if_index)
{
	struct ifaddrs *ifaddr, *ifa;
	int err = EADDRNOTAVAIL;

	if (getifaddrs(&ifaddr) == -1) {
		err = errno;
		perror("getifaddr failed");
		return err;
	}

	for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
		if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
			if (((struct sockaddr_in *)ifa->ifa_addr)
				->sin_addr.s_addr == if_ipaddr) {
				*if_index = if_nametoindex(ifa->ifa_name);
				if (!(*if_index)) {
					char err_buf[50];

					err = errno;
					snprintf(err_buf, sizeof(err_buf),
						"if_nametoindex failed, if name: %s",
						ifa->ifa_name);
					perror(err_buf);
					break;
				}
				memset(ifname, 0, ifname_len);
				strncpy(ifname, ifa->ifa_name, ifname_len - 1);
				err = 0;
				break;
			}
		}
	}

	freeifaddrs(ifaddr);
	return err;
}

int usnic_get_if_mtu(char *ifname, int *mtu)
{
	int		sockfd;
	struct ifreq	ifr;
	int		err;

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd == -1) {
		err = errno;
		perror("socket creation for MTU query failed");
		return -1;
	}

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
	if (ioctl(sockfd, SIOCGIFMTU, &ifr) == -1) {
		*mtu = 0;
		err = errno;
	}
	else {
		*mtu = ifr.ifr_mtu;
		err = 0;
	}

	return err;
}
