/*
 * Copyright (c) 2014isco Systems, Inc. All rights reserved.
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

#ifndef USNIC_UTILS_H
#define USNIC_UTILS_H

#include <stdio.h>
#include <stddef.h>

#include <infiniband/arch.h>
#include <infiniband/driver.h>

#include "usnic_common_util.h"
#include "usnic_user_utils.h"

struct usnic_pd;
union usnic_gid;
struct usnic_device;
struct usnic_qp;
struct usnic_create_qp;
struct vnic_rq_buf;
struct usnic_context;

void *usnic_alloc_consistent(struct usnic_pd *pd, size_t size);
void usnic_free_consistent(void *vaddr);
void usnic_gid_to_mac(union ibv_gid *gid, uint8_t *mac);
int usnic_get_min_safe_cqe(int cqe_asked);
int gid_is_valid(union ibv_gid *gid);
const char *usnic_qp_type_to_string(enum ibv_qp_type qp_type);
#endif /* USNIC_UTILS_H */
