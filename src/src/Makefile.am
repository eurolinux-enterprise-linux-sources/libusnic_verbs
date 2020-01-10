# Copyright (c) 2014, Cisco Systems, Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#  Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in
#  the documentation and/or other materials provided with the
#  distribution.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# this is drivers/common (and tmp override)
VNIC_CMN_DIR=$(abs_srcdir)/../../../../common

# this is top include (and tmp override)
VNIC_INC_DIR=$(abs_srcdir)/../../../../../include

USER_CMN_DIR=$(abs_srcdir)/../../common
USNIC_CMN_DIR=$(abs_srcdir)/../../../common

VPATH=$(srcdir):$(VNIC_CMN_DIR):$(VNIC_INC_DIR):$(USNIC_CMN_DIR):$(USER_CMN_DIR):$(USER_CMN_DIR)/linux

AM_CFLAGS = -g -Wall -Wextra -D_GNU_SOURCE
AM_CPPFLAGS = \
	-I$(TMP_VNIC_CMN_DIR) \
	-I$(TMP_VNIC_INC_DIR) \
	-I$(VNIC_CMN_DIR) \
	-I$(VNIC_INC_DIR) \
	-I$(USNIC_CMN_DIR) \
	-I$(USER_CMN_DIR) \
	-include $(srcdir)/usnic_common.h

usnic_version_script = @USNIC_VERSION_SCRIPT@

lib_LTLIBRARIES = libusnic_verbs.la

libusnic_verbs_la_SOURCES = \
	linux_types.h \
	kcompat.h \
	usnic_ib_abi.h \
	usnic_abi.h \
	usnic_common_pkt_hdr.h \
	usnic_common_util.h \
	usnic_common.h \
	usnic_context.h \
	usnic.h \
	verbs_stats.h \
	usnic_user_utils.h \
	usnic_utils.h \
	kcompat_priv.h \
	usnic_transport_priv.h \
	vnic_resource.h \
	vnic_devcmd.h \
	vnic_dev.h \
	vnic_cq.h \
	vnic_wq.h \
	vnic_rq.h \
	vnic_intr.h \
	vnic_enet.h \
	vnic_stats.h \
	vnic_nic.h \
	usnic_verbs.h \
	delay.h \
	slab.h \
	cq_enet_desc.h \
	wq_enet_desc.h \
	rq_enet_desc.h \
	cq_desc.h \
	context.c \
	driver.c \
	utils.c \
	verbs.c \
	verbs_stats.c \
	qp.c \
	cq.c \
	vnic_cq.c \
	vnic_dev.c \
	vnic_rq.c \
	vnic_wq.c \
	version.c

libusnic_verbs_la_LDFLAGS = \
	-module \
	-Wl,--build-id -release @IBV_DEVICE_LIBRARY_EXTENSION@ \
    $(usnic_version_script)

if USNIC_8915
libusnic_verbs_la_SOURCES += usnic_transport_8915.h \
								usnic_transport_8915.c
else !USNIC_8915
AM_CPPFLAGS += @LIBNL_CFLAGS@
libusnic_verbs_la_SOURCES += usnic_verbs_ext.h \
								usnic_verbs_ext.c \
								usnic_transport_udp.h \
								usnic_transport_udp.c \
								usnic_ip_utils.h \
								usnic_ip_utils.c \
								libnl_utils.h	\
								libnl_utils_common.c
if HAVE_LIBNL3
libusnic_verbs_la_SOURCES += libnl3_utils.h
else !HAVE_LIBNL3
libusnic_verbs_la_SOURCES += libnl1_utils.h
endif !HAVE_LIBNL3
libusnic_verbs_la_LDFLAGS += $(LIBNL_LIBS)
endif !USNIC_8915

#
# Automake does not work with real VPATH, and especially not
# with VPATHed file that live *above* $srcdir
# Make the tarball, copying *all* *.[ch] files to src, we are essentially
# using VPATH to flatten everything anyhow.
#
distdir: $(DISTFILES)
	@echo distdir=$(distdir); \
	for file in $?; do \
		absfile=`readlink -f $${file}`; \
		filename=$${file##*/}; \
		if [[ "$${absfile}" = $(abs_srcdir)/* ]]; then \
			destfile=$${absfile#$(abs_srcdir)/}; \
		else \
			destfile=$${filename}; \
		fi; \
		distsubdir=$${destfile%$${filename}}; \
		if [[ -n "$${distsubdir}" && ! -d $${distdir}/$${distsubdir} ]]; then \
			$(MKDIR_P) $${distdir}/$${distsubdir}; \
		fi; \
		cp $${file} $${distdir}/$${destfile}; \
	done; \
	$(MKDIR_P) $${distdir}/linux; \
	for f in slab.h delay.h; do \
		mv $${distdir}/$${f} $${distdir}/linux; \
	done
