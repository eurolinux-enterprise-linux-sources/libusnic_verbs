# Copyright (c) 2013, Cisco Systems, Inc. All rights reserved.
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

SUBDIRS = src
VPATH = $(srcdir)

ACLOCAL_AMFLAGS = -I m4

AM_CFLAGS = -g -Wall -Wextra -D_GNU_SOURCE

USNIC_WHOLE_VERSION=@USNIC_WHOLE_VERSION@
RPM_RELEASE=@RPM_RELEASE@
USNIC_VERSION=@USNIC_VERSION@
USNIC_RELEASE=@USNIC_RELEASE@
USNIC_REL_DATE=@USNIC_REL_DATE@
USNIC_SVN_BRANCH=@USNIC_SVN_BRANCH@
USNIC_SVN_REV=@USNIC_SVN_REV@
bin_PROGRAMS = pingpong/usnic_udp_pingpong

pingpong_usnic_udp_pingpong_SOURCES = pingpong/ud_pingpong.c \
							pingpong/pingpong.c \
							pingpong/pingpong.h \
							pingpong/usnic_verbs_util.h \
							pingpong/usnic_verbs_util.c \
							src/usnic_verbs_ext.h

pingpong_usnic_udp_pingpong_CPPFLAGS = -I$(srcdir)/src
pingpong_usnic_udp_pingpong_LDFLAGS = -Wl,--build-id
pingpong_usnic_udp_pingpong_LDADD = -libverbs

EXTRA_DIST = \
	libusnic_verbs.spec.in \
	usnic.driver \
	KNOWN_ISSUES

# This is a workaround for a bug in automake:
# http://debbugs.gnu.org/cgi/bugreport.cgi?bug=14594
EXTRA_DIST += config/depcomp
EXTRA_DIST += LICENSE

usnicconfdir = $(sysconfdir)/libibverbs.d
usnicconf_DATA = usnic.driver

define gen_release_version
	@echo "#Auto Generated - Do not edit" >> $(distdir)/RELEASE_VERSION
	@echo "USNIC_VERSION=$(USNIC_VERSION)" >> $(distdir)/RELEASE_VERSION
	@echo "USNIC_REL_DATE=$(USNIC_REL_DATE)" >> $(distdir)/RELEASE_VERSION
	@echo "USNIC_SVN_BRANCH=$(USNIC_SVN_BRANCH)" >> \
					$(distdir)/RELEASE_VERSION
	@echo "USNIC_SVN_REV=$(USNIC_SVN_REV)" >> $(distdir)/RELEASE_VERSION
	@echo "RPM_RELEASE=$(RPM_RELEASE)" >> $(distdir)/RELEASE_VERSION
endef

define insert_license
	files=$$(find $(distdir) -type f); \
	for file in $${files}; do \
		sed -i \
			-e '/^ \* \[Insert appropriate license here/r $(distdir)/LICENSE' \
			-e '/^ \* \[Insert appropriate license here/d' $${file}; \
	done
	rm -rf $(distdir)/LICENSE
endef

dist-hook:
	$(gen_release_version)
	$(insert_license)

libusnic_verbs_src_rpm = libusnic_verbs-${USNIC_WHOLE_VERSION}-${RPM_RELEASE}.src.rpm

$(libusnic_verbs_src_rpm): dist
	rpmbuild --define "_topdir $(abs_builddir)/rpmbuild" \
	    --define "usnic_version ${USNIC_WHOLE_VERSION}" \
	    --define "_sourcedir $(abs_builddir)" -bs libusnic_verbs.spec
	mv rpmbuild/SRPMS/$(libusnic_verbs_src_rpm) .

.PHONY: valgrind_rpm
valgrind_rpm: $(libusnic_verbs_src_rpm)
	rpmbuild --rebuild \
	    --define "_topdir $(abs_builddir)/rpmbuild" \
	    --define "usnic_release ${USNIC_RELEASE}" \
	    --define "usnic_version ${USNIC_WHOLE_VERSION}" \
		--define "valgrind $(VALGRIND_PATH)" \
	    $(libusnic_verbs_src_rpm)

.PHONY: releng
releng: $(libusnic_verbs_src_rpm) $(VALGRIND_RPM)
	rpmbuild --rebuild \
	    --define "_topdir $(abs_builddir)/rpmbuild" \
	    --define "usnic_release ${USNIC_RELEASE}" \
	    --define "usnic_version ${USNIC_WHOLE_VERSION}" \
	    $(libusnic_verbs_src_rpm)
	mv rpmbuild/RPMS/$(ARCH)/* .

clean-local: clean-local-check
.PHONY: clean-local-check
clean-local-check:
	-rm -rf *.rpm rpmbuild *.tar.gz
