%global lib_version 2.0.1
%global tools_version 1.1.1.0

Name: libusnic_verbs
Version: %{lib_version}
Release: 4%{?dist}
Summary: Cisco Virtual NIC OpenFabrics Userspace Driver
Group: System Environment/Libraries
License: GPLv2 or BSD
Url: http://cisco.com/
Source0: https://github.com/cisco/libusnic_verbs/releases/download/v%{lib_version}/%{name}-%{lib_version}.tar.bz2
Source1: https://github.com/cisco/usnic_tools/releases/download/v%{tools_version}/usnic-tools-%{tools_version}.tar.bz2
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)
BuildRequires: libibverbs-devel >= 1.2.0, libfabric-devel >= 1.3.0
BuildRequires: libnl3-devel, valgrind-devel
%ifarch x86_64
BuildRequires: infinipath-psm-devel >= 3.3
%endif
ExcludeArch: s390 s390x

%description
libusnic_verbs provides a device-specific userspace driver for Cisco
Virtual NICs for use with the libibverbs library. This package also
includes an usnic_udp_pingpong modified from ibv_ud_pingpong that works
with libusnic_verbs.

%package -n usnic-tools
Summary: Simple utilities to test Cisco Virtual NIC operability
Group: System Environment/Libraries
Version: %{tools_version}
Provides: libusnic_verbs-utils = 1.1.0.237-5
Obsoletes: libusnic_verbs-utils < 1.1.0.237-5
Requires: %{name} = %{lib_version}-%{release}

%description -n usnic-tools
This package includes ibv_ud_pingpong modified to work with libusnic_verbs
devices and a few minor scripts for checking the versions of software
installed on the machine and providing useful information to Cisco
technical support.

%prep
%setup -q -a 1

%build
%configure --with-release=%{version} --with-valgrind
make %{?_smp_mflags}
pushd usnic-tools-%{tools_version}
%configure
popd

%install
rm -rf %{buildroot}
make DESTDIR=%{buildroot} install
# remove unpackaged files from the buildroot
rm -f %{buildroot}%{_libdir}/*.la %{buildroot}%{_libdir}/libusnic_verbs.so
pushd usnic-tools-%{tools_version}
make DESTDIR=%{buildroot} install
popd

%clean
rm -rf %{buildroot}

%files
%defattr(-,root,root,-)
%{_libdir}/libusnic_verbs-rdmav2.so
%{_sysconfdir}/libibverbs.d/usnic.driver
%{license} LICENSE
%doc AUTHORS VERSION ChangeLog

%files -n usnic-tools
%defattr(-,root,root,-)
%{_bindir}/*

%changelog
* Thu Jul 07 2016 Jarod Wilson <jarod@redhat.com> - 2.0.1-4
- Fix broken Requires: libusnic_verbs dependency in usnic-tools sub-package
- Resolves: bz1353448

* Wed Jun 29 2016 Jarod Wilson <jarod@redhat.com> - 2.0.1-3
- Add missing Requires: libusnic_verbs to usnic-tools sub-package
- Bump to usnic-tools v1.1.1.0, drop patch that was merged upstream
- BuildRequires: libibverbs v1.2.0 or later now, and infinipath-psm-devel
- Related: bz1096997

* Tue May 17 2016 Jarod Wilson <jarod@redhat.com> - 2.0.1-2
- Fix getopt build failure in usnic-tools sub-package
- Resolves: bz1096997

* Tue Apr 12 2016 Jarod Wilson <jarod@redhat.com> - 2.0.1-1
- Update to libusnic_verbs v2.0.1 and usnic-tools v1.1.0.0
- Use actual upstream tarballs
- Call usnic-tools usnic-tools instead of libusnic_verbs-utils
- Resolves: bz1281635

* Wed Sep 30 2015 Doug Ledford <dledford@redhat.com> - 1.1.0.237-4
- Build against libnl3 again now that the UD RoCE bug is fixed
- Related: bz1261028

* Tue Sep 29 2015 Doug Ledford <dledford@redhat.com> - 1.1.0.237-3
- Build against libnl instead of libnl3
- Related: bz1177115

* Fri Jan 23 2015 Doug Ledford <dledford@redhat.com> - 1.1.0.237-2
- Some of the scripts are missing the execute bit, fix that
- Clean up the check script to be more appropriate for in-box use
- Resolves: bz1182413, bz1182417

* Mon Oct 27 2014 Doug Ledford <dledford@redhat.com> - 1.1.0.237-1
- Initial import into rhel
- Related: bz916384
