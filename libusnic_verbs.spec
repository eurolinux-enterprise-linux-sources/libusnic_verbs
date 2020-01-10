%global tools_version 1.0.1.237

Name: libusnic_verbs
Version: 1.1.0.237
Release: 2%{?dist}
Summary: Cisco Virtual NIC OpenFabrics Userspace Driver
Group: System Environment/Libraries
License: GPLv2 or BSD
Url: http://cisco.com/
Source0: libusnic_verbs-%{version}.tar.gz
Source1: usnic_tools-%{tools_version}.tar.gz
Patch0: usnic_tools-kmod.patch
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)
BuildRequires: libibverbs-devel >= 1.1.5, libnl3-devel, valgrind-devel
ExcludeArch: s390 s390x

%description
libusnic_verbs provides a device-specific userspace driver for Cisco
Virtual NICs for use with the libibverbs library. This package also
includes an usnic_udp_pingpong modified from ibv_ud_pingpong that works
with libusnic_verbs.

%package utils
Summary: Simple utilities to test Cisco Virtual NIC operability
Group: System Environment/Libraries

%description utils
This package includes ibv_ud_pingpong modified to work with libusnic_verbs
devices and a few minor scripts for checking the versions of software
installed on the machine and providing useful information to Cisco
technical support.

%prep
%setup -q -n %{name}-%{version}.rhel7u0 -a 1
%patch0 -p1

%build
%configure --with-release=%{version} --with-valgrind
make %{?_smp_mflags}
cd usnic_tools-%{tools_version}.rhel7u0
%configure --with-release=%{tools_version}

%install
rm -rf %{buildroot}
make DESTDIR=%{buildroot} install
# remove unpackaged files from the buildroot
rm -f %{buildroot}%{_libdir}/*.la %{buildroot}%{_libdir}/libusnic_verbs.so
cd usnic_tools-%{tools_version}.rhel7u0
make DESTDIR=%{buildroot} install
mv %{buildroot}/usr/opt/cisco/usnic/bin/* %{buildroot}%{_bindir}
rm -fr %{buildroot}/usr/opt
mv %{buildroot}%{_bindir}/{,usnic_}tech_support_info
chmod +x %{buildroot}%{_bindir}/*

%clean
rm -rf %{buildroot}

%files
%defattr(-,root,root,-)
%{_libdir}/libusnic_verbs-rdmav2.so
%{_sysconfdir}/libibverbs.d/usnic.driver
%doc AUTHORS COPYING README KNOWN_ISSUES

%files utils
%defattr(-,root,root,-)
%{_bindir}/*

%changelog
* Fri Jan 23 2015 Doug Ledford <dledford@redhat.com> - 1.1.0.237-2
- Some of the scripts are missing the execute bit, fix that
- Clean up the check script to be more appropriate for in-box use
- Resolves: bz1182413, bz1182417

* Mon Oct 27 2014 Doug Ledford <dledford@redhat.com> - 1.1.0.237-1
- Initial import into rhel
- Related: bz916384
