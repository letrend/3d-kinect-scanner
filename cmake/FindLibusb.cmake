###############################################################################
# Find libusb
#
# This sets the following variables:
# LIBUSB_FOUND - True if Libusb was found.
# LIBUSB_INCLUDE_DIRS - Directories containing the Libusb include files.
# LIBUSB_DEFINITIONS - Compiler flags for Libusb.

find_package(PkgConfig QUIET)
pkg_check_modules(PC_LIBUSB libusb)
set(LIBUSB_DEFINITIONS ${PC_LIBUSB_CFLAGS_OTHER})

find_path(LIBUSB_INCLUDE_DIR libusb-1.0)

set(LIBUSB_INCLUDE_DIRS ${LIBUSB_INCLUDE_DIR}/libusb-1.0)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Libusb DEFAULT_MSG LIBUSB_INCLUDE_DIR)

mark_as_advanced(LIBUSB_INCLUDE_DIR)


