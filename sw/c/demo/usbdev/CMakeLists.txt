add_executable(usbdev "usbdev_stream_test.c;usb_testutils_streams.c;usb_testutils.c;usb_testutils_diags.c;usb_testutils_controlep.c;dif_usbdev.c;dif_usbdev_autogen.c;print.c;memory.c")
target_link_libraries(usbdev common)
