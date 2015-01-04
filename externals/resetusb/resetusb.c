#include <stdio.h>
#include <usb.h>

int main(void)
{
      struct usb_bus *busses;
      usb_init();
      usb_find_busses();
      usb_find_devices();
      busses = usb_get_busses();
      struct usb_bus *bus;
      int c, i, a;
      /* ... */
      for (bus = busses; bus; bus = bus->next) {
        struct usb_device *dev;
        int val;
        usb_dev_handle *junk;
        for (dev = bus->devices; dev; dev = dev->next) {
          char buf[1024];
          junk = usb_open ( dev );
          usb_get_string_simple(junk,2,buf,1023);
          if ( junk == NULL ){
            printf("Can't open %p (%s)\n", dev, buf );
          } else {
            val = usb_reset(junk);
            printf( "reset %p %d (%s)\n", dev, val, buf );
          }
          usb_close(junk);
        }
      }
}
