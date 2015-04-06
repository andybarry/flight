import dbus
from ctypes import c_longlong


bus = dbus.SessionBus()

parole = bus.get_object('org.Parole.Media.Player', '/org/Parole/Media/Player')

iface = dbus.Interface(parole, 'org.mpris.MediaPlayer2')

print parole.Introspect()
print iface.Introspect()

parole.SetPosition(dbus.ObjectPath("/org/mpris/MediaPlayer2/TrackList/0x7f7c5a8dd100"), dbus.Int64(50000000))
