# PL2303 Vendor Requests from PCAP

|pkt|dir|bmRequestType|bRequest|wValue|wIndex|wLength|data(hex)|notes|
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|276|IN|0xc0|0x01|0x8484|0x0000|1|4c|Reply with single byte 0x4C (observed in PCAP)|
|286|OUT|0x40|0x01|0x0404|0x0000|0||Acknowledge (host-to-device write): no data stage| 
|292|IN|0xc0|0x01|0x8484|0x0000|1|4c|Reply with single byte 0x4C (observed in PCAP)|
|301|IN|0xc0|0x01|0x8383|0x0000|1|f8|Reply with single byte 0xF8 (observed in PCAP)|
|310|IN|0xc0|0x01|0x8484|0x0000|1|4c|Reply with single byte 0x4C (observed in PCAP)|
|319|OUT|0x40|0x01|0x0404|0x0001|0||Acknowledge vendor write (variant index=1)
|325|IN|0xc0|0x01|0x8484|0x0000|1|4c|Reply with single byte 0x4C (observed in PCAP)|
|334|IN|0xc0|0x01|0x8383|0x0000|1|f8|Reply with single byte 0xF8 (observed in PCAP)|
|343|OUT|0x40|0x01|0x0000|0x0001|0||Vendor write (value 0)
|349|OUT|0x40|0x01|0x0001|0x0000|0||Vendor write (value 1)
|355|OUT|0x40|0x01|0x0002|0x0044|0||Vendor write (value 2, index 0x0044)
