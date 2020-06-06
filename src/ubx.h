#ifndef __UBX_H__
#define __UBX_H__

#define UBX_NAV_CLSID                0x01
#define UBX_NAV_PVT_MSGID            0x07
#define UBX_NAV_SAT_MSGID            0x35
#define UBX_NAV_ORB_MSGID            0x34
#define UBX_NAV_STATUS_MSGID         0x03
#define UBX_NAV_SIG_MSGID            0x43
#define UBX_NAV_CLOCK_MSGID          0x22
#define UBX_NAV_DOP_MSGID            0x04
#define UBX_NAV_POSECEF_MSGID        0x01
#define UBX_NAV_HPPOSECEF_MSGID      0x13
#define UBX_NAV_POSLLH_MSGID         0x02
#define UBX_NAV_HPPOSLLH_MSGID       0x14
#define UBX_NAV_RELPOSNED_MSGID      0x3c
#define UBX_NAV_VELECEF_MSGID        0x11
#define UBX_NAV_VELNED_MSGID         0x12
#define UBX_NAV_SVIN_MSGID           0x3b
#define UBX_NAV_EOE_MSGID            0x61
#define UBX_NAV_GEOFENCE_MSGID       0x39
#define UBX_NAV_ODO_MSGID            0x09
#define UBX_NAV_RESETODO_MSGID       0x10
#define UBX_NAV_TIMEUTC_MSGID        0x21
#define UBX_NAV_TIMELS_MSGID         0x26
#define UBX_NAV_TIMEGPS_MSGID        0x20
#define UBX_NAV_TIMEGLO_MSGID        0x23
#define UBX_NAV_TIMEBDS_MSGID        0x24
#define UBX_NAV_TIMEGAL_MSGID        0x25

#define UBX_RXM_CLSID                0x02
#define UBX_RXM_MEASX_MSGID          0x14
#define UBX_RXM_RAWX_MSGID           0x15

#define UBX_MON_CLSID                0x0a
#define UBX_MON_RF_MSGID             0x38

#define UBX_CLSID(msg)                         ((msg)[2])
#define UBX_MSGID(msg)                         ((msg)[3])

#endif // __UBX_H__