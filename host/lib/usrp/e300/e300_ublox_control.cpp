#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <iostream>

#include "e300_ublox_control.hpp"

#include <uhd/utils/byteswap.hpp>
#include <uhd/utils/msg.hpp>


namespace uhd { namespace usrp { namespace gps {

namespace ublox { namespace ubx {

control::control(const std::string &node, const size_t baud_rate)
{
    _decode_init();
    _serial = boost::make_shared<async_serial>(node, baud_rate);
    _serial->set_read_callback(boost::bind(&control::_rx_callback, this, _1, _2));



    //_send_message(MSG_MON_HW, NULL, 0);
    //_send_message(MSG_MON_VER, NULL, 0);

    //_send_message(MSG_CFG_ANT, NULL, 0);
    //_wait_for_ack(MSG_CFG_ANT, 1.0);

    configure_message_rate(MSG_GLL, 0);
    configure_message_rate(MSG_GSV, 0);
    configure_message_rate(MSG_GGA, 0);
    configure_message_rate(MSG_GSA, 0);
    configure_message_rate(MSG_RMC, 0);
    configure_message_rate(MSG_VTG, 0);
    configure_message_rate(MSG_NAV_TIMEUTC, 2);
    configure_message_rate(MSG_NAV_SOL, 2);

    //configure_antenna(0x001b, 0x8251);

    //boost::this_thread::sleep(boost::posix_time::seconds(1));

//    configure_antenna(0x001a, 0x8251);

    UHD_MSG(status) << "Turning on PPS ... " << std::endl;
    configure_pps(0xf4240, 0x3d090, 1, 1, 1, 0, 0, 0);

    //boost::this_thread::sleep(boost::posix_time::seconds(5));

    //configure_pps(0xf4240, 0x3d090, 1, 1, 0, 0, 0, 0);

//    configure_message_rate(MSG_NAV_TIMEUTC, 0);
}

void control::_decode_init(void)
{
    _decode_state = DECODE_SYNC1;
    _rx_ck_a = 0;
    _rx_ck_b = 0;
    _rx_payload_length = 0;
    _rx_payload_index  = 0;
}

void control::_add_byte_to_checksum(const boost::uint8_t b)
{
    _rx_ck_a = _rx_ck_a + b;
    _rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void control::_calc_checksum(
    const boost::uint8_t *buffer,
    const boost::uint16_t length,
    checksum_t &checksum)
{
    for (size_t i = 0; i < length; i++)
    {
        checksum.ck_a = checksum.ck_a + buffer[i];
        checksum.ck_b = checksum.ck_b + checksum.ck_a;
    }
}

void control::configure_message_rate(
    const boost::uint16_t msg,
    const uint8_t rate)
{
    payload_tx_cfg_msg_t cfg_msg;
    cfg_msg.msg  = uhd::htowx<boost::uint16_t>(msg);
    cfg_msg.rate = rate;
    _send_message(
        MSG_CFG_MSG,
        reinterpret_cast<const uint8_t*>(&cfg_msg),
        sizeof(cfg_msg));

    _wait_for_ack(MSG_CFG_MSG, 1.0);
}

void control::configure_antenna(
    const boost::uint16_t flags,
    const boost::uint16_t pins)
{
    payload_tx_cfg_ant_t cfg_ant;
    cfg_ant.pins = uhd::htowx<boost::uint16_t>(pins);
    cfg_ant.flags = uhd::htowx<boost::uint16_t>(flags);
    _send_message(
        MSG_CFG_ANT,
        reinterpret_cast<const uint8_t*>(&cfg_ant),
        sizeof(cfg_ant));
    _wait_for_ack(MSG_CFG_ANT, 1.0);
}

void control::configure_pps(
    const boost::uint32_t interval,
    const boost::uint32_t length,
    const boost::int8_t status,
    const boost::uint8_t time_ref,
    const boost::uint8_t flags,
    const boost::int16_t antenna_delay,
    const boost::int16_t rf_group_delay,
    const boost::int32_t user_delay)
{
    payload_tx_cfg_tp_t cfg_tp;
    cfg_tp.interval = uhd::htowx<boost::uint32_t>(interval);
    cfg_tp.length = uhd::htowx<boost::uint32_t>(length);
    cfg_tp.status = status;
    cfg_tp.time_ref = time_ref;
    cfg_tp.flags = flags;
    cfg_tp.antenna_delay = uhd::htowx<boost::int16_t>(antenna_delay);
    cfg_tp.rf_group_delay = uhd::htowx<boost::int16_t>(rf_group_delay);
    cfg_tp.user_delay = uhd::htowx<boost::int32_t>(user_delay);
    _send_message(
        MSG_CFG_TP,
        reinterpret_cast<const uint8_t*>(&cfg_tp),
        sizeof(cfg_tp));
    _wait_for_ack(MSG_CFG_TP, 1.0);
}


void control::_rx_callback(const char *data, unsigned int len)
{
    //std::cout << "IN RX CALLBACK" << std::flush << std::endl;
    std::vector<char> v(data, data+len);
    BOOST_FOREACH(const char &c, v)
    {
        _parse_char(c);
    }
}

int control::_parse_char(const boost::uint8_t b)
{
    int ret = 0;
    //std::cout << boost::format("in state %ld got: 0x%lx") % _decode_state % int(b) << std::endl;

    switch (_decode_state) {

    // we're expecting the first sync byte
    case DECODE_SYNC1:
        if (b == SYNC1) { // sync1 found goto next step
            _decode_state = DECODE_SYNC2;
        } // else stay around
        break;

    // we're expecting the second sync byte
    case DECODE_SYNC2:
        if (b == SYNC2) { // sync2 found goto next step
            _decode_state = DECODE_CLASS;
        } else {
            // failed, reset
            _decode_init();
        }
        break;

    // we're expecting the class byte
    case DECODE_CLASS:
        //std::cout << boost::format("DECODE_CLASS, ck_a = %lx, ck_b= %lx")
            //% int(_rx_ck_a) % int(_rx_ck_b) << std::endl;
        _add_byte_to_checksum(b);
        _rx_msg = b;
        _decode_state = DECODE_ID;
        break;

    // we're expecting the id byte
    case DECODE_ID:
        //std::cout << boost::format("DECODE_ID, ck_a = %lx, ck_b= %lx")
            //% int(_rx_ck_a) % int(_rx_ck_b) << std::endl;

        _add_byte_to_checksum(b);
        _rx_msg |= (b << 8);
        _decode_state = DECODE_LENGTH1;
        break;

    // we're expecting the first length byte
    case DECODE_LENGTH1:
        //std::cout << boost::format("DECODE_LENGTH1, ck_a = %lx, ck_b= %lx")
            //% int(_rx_ck_a) % int(_rx_ck_b) << std::endl;

        _add_byte_to_checksum(b);
        _rx_payload_length = b;
        _decode_state = DECODE_LENGTH2;
        break;

    // we're expecting the second length byte
    case DECODE_LENGTH2:
        //std::cout << boost::format("DECODE_LENGTH2, ck_a = %lx, ck_b= %lx")
            //% int(_rx_ck_a) % int(_rx_ck_b) << std::endl;

        _add_byte_to_checksum(b);
        _rx_payload_length |= (b << 8);
        if (_rx_payload_length > sizeof(_buf)) {
            std::cout << "payload length b0rked, resetting ...." << std::endl;
            _decode_init();
            break;
        }
        _decode_state = DECODE_PAYLOAD;
        break;

    // we're expecting payload
    case DECODE_PAYLOAD:
        //std::cout << boost::format("DECODE_PAYLOAD(%ld), length=%ld, ck_a=%lx, ck_b=%lx")
            //% _rx_msg % _rx_payload_length % int(_rx_ck_a) % int(_rx_ck_b) << std::endl;
        _add_byte_to_checksum(b);
        switch(_rx_msg) {
        case MSG_MON_VER:
            ret = _payload_rx_add_mon_ver(b);
            break;
        default:
            ret = _payload_rx_add(b);
            break;
        };
        if (ret < 0) {
            // we couldn't deal with the payload, discard the whole thing
            _decode_init();
        } else if (ret > 0) {
            // payload was complete, let's check the checksum;
            _decode_state = DECODE_CHKSUM1;
        } else {
            // more payload expected, don't move
        }
        ret = 0;
        break;

    case DECODE_CHKSUM1:
        if (_rx_ck_a != b) {
            // checksum didn't match, barf
            std::cout << boost::format("Failed checksum byte1 %lx != %lx")
                % int(_rx_ck_a) % int(b) << std::endl;
            _decode_init();
        } else {
            _decode_state = DECODE_CHKSUM2;
        }
        break;

    case DECODE_CHKSUM2:
        if (_rx_ck_b != b) {
            // checksum didn't match, barf
            std::cout << boost::format("Failed checksum byte2 %lx != %lx")
                % int(_rx_ck_b) % int(b) << std::endl;

        } else {
            ret = _payload_rx_done(); // payload done
        }
        _decode_init();
        break;

    default:
        break;
    };
}

int control::_payload_rx_init(void)
{
    int ret = 0;

    _rx_state = RXMSG_HANDLE; // by default handle
    switch(_rx_msg) {

    case MSG_NAV_SOL:
        if (not _rx_payload_length == sizeof(payload_rx_nav_sol_t))
            _rx_state = RXMSG_ERROR_LENGTH;
        break;

    case MSG_NAV_TIMEUTC:
        if (not _rx_payload_length == sizeof(payload_rx_nav_timeutc_t))
            _rx_state = RXMSG_ERROR_LENGTH;
        break;

    case MSG_MON_VER:
        break; // always take this one

    case MSG_ACK_ACK:
        if (not _rx_payload_length == sizeof(payload_rx_ack_ack_t))
            _rx_state = RXMSG_ERROR_LENGTH;
        break;

    case MSG_ACK_NAK:
        if (not _rx_payload_length == sizeof(payload_rx_ack_nak_t))
            _rx_state = RXMSG_ERROR_LENGTH;
        break;

    default:
        _rx_state = RXMSG_DISABLE;
        break;
    };

    switch (_rx_state) {
    case RXMSG_HANDLE: // handle message
    case RXMSG_IGNORE: // ignore message but don't report error
        ret = 0;
        break;
    };

    return ret;
}

int control::_payload_rx_add(const boost::uint8_t b)
{
    int ret = 0;
    //std::cout << boost::format("_payload_rx_add 0x%lx index=%ld length=%ld") % int(b)
        //% _rx_payload_index % _rx_payload_length << std::endl;
    _buf.raw[_rx_payload_index] = b;
    if (++_rx_payload_index >= _rx_payload_length)
        ret = 1;
    return ret;
}

int control::_payload_rx_add_mon_ver(const boost::uint8_t b)
{
    int ret = 0;
    _buf.raw[_rx_payload_index] = b;
    if (++_rx_payload_index >= _rx_payload_length)
        ret = 1;
    return ret;
}

int control::_payload_rx_done(void)
{
    //if (_rxmsg_state != RXMSG_HANDLE) {
        //return 0;
    //}

    switch (_rx_msg) {
    case MSG_MON_VER:
        std::cout << "MON-VER" << std::endl;
        break;

    case MSG_MON_HW:
        std::cout << "MON-HW" << std::endl;
        break;

    case MSG_ACK_ACK:
        std::cout << boost::format("ACK-ACK for 0x%lx 0x%lx")
            % int(_buf.payload_rx_ack_ack.cls_id) % int(_buf.payload_rx_ack_ack.msg_id) << std::endl;
        if ((_ack_state == ACK_WAITING) and (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg))
            _ack_state = ACK_GOT_ACK;
        break;

    case MSG_ACK_NAK:
        std::cout << boost::format("ACK-NAK for 0x%lx 0x%lx")
            % int(_buf.payload_rx_ack_ack.cls_id) % int(_buf.payload_rx_ack_ack.msg_id) << std::endl;
        if ((_ack_state == ACK_WAITING) and (_buf.payload_rx_ack_nak.msg == _ack_waiting_msg))
            _ack_state = ACK_GOT_NAK;

        break;

    case MSG_CFG_ANT:
        std::cout << boost::format("CFG-ANT for 0x%lx 0x%lx")
            % uhd::wtohx<boost::uint16_t>(_buf.payload_tx_cfg_ant.flags)
            % uhd::wtohx<boost::uint16_t>(_buf.payload_tx_cfg_ant.pins) << std::endl;
        break;

    case MSG_NAV_TIMEUTC:
        std::cout << boost::format("NAV-TIMEUTC %u/%u/%u %02u:%02u:%02u - valid? 0x%lx")
            % boost::uint16_t(_buf.payload_rx_nav_timeutc.day)
            % boost::uint16_t(_buf.payload_rx_nav_timeutc.month)
            % uhd::wtohx<boost::uint16_t>(_buf.payload_rx_nav_timeutc.year)
            % boost::int16_t(_buf.payload_rx_nav_timeutc.hour)
            % boost::int16_t(_buf.payload_rx_nav_timeutc.min)
            % boost::int16_t(_buf.payload_rx_nav_timeutc.sec)
            % boost::int16_t(_buf.payload_rx_nav_timeutc.valid)
            << std::endl;
        break;

    case MSG_NAV_SOL:
        std::cout << boost::format("NAV-SOL - valid? 0x%lx")
            % boost::int16_t(_buf.payload_rx_nav_sol.gps_fix)
            << std::endl;
        break;

    default:
        std::cout << "Got unknown message, with good checksum [";
        for(size_t i = 0; i < _rx_payload_length; i++)
            std::cout << boost::format("%lx, ") % int(_buf.raw[i]);
        std::cout << "]"<< std::endl;
        break;
    };
}

void control::_send_message(
    const boost::uint16_t msg,
    const boost::uint8_t *payload,
    const boost::uint16_t len)
{
    header_t header = {SYNC1, SYNC2, msg, len};
    checksum_t checksum = {0, 0};

    // calculate checksums, first header without sync
    // then payload
    _calc_checksum(
        reinterpret_cast<boost::uint8_t*>(&header) + 2,
        sizeof(header) - 2, checksum);
    if (payload)
        _calc_checksum(payload, len, checksum);

    _serial->write(
        reinterpret_cast<const char*>(&header),
        sizeof(header));

    if (payload)
        _serial->write((const char *) payload, len);

    _serial->write(
        reinterpret_cast<const char*>(&checksum),
        sizeof(checksum));
}

int control::_wait_for_ack(
    const boost::uint16_t msg,
    const double timeout)
{
    int ret = -1;

    _ack_state = ACK_WAITING;
    _ack_waiting_msg = msg;

    boost::system_time timeout_time =
        boost::get_system_time() +
        boost::posix_time::milliseconds(timeout * 1000.0);

    do {
        if(_ack_state == ACK_GOT_ACK)
            return 0;
        else if (_ack_state == ACK_GOT_NAK)
            if (_ack_state == ACK_GOT_NAK) {
            return -1;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(20));
    } while (boost::get_system_time() < timeout_time);

    _ack_state = ACK_IDLE;
    return ret;
}


}} // namespace ublox::ubx
}}} // namespace
