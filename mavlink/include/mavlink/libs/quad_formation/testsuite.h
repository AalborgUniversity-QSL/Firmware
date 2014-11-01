/** @file
 *	@brief MAVLink comm protocol testsuite generated from quad_formation.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef QUAD_FORMATION_TESTSUITE_H
#define QUAD_FORMATION_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_quad_formation(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_quad_formation(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_quad_pos(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_quad_pos_t packet_in = {
		17235,17339,17443,151
    };
	mavlink_quad_pos_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.pos_no = packet_in.pos_no;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_quad_pos_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_quad_pos_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_quad_pos_pack(system_id, component_id, &msg , packet1.pos_no , packet1.x , packet1.y , packet1.z );
	mavlink_msg_quad_pos_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_quad_pos_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pos_no , packet1.x , packet1.y , packet1.z );
	mavlink_msg_quad_pos_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_quad_pos_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_quad_pos_send(MAVLINK_COMM_1 , packet1.pos_no , packet1.x , packet1.y , packet1.z );
	mavlink_msg_quad_pos_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_quad_formation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_quad_pos(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // QUAD_FORMATION_TESTSUITE_H
