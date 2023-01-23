/*
 *
 *    Copyright (c) 2020 Project CHIP Authors
 *    Copyright (c) 2018 Nest Labs, Inc.
 *    All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

/**
 *    @file
 *          Defines platform-specific event types and data for the
 *          CHIP Device Layer on the ESP32.
 */

#pragma once

#include <platform/CHIPDeviceEvent.h>

#include <esp_event.h>

#ifdef CONFIG_MESH_DEVICE
#include "esp_mesh.h"
#endif

namespace chip {
namespace DeviceLayer {

namespace DeviceEventType {

/**
 * Enumerates platform-specific event types that are visible to the application.
 */
enum
{
    kESPSystemEvent = kRange_PublicPlatformSpecific,
};

} // namespace DeviceEventType

/**
 * Represents platform-specific event information for the ESP32 platform.
 */
struct ChipDevicePlatformEvent final
{
    union
    {
        struct
        {
            esp_event_base_t Base;
            int32_t Id;
            union
            {
                ip_event_got_ip_t IpGotIp;
                ip_event_got_ip6_t IpGotIp6;
                ip_event_ap_staipassigned_t IpApStaIpAssigned;
                wifi_event_sta_scan_done_t WifiStaScanDone;
                wifi_event_sta_connected_t WifiStaConnected;
                wifi_event_sta_disconnected_t WifiStaDisconnected;
                wifi_event_sta_authmode_change_t WifiStaAuthModeChange;
                wifi_event_sta_wps_er_pin_t WifiStaWpsErPin;
                wifi_event_sta_wps_fail_reason_t WifiStaWpsErFailed;
                wifi_event_ap_staconnected_t WifiApStaConnected;
                wifi_event_ap_stadisconnected_t WifiApStaDisconnected;
                wifi_event_ap_probe_req_rx_t WifiApProbeReqRecved;

                mesh_event_channel_switch_t channel_switch;            /**< channel switch */
                mesh_event_child_connected_t child_connected;          /**< child connected */
                mesh_event_child_disconnected_t child_disconnected;    /**< child disconnected */
                mesh_event_routing_table_change_t routing_table;       /**< routing table change */
                mesh_event_connected_t connected;                      /**< parent connected */
                mesh_event_disconnected_t disconnected;                /**< parent disconnected */
                mesh_event_no_parent_found_t no_parent;                /**< no parent found */
                mesh_event_layer_change_t layer_change;                /**< layer change */
                mesh_event_toDS_state_t toDS_state;                    /**< toDS state, devices shall check this state firstly before trying to send packets to
                                                                            external IP network. This state indicates right now whether the root is capable of sending
                                                                            packets out. If not, devices had better to wait until this state changes to be
                                                                            MESH_TODS_REACHABLE. */
#ifdef CONFIG_MESH_DEVICE
                mesh_event_vote_started_t vote_started;                /**< vote started */
                mesh_event_root_address_t root_addr;                   /**< root address */
                mesh_event_root_switch_req_t switch_req;               /**< root switch request */
                mesh_event_root_conflict_t root_conflict;              /**< other powerful root */
                mesh_event_root_fixed_t root_fixed;                    /**< fixed root */
                mesh_event_scan_done_t scan_done;                      /**< scan done */
                mesh_event_network_state_t network_state;              /**< network state, such as whether current mesh network has a root. */
                mesh_event_find_network_t find_network;                /**< network found that can join */
                mesh_event_router_switch_t router_switch;              /**< new router information */
                mesh_event_ps_duty_t ps_duty;                          /**< PS duty information */
#endif
            } Data;
        } ESPSystemEvent;
    };
};

} // namespace DeviceLayer
} // namespace chip
