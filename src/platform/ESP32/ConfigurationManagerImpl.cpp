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
 *          Provides the implementation of the Device Layer ConfigurationManager object
 *          for the ESP32.
 */
/* this file behaves like a config.h, comes first */
#include <platform/internal/CHIPDeviceLayerInternal.h>

#include <core/CHIPKeyIds.h>
#include <platform/ConfigurationManager.h>
#include <platform/ESP32/ESP32Config.h>
#include <platform/internal/GenericConfigurationManagerImpl.cpp>
#include <support/CodeUtils.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
namespace chip {
namespace DeviceLayer {

using namespace ::chip::DeviceLayer::Internal;

namespace {

enum
{
    kChipProduct_Connect = 0x0016
};

} // unnamed namespace

// TODO: Define a Singleton instance of CHIP Group Key Store here (#1266)

/** Singleton instance of the ConfigurationManager implementation object for the ESP32.
 */
ConfigurationManagerImpl ConfigurationManagerImpl::sInstance;
#ifdef CONFIG_MESH_DEVICE
#define CONFIG_MESH_ROUTE_TABLE_SIZE 50
typedef struct mesh_netif_driver* mesh_netif_driver_t;
/*******************************************************
 *                Constants
 *******************************************************/
static const char *MESH_TAG = "mesh_main";

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static esp_netif_t *netif_sta = NULL;
static esp_netif_t *netif_ap = NULL;
static esp_ip4_addr_t s_current_ip;
static mesh_addr_t s_route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];


static void mesh_free(void *h, void* buffer)
{
    free(buffer);
}

static esp_err_t start_mesh_link_sta(void)
{
    uint8_t mac[MAC_ADDR_LEN];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    esp_netif_set_mac(netif_sta, mac);
    esp_netif_action_start(netif_sta, NULL, 0, NULL);
    esp_netif_action_connected(netif_sta, NULL, 0, NULL);
    return ESP_OK;
}

esp_err_t mesh_netif_transmit_from_node_sta(void *h, void *buffer, size_t len)
{
    mesh_data_t data;
    ESP_LOGD(TAG, "Sending to root, dest addr: " MACSTR ", size: %d" ,MAC2STR((uint8_t*)buffer), len);
    data.data = static_cast<uint8_t *>(buffer);
    data.size = static_cast<uint8_t>(len);
    data.proto = MESH_PROTO_AP; // Node's station transmits data to root's AP
    data.tos = MESH_TOS_P2P;
    esp_err_t err = esp_mesh_send(NULL, &data, MESH_DATA_TODS, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Send with err code %d %s", err, esp_err_to_name(err));
    }
    return err;
}
esp_err_t mesh_netif_transmit_from_root_ap(void *h, void *buffer, size_t len)
{
    // Use only to transmit data from root AP to node's AP
    static const uint8_t eth_broadcast[MAC_ADDR_LEN] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };
    int route_table_size = 0;
    mesh_netif_driver_t mesh_driver = static_cast<mesh_netif_driver_t>(h);
    mesh_addr_t dest_addr;
    mesh_data_t data;
    ESP_LOGD(MESH_TAG, "Sending to node: " MACSTR ", size: %d" ,MAC2STR((uint8_t*)buffer), len);
    memcpy(dest_addr.addr, buffer, MAC_ADDR_LEN);
    data.data = static_cast<uint8_t*>(buffer);
    data.size = static_cast<uint16_t>(len);
    data.proto = MESH_PROTO_STA; // sending from root AP -> Node's STA
    data.tos = MESH_TOS_P2P;
    if (MAC_ADDR_EQUAL(dest_addr.addr, eth_broadcast)) {
        ESP_LOGD(MESH_TAG, "Broadcasting!");
        esp_mesh_get_routing_table((mesh_addr_t *) &s_route_table,
                                   CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);
        for (int i = 0; i < route_table_size; i++) {
            if (MAC_ADDR_EQUAL(s_route_table[i].addr, mesh_driver->sta_mac_addr)) {
                ESP_LOGD(MESH_TAG, "That was me, skipping!");
                continue;
            }
            ESP_LOGD(MESH_TAG, "Broadcast: Sending to [%d] " MACSTR, i, MAC2STR(s_route_table[i].addr));
            esp_err_t err = esp_mesh_send(&s_route_table[i], &data, MESH_DATA_P2P, NULL, 0);
            if (ESP_OK != err) {
                ESP_LOGE(TAG, "Send with err code %d %s", err, esp_err_to_name(err));
            }
        }
    } else {
        // Standard P2P
        esp_err_t err = esp_mesh_send(&dest_addr, &data, MESH_DATA_P2P, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Send with err code %d %s", err, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}
esp_err_t mesh_netif_transmit_from_root_ap_wrap(void *h, void *buffer, size_t len, void *netstack_buf)
{
    return mesh_netif_transmit_from_root_ap(h, buffer, len);
}
esp_err_t mesh_netif_transmit_from_node_sta_wrap(void *h, void *buffer, size_t len, void *netstack_buf)
{
    return mesh_netif_transmit_from_node_sta(h, buffer, len);
}
esp_err_t mesh_driver_start_root_ap(esp_netif_t * esp_netif, void * args)
{
    mesh_netif_driver_t driver = static_cast<mesh_netif_driver_t>(args);
    driver->base.netif = esp_netif;
    esp_netif_driver_ifconfig_t driver_ifconfig = {
            .handle =  driver,
            .transmit = mesh_netif_transmit_from_root_ap,
            .transmit_wrap = mesh_netif_transmit_from_root_ap_wrap,
            .driver_free_rx_buffer = mesh_free
    };

    return esp_netif_set_driver_config(esp_netif, &driver_ifconfig);
}
esp_err_t mesh_driver_start_node_sta(esp_netif_t * esp_netif, void * args)
{
    mesh_netif_driver_t driver = static_cast<mesh_netif_driver_t>(args);
    driver->base.netif = esp_netif;
    esp_netif_driver_ifconfig_t driver_ifconfig = {
            .handle =  driver,
            .transmit = mesh_netif_transmit_from_node_sta,
            .transmit_wrap = mesh_netif_transmit_from_node_sta_wrap,
            .driver_free_rx_buffer = mesh_free
    };

    return esp_netif_set_driver_config(esp_netif, &driver_ifconfig);
}
mesh_netif_driver_t ConnectivityManagerImpl::mesh_create_if_driver(bool is_ap, bool is_root)
{
    mesh_netif_driver_t driver = static_cast<mesh_netif_driver_t>(calloc(1, sizeof(struct mesh_netif_driver)));
    if (driver == NULL) {
        ESP_LOGE(MESH_TAG, "No memory to create a wifi interface handle");
        return NULL;
    }
    if (is_ap && is_root) {
        driver->base.post_attach = mesh_driver_start_root_ap;
    } else if (!is_ap && !is_root) {
        driver->base.post_attach = mesh_driver_start_node_sta;
    } else {
        return NULL;
    }
    // save station mac address to exclude it from routing-table on broadcast
    esp_wifi_get_mac(WIFI_IF_STA, driver->sta_mac_addr);

    return driver;
}
esp_err_t start_wifi_link_sta(void)
{
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    esp_err_t ret;
    void *driver = esp_netif_get_io_driver(netif_sta);
    if ((ret = esp_wifi_register_if_rxcb(static_cast<wifi_netif_driver_t>(driver),  esp_netif_receive, netif_sta)) != ESP_OK) {
        ESP_LOGE(MESH_TAG, "esp_wifi_register_if_rxcb for if=%p failed with %d", driver, ret);
        return ESP_FAIL;
    }
    esp_netif_set_mac(netif_sta, mac);
    esp_netif_action_start(netif_sta, NULL, 0, NULL);
    return ESP_OK;
}

void ConnectivityManagerImpl::mesh_netif_init_station(void)
{
    // By default create a station that would connect to AP (expecting root to connect to external network)
    esp_netif_config_t cfg_sta = ESP_NETIF_DEFAULT_WIFI_STA();
    netif_sta = esp_netif_new(&cfg_sta);
    assert(netif_sta);
    ESP_ERROR_CHECK(esp_netif_attach_wifi_station(netif_sta));
    ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());
}
esp_netif_t* create_mesh_link_sta(void)
{
    esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    base_cfg.if_desc = "mesh_link_sta";

    esp_netif_config_t cfg = {
        .base = &base_cfg,
        .driver = NULL,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA };
    esp_netif_t * netif = esp_netif_new(&cfg);
    assert(netif);
    return netif;
}
esp_err_t ConnectivityManagerImpl::mesh_netifs_start(bool is_root)
{
    if (is_root) {
        // ROOT: need both sta should use standard wifi, AP mesh link netif
        // Root: Station
        if (netif_sta && strcmp(esp_netif_get_desc(netif_sta), "sta") == 0) {
            ESP_LOGI(MESH_TAG, "Already wifi station, no need to do anything");
        } else if (netif_sta && strcmp(esp_netif_get_desc(netif_sta), "mesh_link_sta") == 0) {
            esp_netif_action_disconnected(netif_sta, NULL, 0, NULL);
            free(esp_netif_get_io_driver(netif_sta));
            esp_netif_destroy(netif_sta);
            mesh_netif_init_station();
        } else if (netif_sta == NULL) {
            mesh_netif_init_station();
        }
        // Root: AP is initialized only if GLOBAL DNS configured
        // (otherwise have to wait until the actual DNS record received from the router)
    } else {
        // NODE: create only STA in form of mesh link
        if (netif_sta && strcmp(esp_netif_get_desc(netif_sta), "mesh_link_sta") == 0) {
            ESP_LOGI(MESH_TAG, "Already mesh link station, no need to do anything");
            return ESP_OK;
        }
        if (netif_sta) {
            esp_netif_action_disconnected(netif_sta, NULL, 0, NULL);
            // should remove the actual driver
            if (strcmp(esp_netif_get_desc(netif_sta), "sta") == 0) {
                ESP_LOGI(MESH_TAG, "It was a wifi station removing stuff");
                esp_wifi_clear_default_wifi_driver_and_handlers(netif_sta);
            }
            esp_netif_destroy(netif_sta);
        }
        netif_sta = create_mesh_link_sta();
        // now we create a mesh driver and attach it to the existing netif
        mesh_netif_driver_t driver = mesh_create_if_driver(false, false);
        if (driver == NULL) {
            ESP_LOGE(TAG, "Failed to create wifi interface handle");
            return ESP_FAIL;
        }
        esp_netif_attach(netif_sta, driver);
        start_mesh_link_sta();
        // If we have a AP on NODE -> stop and remove it!
        if (netif_ap) {
            esp_netif_action_disconnected(netif_ap, NULL, 0, NULL);
            free(esp_netif_get_io_driver(netif_ap));
            esp_netif_destroy(netif_ap);
            netif_ap = NULL;
        }
    }
    return ESP_OK;
}

esp_err_t ConnectivityManagerImpl::mesh_netifs_stop(void)
{
    if (netif_sta && strcmp(esp_netif_get_desc(netif_sta), "sta") == 0 && netif_ap == NULL) {
        return ESP_OK;
    }

    if (netif_sta) {
        if (strcmp(esp_netif_get_desc(netif_sta), "sta") == 0) {
            esp_netif_action_disconnected(netif_sta, NULL, 0, NULL);
            esp_netif_action_stop(netif_sta, NULL, 0, NULL);
            esp_wifi_clear_default_wifi_driver_and_handlers(netif_sta);
        } else {
            esp_netif_action_disconnected(netif_sta, NULL, 0, NULL);
            free(esp_netif_get_io_driver(netif_sta));
        }
        esp_netif_destroy(netif_sta);
        netif_sta = NULL;
    }

    if (netif_ap) {
        esp_netif_action_disconnected(netif_ap, NULL, 0, NULL);
        free(esp_netif_get_io_driver(netif_ap));
        esp_netif_destroy(netif_ap);
        netif_ap = NULL;
    }
    // reserve the default (STA gets ready to become root)
    mesh_netif_init_station();
    start_wifi_link_sta();
    return ESP_OK;
}
#endif

CHIP_ERROR ConfigurationManagerImpl::_Init()
{
    CHIP_ERROR err;
    bool failSafeArmed;

    // Force initialization of NVS namespaces if they doesn't already exist.
    err = EnsureNamespace(kConfigNamespace_ChipFactory);
    SuccessOrExit(err);
    err = EnsureNamespace(kConfigNamespace_ChipConfig);
    SuccessOrExit(err);
    err = EnsureNamespace(kConfigNamespace_ChipCounters);
    SuccessOrExit(err);

    // Initialize the generic implementation base class.
    err = Internal::GenericConfigurationManagerImpl<ConfigurationManagerImpl>::_Init();
    SuccessOrExit(err);

    // TODO: Initialize the global GroupKeyStore object here (#1266)

#if CHIP_DEVICE_CONFIG_ENABLE_FACTORY_PROVISIONING

    {
        FactoryProvisioning factoryProv;
        uint8_t * const kInternalSRAM12Start = (uint8_t *) 0x3FFAE000;
        uint8_t * const kInternalSRAM12End   = kInternalSRAM12Start + (328 * 1024) - 1;

        // Scan ESP32 Internal SRAM regions 1 and 2 for injected provisioning data and save
        // to persistent storage if found.
        err = factoryProv.ProvisionDeviceFromRAM(kInternalSRAM12Start, kInternalSRAM12End);
        SuccessOrExit(err);
    }

#endif // CHIP_DEVICE_CONFIG_ENABLE_FACTORY_PROVISIONING

    // If the fail-safe was armed when the device last shutdown, initiate a factory reset.
    if (_GetFailSafeArmed(failSafeArmed) == CHIP_NO_ERROR && failSafeArmed)
    {
        ChipLogProgress(DeviceLayer, "Detected fail-safe armed on reboot; initiating factory reset");
        _InitiateFactoryReset();
    }
    err = CHIP_NO_ERROR;

exit:
    return err;
}

CHIP_ERROR ConfigurationManagerImpl::_GetPrimaryWiFiMACAddress(uint8_t * buf)
{
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    if ((mode == WIFI_MODE_AP) || (mode == WIFI_MODE_APSTA))
        return MapConfigError(esp_wifi_get_mac(WIFI_IF_AP, buf));
    else
        return MapConfigError(esp_wifi_get_mac(WIFI_IF_STA, buf));
}

CHIP_ERROR ConfigurationManagerImpl::MapConfigError(esp_err_t error)
{
    switch (error)
    {
    case ESP_OK:
        return CHIP_NO_ERROR;
    case ESP_ERR_WIFI_NOT_INIT:
        return CHIP_ERROR_WELL_UNINITIALIZED;
    case ESP_ERR_INVALID_ARG:
    case ESP_ERR_WIFI_IF:
        return CHIP_ERROR_INVALID_ARGUMENT;
    default:
        return CHIP_ERROR_INTERNAL;
    }
}

bool ConfigurationManagerImpl::_CanFactoryReset()
{
    // TODO: query the application to determine if factory reset is allowed.
    return true;
}

void ConfigurationManagerImpl::_InitiateFactoryReset()
{
    PlatformMgr().ScheduleWork(DoFactoryReset);
}

CHIP_ERROR ConfigurationManagerImpl::_ReadPersistedStorageValue(::chip::Platform::PersistedStorage::Key key, uint32_t & value)
{
    ESP32Config::Key configKey{ kConfigNamespace_ChipCounters, key };

    CHIP_ERROR err = ReadConfigValue(configKey, value);
    if (err == CHIP_DEVICE_ERROR_CONFIG_NOT_FOUND)
    {
        err = CHIP_ERROR_PERSISTED_STORAGE_VALUE_NOT_FOUND;
    }
    return err;
}

CHIP_ERROR ConfigurationManagerImpl::_WritePersistedStorageValue(::chip::Platform::PersistedStorage::Key key, uint32_t value)
{
    ESP32Config::Key configKey{ kConfigNamespace_ChipCounters, key };
    return WriteConfigValue(configKey, value);
}

void ConfigurationManagerImpl::DoFactoryReset(intptr_t arg)
{
    CHIP_ERROR err;

    ChipLogProgress(DeviceLayer, "Performing factory reset");

    // Erase all values in the chip-config NVS namespace.
    err = ClearNamespace(kConfigNamespace_ChipConfig);
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "ClearNamespace(ChipConfig) failed: %s", chip::ErrorStr(err));
    }

    // Restore WiFi persistent settings to default values.
    err = esp_wifi_restore();
    if (err != ESP_OK)
    {
        ChipLogError(DeviceLayer, "esp_wifi_restore() failed: %s", chip::ErrorStr(err));
    }

    // Restart the system.
    ChipLogProgress(DeviceLayer, "System restarting");
    esp_restart();
}

} // namespace DeviceLayer
} // namespace chip
