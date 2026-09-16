# 蓝牙（BLE）相关模块划分：
#   ble_gap.c    —— BLE GAP 事件统一分发（协议栈只允许注册一个 GAP 回调，避免被顶掉）
#   ble_client.c —— **BLE 主机（客户端）**：扫描并连接手表 OV_WATCH（服务 0xFFF0 / 特征 0xFFF1）
#
# 已停用（源码保留备查，需要时把下面 SRCS 里对应的注释去掉即可；不要同时启用会冲突的模块）：
#   ble_debug.c  —— 调试阶段代码：连上后定期发 $PING + 打印收到的帧（详见文件顶部说明）
#   ble.c        —— 早期 BLE 服务端+客户端示例实现（已废弃）
#   ble_server.c —— 早前"音箱作 BLE 从机"的反向方案（当前设计是音箱只作主机）
idf_component_register(SRCS "amp.c"
                            "audio_vol.c"
                            "ble_gap.c"
                            "ble_client.c"
                            # "ble_debug.c"      # 调试阶段代码，已停用
                            "adc.c"
                            "sd.c"
                            "uart.c"
                            "led_strip.c"
                            "bt_app_av.c"
                            "bt_app_core.c"
                            "bt_a2dp.c"
                            "main.c"
                    PRIV_REQUIRES esp_driver_i2s bt nvs_flash esp_ringbuf esp_driver_dac driver esp_adc fatfs sdmmc
                    INCLUDE_DIRS ".")
