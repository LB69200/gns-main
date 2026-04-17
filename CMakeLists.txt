# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE "${CMAKE_SOURCE_DIR}/boards/nrf54l15dk_nrf54l10_cpuapp_ns.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(echo_bot)

target_sources(app PRIVATE src/main.c)
