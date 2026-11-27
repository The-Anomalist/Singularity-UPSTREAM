#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Smoke-test the upstream Android common MGLRU sysfs ABI.  This intentionally
# does not alter the enabled mask: list migration and reclaim behavior need
# device-level stress testing, not a host selftest toggle.

KSFT_SKIP=4
LRU_GEN=/sys/kernel/mm/lru_gen

if [ "$(id -u)" -ne 0 ]; then
	echo "SKIP: root is required to read the MGLRU control ABI"
	exit "$KSFT_SKIP"
fi

if [ ! -d "$LRU_GEN" ]; then
	echo "SKIP: CONFIG_LRU_GEN is unavailable"
	exit "$KSFT_SKIP"
fi

for control in enabled min_ttl_ms; do
	if [ ! -r "$LRU_GEN/$control" ]; then
		echo "FAIL: missing readable $LRU_GEN/$control"
		exit 1
	fi
done

enabled=$(cat "$LRU_GEN/enabled") || exit 1
min_ttl_ms=$(cat "$LRU_GEN/min_ttl_ms") || exit 1

case "$enabled" in
	0x[0-9a-fA-F]*) ;;
	*)
		echo "FAIL: unexpected enabled value: $enabled"
		exit 1
		;;
esac

case "$min_ttl_ms" in
	''|*[!0-9]*)
		echo "FAIL: unexpected min_ttl_ms value: $min_ttl_ms"
		exit 1
		;;
esac

echo "PASS: MGLRU sysfs ABI is present (enabled=$enabled min_ttl_ms=$min_ttl_ms)"
