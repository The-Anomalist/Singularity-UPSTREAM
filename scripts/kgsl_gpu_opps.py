#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Rewrite Kona KGSL GPU OPP and pwrlevel tables.

The KGSL power table stores active GPU frequencies plus one trailing sleep
entry. This helper accepts up to 15 active frequencies and appends the sleep
level automatically.
"""

import argparse
import re
from pathlib import Path


MAX_ACTIVE_OPPS = 15
DEFAULT_DTSI = "arch/arm64/boot/dts/vendor/qcom/kona-v2-gpu.dtsi"
DEFAULT_LABEL = "gpu_opp_table_v2"


def parse_rate(value):
    text = value.strip().lower().replace("_", "")
    mult = 1
    for suffix, suffix_mult in (("mhz", 1000000), ("m", 1000000),
                                ("khz", 1000), ("k", 1000), ("hz", 1)):
        if text.endswith(suffix):
            text = text[:-len(suffix)]
            mult = suffix_mult
            break

    rate = float(text) * mult
    if rate <= 0:
        raise argparse.ArgumentTypeError(f"invalid non-positive rate: {value}")

    return int(rate)


def parse_csv_rates(value):
    rates = [parse_rate(item) for item in value.split(",") if item.strip()]
    rates = sorted(set(rates), reverse=True)

    if not rates:
        raise argparse.ArgumentTypeError("at least one frequency is required")
    if len(rates) > MAX_ACTIVE_OPPS:
        raise argparse.ArgumentTypeError(
            f"too many active OPPs: {len(rates)} > {MAX_ACTIVE_OPPS}")

    return rates


def split_csv(value):
    items = [item.strip() for item in value.split(",") if item.strip()]
    if not items:
        raise argparse.ArgumentTypeError("empty list")
    return items


def parse_csv_ints(value):
    items = []
    for item in value.split(","):
        item = item.strip()
        if item:
            items.append(int(item, 0))
    if not items:
        raise argparse.ArgumentTypeError("empty list")
    return items


def find_matching_brace(text, open_index):
    depth = 0
    for index in range(open_index, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return index
    raise ValueError("unmatched brace")


def find_labeled_block(text, label):
    match = re.search(rf"(^[ \t]*){re.escape(label)}\s*:\s*[-\w]+\s*{{",
                      text, re.M)
    if not match:
        raise ValueError(f"unable to find OPP table label {label!r}")

    open_index = text.index("{", match.end() - 1)
    close_index = find_matching_brace(text, open_index)
    end = text.find(";", close_index)
    if end < 0:
        raise ValueError(f"unable to find block terminator for {label!r}")

    return match.start(), end + 1, text[match.start():end + 1], match.group(1)


def parse_existing_opp_voltages(block):
    opps = []
    pattern = re.compile(
        r"opp-(\d+)\s*{[^{}]*?opp-hz\s*=\s*/bits/\s*64\s*<(?P<hz>\d+)>;"
        r"[^{}]*?opp-microvolt\s*=\s*<(?P<uv>[^>]+)>;",
        re.S)

    for match in pattern.finditer(block):
        opps.append((int(match.group("hz")), match.group("uv").strip()))

    return opps


def nearest_voltage(freq, existing):
    if not existing:
        raise ValueError("no existing OPP voltages found; pass --voltages")
    return min(existing, key=lambda item: abs(item[0] - freq))[1]


def build_opp_table(label, node_name, freqs, voltages, indent):
    lines = [f"{indent}{label}: {node_name} {{", f"{indent}\tcompatible = \"operating-points-v2\";"]

    for freq, voltage in zip(freqs, voltages):
        lines.extend([
            "",
            f"{indent}\topp-{freq} {{",
            f"{indent}\t\topp-hz = /bits/ 64 <{freq}>;",
            f"{indent}\t\topp-microvolt = <{voltage}>;",
            f"{indent}\t}};",
        ])

    lines.append(f"{indent}}};")
    return "\n".join(lines)


def replace_opp_table(text, label, freqs, voltage_args):
    start, end, block, indent = find_labeled_block(text, label)
    header = re.search(rf"{re.escape(label)}\s*:\s*(?P<node>[-\w]+)\s*{{", block)
    node_name = header.group("node")

    if voltage_args is not None:
        if len(voltage_args) != len(freqs):
            raise ValueError("--voltages count must match --freqs count")
        voltages = voltage_args
    else:
        existing = parse_existing_opp_voltages(block)
        voltages = [nearest_voltage(freq, existing) for freq in freqs]

    return text[:start] + build_opp_table(label, node_name, freqs, voltages, indent) + text[end:]


def find_pwrlevel_group_blocks(text):
    pattern = re.compile(
        r"(^[ \t]*)qcom,gpu-pwrlevels(?:-(?P<idx>\d+))?\s*{", re.M)
    for match in pattern.finditer(text):
        open_index = text.index("{", match.end() - 1)
        close_index = find_matching_brace(text, open_index)
        end = text.find(";", close_index)
        if end < 0:
            raise ValueError("unable to find pwrlevel group terminator")
        group_index = match.group("idx") if match.group("idx") is not None else "0"
        yield match.start(), end + 1, text[match.start():end + 1], match.group(1), group_index


def parse_pwrlevels(block):
    levels = []
    pattern = re.compile(r"(^[ \t]*)qcom,gpu-pwrlevel@(?P<idx>\d+)\s*{", re.M)

    for match in pattern.finditer(block):
        open_index = block.index("{", match.end() - 1)
        close_index = find_matching_brace(block, open_index)
        end = block.find(";", close_index)
        child = block[match.start():end + 1]
        freq_match = re.search(r"qcom,gpu-freq\s*=\s*<(?P<freq>\d+)>;", child)
        if not freq_match:
            continue
        levels.append({
            "freq": int(freq_match.group("freq")),
            "body": child,
        })

    return levels


def extract_properties(level_body, drop_acd=False):
    props = []
    for line in level_body.splitlines():
        stripped = line.strip()
        if drop_acd and stripped.startswith("qcom,acd-level"):
            continue
        if stripped.startswith("qcom,bus-") or stripped.startswith("qcom,acd-level"):
            props.append(stripped)
    return props


def has_ddr_bus_props(props):
    return any(prop.startswith("qcom,bus-freq-ddr") for prop in props)


def replace_bus_props(props, bus_level):
    if bus_level is None:
        return props

    acd_props = [prop for prop in props if prop.startswith("qcom,acd-level")]

    if has_ddr_bus_props(props):
        return [
            f"qcom,bus-freq-ddr7 = <{bus_level}>;",
            f"qcom,bus-min-ddr7 = <{bus_level}>;",
            f"qcom,bus-max-ddr7 = <{bus_level}>;",
            "",
            f"qcom,bus-freq-ddr8 = <{bus_level}>;",
            f"qcom,bus-min-ddr8 = <{bus_level}>;",
            f"qcom,bus-max-ddr8 = <{bus_level}>;",
            *acd_props,
        ]

    return [
        f"qcom,bus-freq = <{bus_level}>;",
        f"qcom,bus-min = <{bus_level}>;",
        f"qcom,bus-max = <{bus_level}>;",
        *acd_props,
    ]


def nearest_level(freq, levels):
    active = [level for level in levels if level["freq"] > 0]
    if not active:
        raise ValueError("no active pwrlevels found to use as templates")
    return min(active, key=lambda level: abs(level["freq"] - freq))


def build_pwrlevel_child(index, freq, props, indent):
    lines = [
        f"{indent}\tqcom,gpu-pwrlevel@{index} {{",
        f"{indent}\t\treg = <{index}>;",
        f"{indent}\t\tqcom,gpu-freq = <{freq}>;",
    ]

    for prop in props:
        if prop:
            lines.append(f"{indent}\t\t{prop}")
        else:
            lines.append("")

    lines.append(f"{indent}\t}};")
    return "\n".join(lines)


def build_sleep_child(index, props, indent):
    lines = [
        f"{indent}\tqcom,gpu-pwrlevel@{index} {{",
        f"{indent}\t\treg = <{index}>;",
        f"{indent}\t\tqcom,gpu-freq = <0>;",
    ]

    for prop in props:
        lines.append(f"{indent}\t\t{prop}")

    lines.append(f"{indent}\t}};")
    return "\n".join(lines)


def replace_group_pwrlevels(group, freqs, initial, bus_levels, drop_acd):
    levels = parse_pwrlevels(group)
    first = re.search(r"(^[ \t]*)qcom,gpu-pwrlevel@\d+\s*{", group, re.M)
    if not first:
        return group

    child_indent = first.group(1)
    prefix = group[:first.start()].rstrip()
    suffix_start = max(match.end() for match in re.finditer(
        r"(^[ \t]*)qcom,gpu-pwrlevel@\d+\s*{", group, re.M))
    last_open = group.rfind("{", 0, suffix_start)
    last_close = find_matching_brace(group, last_open)
    last_end = group.find(";", last_close) + 1
    suffix = group[last_end:].lstrip("\n")

    if initial is not None and "qcom,speed-bin" in prefix:
        replacement = f"qcom,initial-pwrlevel = <{initial}>;"
        if re.search(r"qcom,initial-pwrlevel\s*=\s*<\d+>;", prefix):
            prefix = re.sub(r"qcom,initial-pwrlevel\s*=\s*<\d+>;", replacement, prefix)
        else:
            prefix += f"\n{child_indent}{replacement}"

    children = []
    for index, freq in enumerate(freqs):
        props = extract_properties(nearest_level(freq, levels)["body"], drop_acd)
        props = replace_bus_props(props,
                                  bus_levels[index] if bus_levels else None)
        children.append(build_pwrlevel_child(index, freq, props, child_indent))

    sleep = next((level for level in levels if level["freq"] == 0), None)
    sleep_props = extract_properties(sleep["body"], drop_acd) if sleep else [
        "qcom,bus-freq = <0>;", "qcom,bus-min = <0>;", "qcom,bus-max = <0>;"
    ]
    children.append(build_sleep_child(len(freqs), sleep_props, child_indent))
    return prefix + "\n\n" + "\n\n".join(children) + "\n" + suffix


def replace_pwrlevel_groups(text, freqs, bins, initial, bus_levels, drop_acd):
    result = []
    cursor = 0
    changed = 0

    for start, end, block, _indent, group_index in find_pwrlevel_group_blocks(text):
        result.append(text[cursor:start])
        if bins is None or group_index in bins:
            result.append(replace_group_pwrlevels(block, freqs, initial,
                                                  bus_levels, drop_acd))
            changed += 1
        else:
            result.append(block)
        cursor = end

    result.append(text[cursor:])

    if changed == 0:
        raise ValueError("no qcom,gpu-pwrlevels groups matched")

    return "".join(result), changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dtsi", default=DEFAULT_DTSI,
                        help=f"DTSI to patch, default: {DEFAULT_DTSI}")
    parser.add_argument("--opp-label", default=DEFAULT_LABEL,
                        help=f"OPP table label, default: {DEFAULT_LABEL}")
    parser.add_argument("--freqs", required=True, type=parse_csv_rates,
                        help="Comma-separated active GPU freqs, e.g. 900m,850m,800m")
    parser.add_argument("--voltages", type=split_csv,
                        help="Comma-separated OPP voltage labels/values. Defaults to nearest existing OPP voltage.")
    parser.add_argument("--bins", type=split_csv,
                        help="Comma-separated qcom,gpu-pwrlevels-N groups to patch. Defaults to all.")
    parser.add_argument("--initial-pwrlevel", type=int,
                        help="Optional qcom,initial-pwrlevel value to set in patched bins.")
    parser.add_argument("--bus-levels", type=parse_csv_ints,
                        help="Comma-separated bus level for each active frequency.")
    parser.add_argument("--drop-acd", action="store_true",
                        help="Do not copy qcom,acd-level properties into generated pwrlevels.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Validate and print a summary without writing.")
    args = parser.parse_args()

    path = Path(args.dtsi)
    text = path.read_text()
    bins = set(args.bins) if args.bins is not None else None

    if args.bus_levels is not None and len(args.bus_levels) != len(args.freqs):
        raise ValueError("--bus-levels count must match --freqs count")

    text = replace_opp_table(text, args.opp_label, args.freqs, args.voltages)
    text, changed = replace_pwrlevel_groups(text, args.freqs, bins,
                                            args.initial_pwrlevel,
                                            args.bus_levels, args.drop_acd)

    if args.dry_run:
        print(f"validated {len(args.freqs)} active OPPs + sleep across {changed} pwrlevel group(s)")
        return

    path.write_text(text)
    print(f"wrote {path}: {len(args.freqs)} active OPPs + sleep across {changed} pwrlevel group(s)")


if __name__ == "__main__":
    main()
