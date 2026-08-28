#!/usr/bin/env python3
"""Resolve MCU pin numbers to net labels in Autonomous_Sailboat.kicad_sch.

KiCad schematic files are S-expressions; there is no direct pin->net table,
so we compute each MCU pin's absolute sheet position from the symbol's
library pin offsets + placement transform, then trace wires/junctions to
find which net label (if any) reaches that point.
"""
import math
import os

SCH_PATH = os.path.join(
    os.path.dirname(__file__), "..", "Autonomous_Sailboat", "Autonomous_Sailboat.kicad_sch"
)
MCU_LIB_ID = "MCU_ST_STM32H7:STM32H755ZITx"


def parse(s, i=0):
    items = []
    while i < len(s):
        c = s[i]
        if c.isspace():
            i += 1
            continue
        if c == "(":
            sub, i = parse(s, i + 1)
            items.append(sub)
        elif c == ")":
            return items, i + 1
        elif c == '"':
            j = s.index('"', i + 1)
            items.append(s[i + 1 : j])
            i = j + 1
        else:
            j = i
            while j < len(s) and not s[j].isspace() and s[j] not in "()":
                j += 1
            items.append(s[i:j])
            i = j
    return items, i


def find_all(node, tag, out=None):
    if out is None:
        out = []
    if isinstance(node, list):
        if node and node[0] == tag:
            out.append(node)
        for c in node:
            find_all(c, tag, out)
    return out


def get_clause(node, tag):
    for item in node:
        if isinstance(item, list) and item and item[0] == tag:
            return item
    return None


def main():
    text = open(SCH_PATH).read()
    tree, _ = parse(text)
    root = tree[0]

    lib_symbols = find_all(root, "lib_symbols")[0]
    mcu_symbol = None
    for s in lib_symbols[1:]:
        if isinstance(s, list) and s[0] == "symbol" and s[1] == MCU_LIB_ID:
            mcu_symbol = s
            break
    if mcu_symbol is None:
        raise SystemExit(f"Could not find lib symbol {MCU_LIB_ID}")

    pin_info = []  # (number, name, (x, y))
    for p in find_all(mcu_symbol, "pin"):
        at = get_clause(p, "at")
        name_c = get_clause(p, "name")
        number_c = get_clause(p, "number")
        if at is None or name_c is None or number_c is None:
            continue
        pin_info.append((number_c[1], name_c[1], (float(at[1]), float(at[2]))))

    # placed MCU instance (top-level symbol with matching lib_id)
    mcu_instance = None
    for s in root:
        if isinstance(s, list) and s and s[0] == "symbol":
            lib_id_c = get_clause(s, "lib_id")
            if lib_id_c and lib_id_c[1] == MCU_LIB_ID:
                mcu_instance = s
                break
    if mcu_instance is None:
        raise SystemExit("MCU not placed on sheet yet")

    at_c = get_clause(mcu_instance, "at")
    ox, oy = float(at_c[1]), float(at_c[2])
    rot = float(at_c[3]) if len(at_c) > 3 else 0.0
    mirror_c = get_clause(mcu_instance, "mirror")
    mirror = mirror_c[1] if mirror_c else None

    def abs_pos(lx, ly):
        # KiCad symbol-editor local coords are Y-up; sheet coords are Y-down,
        # so placement at rotation 0 / no mirror requires negating local Y.
        x, y = lx, -ly
        if mirror == "x":
            y = -y
        elif mirror == "y":
            x = -x
        a = math.radians(rot)
        ax = x * math.cos(a) - y * math.sin(a)
        ay = x * math.sin(a) + y * math.cos(a)
        return (round(ox + ax, 2), round(oy + ay, 2))

    # wires -> build adjacency of points, union-find style via shared coords
    wire_segs = []
    for w in find_all(root, "wire"):
        pts_c = get_clause(w, "pts")
        coords = [(float(p[1]), float(p[2])) for p in pts_c[1:]]
        wire_segs.append(coords)

    labels = []
    for tag in ("label", "global_label", "hierarchical_label"):
        for l in find_all(root, tag):
            at_l = get_clause(l, "at")
            labels.append((l[1], (round(float(at_l[1]), 2), round(float(at_l[2]), 2))))

    # union-find over points connected by wire segments
    parent = {}

    def find(p):
        parent.setdefault(p, p)
        while parent[p] != p:
            parent[p] = parent[parent[p]]
            p = parent[p]
        return p

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for seg in wire_segs:
        pts = [(round(x, 2), round(y, 2)) for x, y in seg]
        for a, b in zip(pts, pts[1:]):
            union(a, b)
        for p in pts:
            find(p)

    label_by_root = {}
    for name, pos in labels:
        find(pos)
        label_by_root.setdefault(find(pos), []).append(name)

    no_connects = set()
    for nc in find_all(root, "no_connect"):
        at_nc = get_clause(nc, "at")
        no_connects.add((round(float(at_nc[1]), 2), round(float(at_nc[2]), 2)))

    print(f"{'PIN#':<6}{'NAME':<12}{'NET LABEL'}")
    connected = []
    no_connect_pins = []
    unconnected = []
    for number, name, at in pin_info:
        pos = abs_pos(*at)
        if pos in no_connects:
            no_connect_pins.append((number, name))
            continue
        net_names = label_by_root.get(find(pos), []) if pos in parent else []
        if net_names:
            connected.append((number, name, ", ".join(sorted(set(net_names)))))
        else:
            unconnected.append((number, name))

    for number, name, net in sorted(connected, key=lambda t: int(t[0])):
        print(f"{number:<6}{name:<12}{net}")

    print(f"\n# {len(connected)} pins connected to a net label, "
          f"{len(no_connect_pins)} explicitly no-connect, {len(unconnected)} unlabeled")


if __name__ == "__main__":
    main()
