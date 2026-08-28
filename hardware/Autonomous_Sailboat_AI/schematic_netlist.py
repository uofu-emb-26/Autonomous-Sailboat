#!/usr/bin/env python3
"""Build a full ref-designator/pin -> net mapping for Autonomous_Sailboat.kicad_sch.

Generalizes schematic_pinmap.py to every placed symbol (not just the MCU):
resolves each symbol instance's pin absolute positions from its lib_symbols
definition + placement transform, then unions them with wires/junctions/
labels/no_connects that share a coordinate to form nets.

Usage: python3 schematic_netlist.py            # print every net
       python3 schematic_netlist.py B1 B2 Q1   # print only nets touching these refs
"""
import math
import os
import sys

SCH_PATH = os.path.join(
    os.path.dirname(__file__), "..", "Autonomous_Sailboat", "Autonomous_Sailboat.kicad_sch"
)


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

    # --- library pin geometry per lib_id: name -> [(number, name, (x,y))] ---
    lib_symbols = find_all(root, "lib_symbols")[0]
    lib_pins = {}  # lib_id -> list[(number, name, (x, y))]
    for sym in lib_symbols[1:]:
        if not (isinstance(sym, list) and sym[0] == "symbol"):
            continue
        lib_id = sym[1]
        pins = []
        for p in find_all(sym, "pin"):
            at = get_clause(p, "at")
            name_c = get_clause(p, "name")
            number_c = get_clause(p, "number")
            if at is None or name_c is None or number_c is None:
                continue
            pins.append((number_c[1], name_c[1], (float(at[1]), float(at[2]))))
        lib_pins[lib_id] = pins

    # --- placed instances (top-level symbols, i.e. not inside lib_symbols) ---
    instances = []
    for s in root:
        if not (isinstance(s, list) and s and s[0] == "symbol"):
            continue
        lib_id_c = get_clause(s, "lib_id")
        at_c = get_clause(s, "at")
        if lib_id_c is None or at_c is None:
            continue
        ref = None
        for prop in find_all(s, "property"):
            if prop[1] == "Reference":
                ref = prop[2]
        mirror_c = get_clause(s, "mirror")
        instances.append({
            "lib_id": lib_id_c[1],
            "ref": ref or "?",
            "ox": float(at_c[1]),
            "oy": float(at_c[2]),
            "rot": float(at_c[3]) if len(at_c) > 3 else 0.0,
            "mirror": mirror_c[1] if mirror_c else None,
        })

    def abs_pos(ox, oy, rot, mirror, lx, ly):
        x, y = lx, -ly  # symbol-editor Y-up -> sheet Y-down
        if mirror == "x":
            y = -y
        elif mirror == "y":
            x = -x
        a = math.radians(rot)
        ax = x * math.cos(a) - y * math.sin(a)
        ay = x * math.sin(a) + y * math.cos(a)
        return (round(ox + ax, 2), round(oy + ay, 2))

    # ref/pin -> absolute position
    pin_points = {}  # (ref, number) -> (x,y)
    pin_name_lookup = {}  # (ref, number) -> name
    for inst in instances:
        pins = lib_pins.get(inst["lib_id"], [])
        for number, name, (lx, ly) in pins:
            pos = abs_pos(inst["ox"], inst["oy"], inst["rot"], inst["mirror"], lx, ly)
            pin_points[(inst["ref"], number)] = pos
            pin_name_lookup[(inst["ref"], number)] = name

    # --- wires -> union-find over coordinates ---
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

    for w in find_all(root, "wire"):
        pts_c = get_clause(w, "pts")
        pts = [(round(float(p[1]), 2), round(float(p[2]), 2)) for p in pts_c[1:]]
        for a, b in zip(pts, pts[1:]):
            union(a, b)
        for p in pts:
            find(p)

    labels = []
    for tag in ("label", "global_label", "hierarchical_label"):
        for l in find_all(root, tag):
            at_l = get_clause(l, "at")
            labels.append((l[1], (round(float(at_l[1]), 2), round(float(at_l[2]), 2))))

    no_connects = set()
    for nc in find_all(root, "no_connect"):
        at_nc = get_clause(nc, "at")
        no_connects.add((round(float(at_nc[1]), 2), round(float(at_nc[2]), 2)))

    # union pin points into the wire graph too (register them as nodes)
    for pos in pin_points.values():
        find(pos)

    label_by_root = {}
    for name, pos in labels:
        find(pos)
        label_by_root.setdefault(find(pos), []).append(name)

    # --- build net groups: root -> {"labels": [...], "pins": [(ref,num,name)], "no_connect": bool} ---
    nets = {}
    for (ref, number), pos in pin_points.items():
        r = find(pos)
        net = nets.setdefault(r, {"labels": [], "pins": [], "no_connect": pos in no_connects})
        net["pins"].append((ref, number, pin_name_lookup[(ref, number)]))
    for root_pt, names in label_by_root.items():
        net = nets.setdefault(root_pt, {"labels": [], "pins": [], "no_connect": False})
        net["labels"].extend(names)

    filter_refs = set(sys.argv[1:])

    for root_pt, net in nets.items():
        if not net["pins"] and not net["labels"]:
            continue
        if filter_refs and not (filter_refs & {p[0] for p in net["pins"]}):
            continue
        label_str = ", ".join(sorted(set(net["labels"]))) if net["labels"] else "(unlabeled)"
        nc = " [NO_CONNECT]" if net["no_connect"] else ""
        pin_str = ", ".join(f"{ref}.{num}({name})" for ref, num, name in sorted(net["pins"]))
        print(f"{label_str}{nc}: {pin_str}")


if __name__ == "__main__":
    main()
