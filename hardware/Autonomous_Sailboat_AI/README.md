# Autonomous_Sailboat_AI

Scratch scripts and reference files used to analyze/query the KiCad schematic
in `../Autonomous_Sailboat/`. Not part of the KiCad project itself.

- `schematic_pinmap.py` - parses `Autonomous_Sailboat.kicad_sch`, resolves the
  MCU symbol's pin positions against wires/labels in the sheet, and prints a
  pin-number -> net-name mapping plus a list of unconnected pins. Run with:

  ```
  python3 schematic_pinmap.py
  ```
