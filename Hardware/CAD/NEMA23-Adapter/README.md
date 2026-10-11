# Tercio S1 · NEMA 23 adapter plate

A compact cap that lets the Tercio S1 close the loop on a NEMA 23. It replaces the S1's NEMA 17 cover and follows the outline of the motor's octagonal rear end cap, 0.2 mm inside it all round, so nothing sticks out past the motor.

- The S1 PCB drops into a pocket and screws onto four bosses at its own mounting holes, which puts the AS5600 exactly on the motor axis.
- Four M3 screws hold the plate in the motor's own rear screw holes.
- The magnet is glued to the flush shaft end, and the plate sets a **1.0 mm air gap** to the AS5600 (datasheet range 0.5–3 mm).

```bash
node serve.mjs          # 3D viewer at http://localhost:8766
```

## Files

| File | |
|---|---|
| `step/Tercio-S1_NEMA23-Adapter.step` | The plate |
| `step/Tercio-S1_NEMA23-Assembly.step` | Motor, plate, magnet, screws and Tercio S1 (from `../Tercio-S1.step`), in place |
| `adapter.py` | The parametric source (CadQuery). It writes the STEP files and the viewer's meshes |
| `viewer/` | The web viewer: three.js with section view, explode and dimensions |

To regenerate after changing a parameter:

```bash
uv venv -p 3.12 .venv && uv pip install --python .venv/bin/python cadquery trimesh shapely
.venv/bin/python adapter.py
```

The build fails if the air gap leaves 0.5–3 mm, if a bottom-side part would touch the motor, or if the wall around a motor screw drops under 1.5 mm.

## Key dimensions

Z is measured from the motor's rear face.

| | |
|---|---|
| Outline | 56.0 mm octagon, corners at \|x\| + \|y\| ≤ 42.8 mm (the motor's rear end cap less 0.2 mm) |
| Thickness | 6.34 mm |
| Motor screws | 4 × M3 clearance (Ø3.4) on 47.14 × 29.0 mm |
| S1 mounting | 4 × M3 (tap drill Ø2.5) in bosses at the PCB's own holes |
| PCB underside | Z = 5.34 mm; the pocket rim holds 1 mm of the board's edge |
| Magnet top | Z = 2.50 mm |
| AS5600 package underside | Z = 3.50 mm, so the air gap is 1.00 mm |
| Bottom-side parts to motor | 2.55 mm |
| Thinnest wall | 1.64 mm, around a motor screw |
| Mass | ≈ 23 g in 6061-T6 |

The S1 sits turned 90° in the plate, which puts both side-entry connectors on sides clear of the motor screws:
- J5 (JST PH) faces the motor's lead side, with a channel out to the edge for its plug;
- J2 (PHD) faces +X, above the plate top.

## Parts list

- Plate: 6061-T6, bead-blasted or black anodised; or printed in PETG / PA-CF (tap the M3 holes or use heat-set inserts).
- Magnet: Ø6 × 2.5 mm, diametrically magnetised, N35.
- 4 × M3 × 10 button head: plate to motor (3.7 mm of thread engagement).
- 4 × M3 × 6 socket head: S1 to plate.

## Assembly

1. **Set the magnet.** Clean the rear shaft end and glue the magnet centred on it with cyanoacrylate or epoxy.
2. **Fit the S1.** Take it off its NEMA 17 cover and screw it into the plate with 4 × M3 × 6.
3. **Fit the plate.** Set it on the motor's rear face with the J5 channel on the motor's lead side, and screw it down with 4 × M3 × 10.

## Reference motor

A 57 mm NEMA 23 per the supplied drawing:
- 56.4 mm max frame;
- 47.14 ± 0.2 mm mounting holes, Ø5.1;
- pilot Ø38.1 × 1.6 mm;
- shaft Ø8, 5 mm flange, 56 mm body.

The shaft end is flush with the rear face; there is no rear shaft extension.

**Check the rear screw holes on your motor before cutting metal.** The pattern differs between makers. The model assumes M3 threads on 47.14 × 29.0 mm, where the motor's assembly screws come through the rear end cap.

To change it, edit `MOTOR_SCREWS` (and `MOTOR_SCREW` / `PLATE_SCREW_L`) in `adapter.py` and re-run. The wall check tells you if the new pattern no longer fits around the pocket.

## Note on the S1 hole pattern

The S1 PCB's mounting holes (KiCad H1–H4) sit on a **30.6 mm** square around the AS5600. The S1's NEMA 17 cover has its holes on **31.0 mm**, centred 0.2 mm off the AS5600.

This plate uses the PCB's own holes, so the encoder lands on the motor axis regardless.
