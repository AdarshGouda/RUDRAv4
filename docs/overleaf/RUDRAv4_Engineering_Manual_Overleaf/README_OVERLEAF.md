# RUDRAv4 Engineering Development Manual - Overleaf Package

Upload this folder or the ZIP file to Overleaf and set the compiler to **XeLaTeX**.

Main file:

```text
main.tex
```

Structure:

```text
main.tex
manual.sty
chapters/*.tex
appendices/*.tex
figures/photos/*.png
code/*
```

The manual is chapter-split, uses TikZ/PGFPlots for diagrams and plots, and includes selected source/config snapshots from the current RUDRAv4 workspace.

Recommended Overleaf setting: Menu -> Compiler -> XeLaTeX.


Revision note: this package includes Ubuntu setup for `rudra` and `aghora`, dedicated hardware chapters for the NUC7i7BNH, Mini-Box DCDC-USB, Sabertooth drivers, Teensy/Uno/PS2, MPU6050, customized YDLIDAR stack, robot body, batteries, BOM, current launch operations, localization/TF theory, and the staged navigation/autonomy roadmap.

When the workspace changes, regenerate the `code/` snapshots from the live repository before exporting the Overleaf package. The live repository remains authoritative; the snapshots are for review, traceability, and offline reading.
