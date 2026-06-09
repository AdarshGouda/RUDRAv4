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

The manual is chapter-split, uses TikZ/PGFPlots for diagrams and plots, and includes selected source/config snapshots from the RUDRAv4 workspace.

Recommended Overleaf setting: Menu -> Compiler -> XeLaTeX.


Revision note: this updated package adds fresh Ubuntu setup for `rudra` and `aghora`, dedicated hardware chapters for the NUC7i7BNH, Mini-Box DCDC-USB, Sabertooth drivers, Teensy/Uno/PS2, MPU6050, customized YDLIDAR stack, robot body, batteries, and BOM.
