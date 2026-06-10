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
```

The manual is chapter-split and uses TikZ/PGFPlots for diagrams and plots. It intentionally does not duplicate implementation code from `src/`; the RUDRAv4 GitHub repository is the source of truth for code, firmware, launch files, configuration, and scripts.

Recommended Overleaf setting: Menu -> Compiler -> XeLaTeX.


Revision note: this package includes Ubuntu setup for `rudra` and `aghora`, dedicated hardware chapters for the NUC7i7BNH, Mini-Box DCDC-USB, Sabertooth drivers, Teensy/Uno/PS2, MPU6050, customized YDLIDAR stack, robot body, batteries, BOM, current launch operations, RUDRA Voice AI v0.5 bring-up with P610/Vosk/Ollama notes, localization/TF theory, and the staged navigation/autonomy roadmap.

When the workspace changes, update the implementation in the repository first. Then update this manual only when behavior, launch procedure, interfaces, theory, diagnostics, safety limits, or wiring assumptions change. If this PDF is being tied to a specific release, update `\RepoBranch` in `manual.sty` to the matching branch, tag, or commit reference.
