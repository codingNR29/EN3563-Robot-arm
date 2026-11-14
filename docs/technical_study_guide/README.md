# GP7 Technical Study Guide (Overleaf project)

This directory contains an editable, self-contained LaTeX study guide grounded
in the `main` branch implementation, its documentation, the EN3563 task sheet,
and the recorded validation summaries. It is intended for learning and viva
preparation; it does not replace the lecturer's requested submitted report or
Gazebo video.

## Import into Overleaf

1. Zip this directory, or select all its files and folders, preserving the
   `chapters/` and `figures/` subdirectories.
2. In Overleaf, choose **New Project → Upload Project** and upload the ZIP.
3. Set `main.tex` as the main document if Overleaf does not select it
   automatically.
4. Select **pdfLaTeX** (LuaLaTeX should also work) and compile. The bibliography
   uses BibTeX; Overleaf normally runs it automatically. If references show as
   question marks on the first pass, compile again.

All diagrams are native TikZ/PGFPlots and compile in Overleaf. No image
conversion, shell escape, absolute path, or external script is required. The
repo's `docs/coordinate_frames.svg` is included separately as a source
reference; the LaTeX guide draws its own equivalent frame schematic in TikZ.

## File map

- `main.tex` — cover, contents, lists, chapter includes, references.
- `chapters/` — sixteen chapters and appendices.
- `figures/` — editable TikZ figure fragments.
- `references.bib` — official external documentation cited by the guide.

The group-member fields in Chapter 1 are intentionally blank placeholders.
