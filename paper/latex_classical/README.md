# Classical-controller paper (PFB)

A second, separate paper on the Protective-Field Barrier (PFB) that keeps only the classical local controllers
(B1 pure pursuit, B2 regulated pure pursuit, B3 holonomic DWA), called field-blind because none accounts for the certified fields; B3 is the lead case. The field-aware MPPI controller B6 is not part of this paper. Learned planners (SAC, L1 to L5) appear only
as future work (Section 8), and no learned-run result is cited. Built on the PRIME arXiv template (`PRIMEarxiv.sty`,
`unsrt` bibliography). It was derived from `paper/latex/` (the full paper), which is unchanged.

## Files
- `main.tex`: preamble and front matter, then one `\input` per section.
- `sections/01_introduction.tex` to `09_conclusion.tex`: `08_future_work.tex` holds the learned-planner outlook. Section 6.2 reports HF-c on the sealed test set from the decision log of 2026-11-08; placeholders remain for the constant-margin comparator (6.4).
- `references.bib`: copied unchanged from `paper/latex/`; entries only cited by the full paper are kept and unused.
- `figures/`: figures 2 to 4, the motivation figure and the flowchart are copied from `paper/latex/figures/`;
  `fig1_architecture.pdf` is redrawn for classical controllers only.
- `figure_scripts/fig1_architecture_classical.py`: script for that figure. It imports `paper/draft/figures/scripts/style.py`.
  From the repository root (PowerShell): `$env:FIG_OUT="paper/latex_classical/figures"; python paper/latex_classical/figure_scripts/fig1_architecture_classical.py`.

## Build (PowerShell, from this folder)
```powershell
pdflatex main.tex
bibtex main
pdflatex main.tex
pdflatex main.tex
```

## Conventions
- `\tbd{...}` (red) marks a result or text that does not exist yet; `\citeq{...}` (red) marks a reference that is not saved.
- All classical results are exploratory: validation sets, seed 0, one run each. HF-c on the sealed test set is pending.
- Author names, affiliation and e-mail in `main.tex` are placeholders.
- Equations are numbered by hand (`\tag`), renumbered 1 to 15 after the learned-planner equations were removed.
