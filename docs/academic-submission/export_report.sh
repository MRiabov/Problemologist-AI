#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
md_file="$script_dir/final_project_report.md"
tex_file="$script_dir/final_project_report.tex"
pdf_file="$script_dir/final_project_report.pdf"

author_name="Maksym Riabov"
author_email="x25194372@student.ncirl.ie"

if ! command -v pandoc >/dev/null 2>&1; then
  echo "error: pandoc is required but was not found on PATH" >&2
  exit 1
fi

tectonic_bin="${TECTONIC_BIN:-}"
if [[ -z "$tectonic_bin" ]]; then
  if command -v tectonic >/dev/null 2>&1; then
    tectonic_bin="$(command -v tectonic)"
  elif [[ -x /tmp/problemologist-tectonic-bin/tectonic/tectonic ]]; then
    tectonic_bin="/tmp/problemologist-tectonic-bin/tectonic/tectonic"
  else
    echo "error: tectonic is required but was not found on PATH or in /tmp/problemologist-tectonic-bin/tectonic/tectonic" >&2
    exit 1
  fi
fi

title="$(sed -n '1s/^# //p' "$md_file")"
body_tmp="$(mktemp)"
body_without_table_tmp="$(mktemp)"
table_tmp="$(mktemp)"
appendix_image_src=""
appendix_image_dst=""
trap 'rm -f "$body_tmp" "$body_without_table_tmp" "$table_tmp"' EXIT

for candidate in \
  "$script_dir/../../test_output/manual_zone_layout_repro/layout_render.png" \
  "$script_dir/../../test_output/manual_zone_preview/preview/preview_pitch-25_yaw35.jpg"; do
  if [[ -f "$candidate" ]]; then
    appendix_image_src="$candidate"
    appendix_image_dst="$script_dir/appendix_agent_input.png"
    python3 - "$appendix_image_src" "$appendix_image_dst" <<'PY'
from pathlib import Path
from PIL import Image
import sys

src = Path(sys.argv[1])
dst = Path(sys.argv[2])

img = Image.open(src).convert("RGB")
mask = img.convert("L").point(lambda p: 255 if p > 48 else 0)
bbox = mask.getbbox() or (0, 0, img.width, img.height)
pad = 90
left = max(0, bbox[0] - pad)
top = max(0, bbox[1] - pad)
right = min(img.width, bbox[2] + pad)
bottom = min(img.height, bbox[3] + pad)
crop = img.crop((left, top, right, bottom))
target_width = 1200
if crop.width != target_width:
    target_height = round(crop.height * target_width / crop.width)
    crop = crop.resize((target_width, target_height), Image.Resampling.LANCZOS)
crop.save(dst)
PY
    break
  fi
done

if [[ -z "$appendix_image_src" ]]; then
  echo "error: could not find a recent agent-input image in test_output" >&2
  exit 1
fi

tail -n +3 "$md_file" | pandoc -f markdown -t latex --wrap=none > "$body_tmp"
perl -0ne 'if (/(\\begin\{longtable\}.*?\\end\{longtable\})/s) { print $1 }' "$body_tmp" > "$table_tmp"

if [[ ! -s "$table_tmp" ]]; then
  echo "error: failed to extract the reward table from the markdown conversion" >&2
  exit 1
fi

perl -0pe 's/\\begin\{longtable\}.*?\\end\{longtable\}/The full reward table is provided in Appendix~A./s' "$body_tmp" > "$body_without_table_tmp"

cat > "$tex_file" <<EOF
\documentclass[journal]{IEEEtran}
\usepackage[T1]{fontenc}
\usepackage[utf8]{inputenc}
\usepackage{lmodern}
\usepackage{cite}
\usepackage{graphicx}
\usepackage{booktabs}
\usepackage{longtable}
\usepackage{array}
\usepackage{calc}
\usepackage{ragged2e}
\usepackage{url}
\usepackage[hidelinks]{hyperref}
\providecommand{\tightlist}{}
\providecommand{\subparagraph}[1]{\paragraph{#1}}
\title{$title}
\author{$author_name\\\\$author_email}
\begin{document}
\maketitle
EOF

cat "$body_without_table_tmp" >> "$tex_file"
printf '\n\\clearpage\n\\onecolumn\n\\appendices\n\n\\section{Reward Architecture Table}\n\\begingroup\n\\footnotesize\n\\setlength{\\tabcolsep}{3pt}\n' >> "$tex_file"
cat "$table_tmp" >> "$tex_file"
printf '\n\\endgroup\n\\clearpage\n\\section{Representative Agent Input}\n\\begin{figure}[!htbp]\n\\centering\n\\includegraphics[width=0.95\\linewidth]{%s}\n\\caption{Normalized input observed by the engineering agent during a recent integration test.}\n\\label{fig:appendix-agent-input}\n\\end{figure}\n\\end{document}\n' "$(basename "$appendix_image_dst")" >> "$tex_file"

"$tectonic_bin" "$tex_file" --outdir "$script_dir" --keep-logs

if [[ ! -f "$pdf_file" ]]; then
  echo "error: PDF was not generated at $pdf_file" >&2
  exit 1
fi
