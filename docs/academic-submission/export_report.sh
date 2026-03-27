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
appendix_image_dst=""
trap 'rm -f "$body_tmp" "$body_without_table_tmp" "$table_tmp"' EXIT

appendix_image_dst="$script_dir/appendix_agent_input.png"
"$script_dir/../../.venv/bin/python" "$script_dir/paper_benchmark/render_figure.py" "$appendix_image_dst"

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
printf '\n\\clearpage\n\\onecolumn\n\\appendices\n\n\\section{Reward Architecture Table}\n\\begingroup\n\\scriptsize\n\\setlength{\\tabcolsep}{2pt}\n\\renewcommand{\\arraystretch}{0.94}\n\\sloppy\n' >> "$tex_file"
cat "$table_tmp" >> "$tex_file"
printf '\n\\endgroup\n\\clearpage\n\\section{Representative Benchmark OCP Projection}\n\\begin{figure}[!htbp]\n\\centering\n\\includegraphics[width=0.95\\linewidth]{%s}\n\\caption{Representative build123d/OCP projection of the compact engineering-coder benchmark environment.}\n\\label{fig:appendix-agent-input}\n\\end{figure}\n\\end{document}\n' "$(basename "$appendix_image_dst")" >> "$tex_file"

"$tectonic_bin" "$tex_file" --outdir "$script_dir" --keep-logs

if [[ ! -f "$pdf_file" ]]; then
  echo "error: PDF was not generated at $pdf_file" >&2
  exit 1
fi
