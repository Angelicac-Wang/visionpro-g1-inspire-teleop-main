#!/usr/bin/env python3
"""Generate PDF from docs/PROGRESS_REPORT_zh.md (plain-text subset, CJK-safe)."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MD_PATH = ROOT / "docs" / "PROGRESS_REPORT_en.md"
PDF_PATH = ROOT / "docs" / "PROGRESS_REPORT_en.pdf"
FONT_PATH = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
if not FONT_PATH.exists():
    FONT_PATH = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Light.ttc")


def strip_md(line: str) -> str:
    line = line.rstrip()
    if line.startswith("#"):
        line = re.sub(r"^#+\s*", "", line)
    line = re.sub(r"\*\*(.+?)\*\*", r"\1", line)
    line = re.sub(r"`([^`]+)`", r"\1", line)
    return line


def parse_blocks(text: str) -> list[tuple[str, str]]:
    """Return list of (style, text). style: title | h1 | h2 | body | code | table."""
    blocks: list[tuple[str, str]] = []
    in_code = False
    for raw in text.splitlines():
        line = raw.rstrip()
        if line.strip().startswith("```"):
            in_code = not in_code
            continue
        if in_code:
            blocks.append(("code", line))
            continue
        if not line.strip():
            blocks.append(("body", ""))
            continue
        if line.startswith("# "):
            blocks.append(("title", strip_md(line)))
        elif line.startswith("## "):
            blocks.append(("h1", strip_md(line)))
        elif line.startswith("### "):
            blocks.append(("h2", strip_md(line)))
        elif line.startswith("|") and "---" not in line:
            blocks.append(("table", strip_md(line)))
        elif line.startswith("| ---"):
            continue
        else:
            blocks.append(("body", strip_md(line)))
    return blocks


def sanitize(text: str) -> str:
    text = text.replace("✅", "[完成]").replace("🔄", "[进行中]")
    text = text.replace("→", "->").replace("↘", "->")
    return text


def main() -> int:
    try:
        from fpdf import FPDF
    except ImportError:
        print("Install fpdf2: pip install fpdf2", file=sys.stderr)
        return 1

    if not MD_PATH.exists():
        print(f"Missing {MD_PATH}", file=sys.stderr)
        return 1
    if not FONT_PATH.exists():
        print(f"CJK font not found: {FONT_PATH}", file=sys.stderr)
        return 1

    text = MD_PATH.read_text(encoding="utf-8")
    blocks = parse_blocks(text)

    pdf = FPDF(format="A4")
    pdf.set_margins(18, 18, 18)
    pdf.set_auto_page_break(auto=True, margin=18)
    pdf.add_page()
    pdf.add_font("Noto", "", str(FONT_PATH))
    pdf.add_font("Noto", "B", str(FONT_PATH))
    usable_w = pdf.w - pdf.l_margin - pdf.r_margin

    def write_line(style: str, content: str) -> None:
        content = sanitize(content)
        if style == "title":
            pdf.ln(4)
            pdf.set_font("Noto", "B", 16)
            pdf.multi_cell(usable_w, 9, content)
            pdf.ln(2)
        elif style == "h1":
            pdf.ln(3)
            pdf.set_font("Noto", "B", 13)
            pdf.multi_cell(usable_w, 8, content)
            pdf.ln(1)
        elif style == "h2":
            pdf.ln(2)
            pdf.set_font("Noto", "B", 11)
            pdf.multi_cell(usable_w, 7, content)
        elif style == "code":
            pdf.set_font("Noto", "", 8)
            pdf.set_text_color(40, 40, 40)
            pdf.multi_cell(usable_w, 5, content)
            pdf.set_text_color(0, 0, 0)
        elif style == "table":
            pdf.set_font("Noto", "", 7)
            pdf.multi_cell(usable_w, 5, content)
        else:
            if content == "":
                pdf.ln(3)
                return
            pdf.set_font("Noto", "", 10)
            pdf.multi_cell(usable_w, 6, content)

    for style, content in blocks:
        write_line(style, content)

    PDF_PATH.parent.mkdir(parents=True, exist_ok=True)
    pdf.output(str(PDF_PATH))
    print(f"Wrote {PDF_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
