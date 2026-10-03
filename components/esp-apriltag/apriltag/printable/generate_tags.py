"""Generate A4 test tags from this repository's tag16h5 family.

Requires reportlab. Run: python3 generate_tags.py
The 120 mm black border matches TAG_SIZE in main/at_detect.c.
"""

from pathlib import Path
import re

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


def load_patterns():
    source = (Path(__file__).resolve().parent.parent / "tag16h5.c").read_text()
    source = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)
    table = re.search(r"codedata\[\d+\]\s*=\s*\{(.*?)\};", source, re.S).group(1)
    codes = [int(value, 16) for value in re.findall(r"0x[0-9a-fA-F]+", table)]
    coordinates = {}
    for axis in ("x", "y"):
        coordinates[axis] = dict((int(i), int(v)) for i, v in
                                re.findall(rf"tf->bit_{axis}\[(\d+)\]\s*=\s*(\d+);", source))
    assert "tf->width_at_border = 6;" in source
    assert "tf->total_width = 8;" in source
    assert len(coordinates["x"]) == len(coordinates["y"]) == 16
    patterns = []
    for code in codes:
        # Match apriltag_to_image(): white outer ring, black inner square,
        # then white data cells, most significant bit first. Row 0 is top.
        pixels = [[255 if x in (0, 7) or y in (0, 7) else 0
                   for x in range(8)] for y in range(8)]
        for i in range(16):
            if code & (1 << (15 - i)):
                pixels[coordinates["y"][i] + 1][coordinates["x"][i] + 1] = 255
        patterns.append(pixels)
    return patterns


def main():
    output = Path(__file__).resolve().parent / "tag16h5_ids_00-21_120mm_A4.pdf"
    patterns = load_patterns()
    pdf = canvas.Canvas(str(output), pagesize=A4)
    pdf.setTitle("AprilTag tag16h5 - IDs 0-21 - 120 mm - A4")
    pdf.setAuthor("esp-everything-qgc-bridge")
    width, _ = A4
    for tag_id, pixels in enumerate(patterns):
        pdf.setFont("Helvetica-Bold", 22)
        pdf.drawCentredString(width / 2, 272 * mm, f"AprilTag {tag_id:02d}")
        pdf.setFont("Helvetica", 11)
        role = "Landing" if tag_id < 12 else "Navigation"
        pdf.drawCentredString(width / 2, 263 * mm, f"tag16h5  |  ID {tag_id}  |  {role}")
        pdf.drawCentredString(width / 2, 253 * mm, "Print on A4 at 100% / Actual size. Disable Fit to page.")
        pdf.drawCentredString(width / 2, 246 * mm, "Black outer square: 120 x 120 mm. Keep the white margin.")

        cell = 20 * mm  # six cells across the black border = 120 mm
        left, bottom = (width - 8 * cell) / 2, 65 * mm
        pdf.setFillColorRGB(0, 0, 0)
        for row, values in enumerate(pixels):
            for col, value in enumerate(values):
                if value == 0:
                    pdf.rect(left + col * cell, bottom + (7 - row) * cell,
                             cell, cell, fill=1, stroke=0)

        pdf.setFont("Helvetica", 10)
        pdf.drawCentredString(width / 2, 49 * mm, "Check print scale: the line below must measure 100 mm.")
        start, end, y = width / 2 - 50 * mm, width / 2 + 50 * mm, 40 * mm
        pdf.setLineWidth(0.5)
        pdf.line(start, y, end, y)
        for x in (start, end):
            pdf.line(x, y - 2 * mm, x, y + 2 * mm)
        pdf.setFont("Helvetica", 9)
        pdf.drawCentredString(width / 2, 32 * mm, "100 mm")
        pdf.drawCentredString(width / 2, 16 * mm,
                             f"Repository tag16h5.c code table  |  Page {tag_id + 1} of {len(patterns)}")
        pdf.showPage()
    pdf.save()
    print(output)


if __name__ == "__main__":
    main()
